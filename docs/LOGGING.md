# 板载记录仪怎么实现

> ⚠️ **本仓库的参赛固件没有内置记录仪。** 这一页是设计文档 + 踩坑记录，
> 不是"已实现的功能说明"。
>
> **但它是你读完这个仓库后最该先做的一件事。** 因为我们所有真正的进展，
> 都是在"能看到逐帧数据"之后才发生的；在那之前的两天，全是在瞎调。

---

## 为什么一定要板载记录

| 方案 | 问题 |
|---|---|
| 靠串口实时打印 | 921600 波特也撑不住 200 Hz × 20 字段；而且 `printf` 阻塞主循环，控制环会被拖垮 |
| 靠电脑录串口 | 车跑起来会拽线；一旦掉串口，后半段全丢 |
| **板载环形缓冲** | 不占串口带宽，控制环只做一次内存写，跑完再慢慢倒出来 |

★ **成本极低**：3000 条 × 28 字节 = 84 KB，ESP32-S3 有 512 KB SRAM。

---

## 设计

### 数据结构

```c
struct LogRec {            // 保持 4 字节对齐，方便直接 memcpy 成 CSV
    uint32_t ms;           // 时间戳
    float    odo;          // 里程 [m]（注意：我们踩过 mm/100 的坑，见下）
    float    vCmd, vMeas;  // 指令/实测速度
    float    e;            // 横向偏差
    float    edot;         // 偏差变化率
    float    turn;         // 舵量
    float    arcff;        // 前馈
    float    gyro;         // 偏航率
    float    steer;        // 最终舵量
    float    kp, kd;       // 当前增益（★很重要：会变，必须记）
    ...
};

static LogRec  lgBuf[LG_N];      // LG_N = 3000
static uint16_t lgHead = 0;
static bool     lgFull = false;
```

### 环形写入（在控制环里，越轻越好）

```c
static inline void lgPush(...)
{
    LogRec &r = lgBuf[lgHead];
    r.ms = millis();
    r.odo = lineOdoMm * 0.001f;      // ★统一成 SI 单位，别留 mm/100 这种
    ...
    lgHead = (lgHead + 1) % LG_N;
    if (lgHead == 0) lgFull = true;
}
```

**开销**：一次取模 + 十几个浮点赋值，20 ms 一次，对 200 Hz 环可以忽略。

### 落盘触发（最关键的部分）

```c
// 只在"车停稳"之后写
static bool  lgSaved = false;
static float vStopMs = 0;

if (!lineOn) {                                  // 巡线结束了
    if (fabsf(vMeas) < 0.03f) vStopMs += dt;    // 累计停稳时间
    else                      vStopMs = 0;
}
if (!lineOn && !lgSaved && vStopMs > 1.0f && lgCount() >= 100) {
    lgSaved = true;
    lgSaveToFlash();                            // 此时已经停了，不怕饿死控制环
}
```

★ **绝不能在跑动中写 flash。** 一次擦写几十毫秒，控制环会被饿死，
车会突然抽一下——而这正好会被你记录成"莫名的抖动"，污染整份数据。

### 导出

```c
case 'D':   // 导出：RAM 有内容就导 RAM，空了（比如断电重连）就自动改读 flash
    if (lgCount() > 0) lgDumpRam();
    else               lgDumpFromFlash();
    break;

case 'E':   // 手动强制落盘（RAM 空时写一条测试记录，用来验证链路通不通）
    lgSaveToFlash();
    break;
```

★ `D` 的"RAM 空自动读 flash"很重要。因为**跑完一趟之后串口经常会掉**
（尤其 USB-CDC），你必须能在**重新上电之后**把数据导出来。

---

## 四个必踩的坑

### 坑 1 `create` 参数必须显式 true

```c
// ✗ 会失败：报 "no permits for creation"，文件根本建不出来
f = LittleFS.open("/lg.csv", "w");

// ✓ 正确
f = LittleFS.open("/lg.csv", "w", true);
```

Arduino core 2.0.x 里 `open()` 的第 3 个参数 `create` **默认是 false**，
即使 `mode = "w"` 也会被 VFS 拒绝。

### 坑 2 首次挂载必然报错，那是正常的

```
Corrupted dir pair at {0x0, 0x1}
mount failed (-84)
```

**这是"还没格式化过"的正常表现**，不是坏了。格式化一次之后就不会再报。
不要为了消掉这条日志去改代码，越改越乱。

### 坑 3 必须懒加载，不能放 `setup()`

LittleFS 第一次挂载要格式化，**花好几秒**。放在 `setup()` 里会把整个上电时序推后，
自启逻辑（"上电 1.5 s 稳 → 标零 4 s → 3 s → 发车"）全部错位。

**做法**：第一次真正需要写/读的时候才 `LittleFS.begin(true)`。

### 坑 4 `printf` 的格式符和参数个数必须对得上

```c
// ✗ 13 个格式符，只传了 12 个参数 ⇒ 最后一列打印栈上的垃圾
Serial.printf("%.0f,%.3f,%.3f,%.3f,%.1f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", a,b,c,...);

// ✓ 数一遍
```

GCC **不会报错**（`printf` 可变参数不做个数检查），所以这个 bug 会一直存在，
而且**每次最后一列的值都不一样**——你大概率会以为是传感器坏了或者内存出问题。

> 如果已经中招：分析时**只取前 N-1 列**，别去追究最后一列。

---

## CSV 输出格式建议

**第一行写表头**，这样 `pandas` / Excel 直接能读，也不用每次找字段含义：

```
ms,odo,vcmd,vmeas,e_mm,edot,turn,arcff,gyro,kp,kd,flag
```

后面每条一行。`flag` 建议编码当前状态（正常跟线 / 搜线 / 转角 / 四圆区 / 丢线），
**这是事后切分数据最有用的一列**——否则你只能靠里程猜"这段到底发生了什么"。

---

## 导出之后先看什么

不要一上来就看全部 3000 行。按这个顺序看：

| 顺序 | 看什么 | 判据 |
|---|---|---|
| 1 | 记录**总行数** | 应该是 `时长 / 20 ms`。**少很多 = 提前丢线了**，说明"车在 X 米出问题"这个描述本身是错的 |
| 2 | `flag` 列的转变点 | 确认车真的跑完了全程，而不是搜线蒙过去的 |
| 3 | 直道的 `e` 峰峰值 | 应该是 0 或 ±1 格 |
| 4 | 弧内的 `e` 峰峰值 | 和 `e_req` 比较（见 CONTROL_THEORY 第 2 节） |
| 5 | 出弧前后 0.3 s 的 `e` | **最容易出问题的地方** |
| 6 | `gyro` 的积分 | 每个转角到底转了多少度 |

> ★ **第 6 项最重要也最容易被忽略**。我们就是靠这一项发现：
> 那份"全程跑通"的基线里，R10 只积分了 +29°（应该是 −90°）、R11 只 −32°（应该 −180°）。
> 也就是说那两个最难的弯**从来没有真正转过去过**，是丢线后在搜线过程里蒙过去的。
> 如果没发现这件事，后面所有针对 R10 的"优化"都是建立在错误前提上的。

---

## 一个最小的可运行版本

如果你只想先拿到数据，不需要完整实现，这样可以：

```c
#define LG_N 1500

struct { uint32_t ms; float odo, e, edot, turn, arcff, gyro; } lg[LG_N];
static uint16_t head = 0;

// 控制环末尾：
lg[head].ms = millis();
lg[head].odo = lineOdoMm * 0.001f;
lg[head].e = lineE;
lg[head].edot = lineEDot;
lg[head].turn = robotMotion.turn;
lg[head].arcff = lineArcFF;
lg[head].gyro = robotPose.GyroZ;
head = (head + 1) % LG_N;

// 串口命令 'D'：从头导出成 CSV
for (int i = 0; i < LG_N; i++) {
    int k = (head + i) % LG_N;
    Serial.printf("%u,%.3f,%.2f,%.3f,%.3f,%.3f,%.3f\n",
                  lg[k].ms, lg[k].odo, lg[k].e, lg[k].edot,
                  lg[k].turn, lg[k].arcff, lg[k].gyro);
}
```

30 秒就能写完，1500 条 × 20 ms = 30 秒的缓冲，够跑完一段。

**先有这个，再谈 flash 落盘。** 顺序不要反。
