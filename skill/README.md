# AI 技能包：循迹车调试助手

> 把这一份东西喂给你的 AI agent，它就变成一个**懂循迹车的调试助手**——
> 不会再说"试试把 Kp 调小一点"，而是会先问你要数据、再算给你看。

## 这是什么

`line-follower-tuning/` 是一个 **Agent Skill**（技能包），格式是通用的
`SKILL.md + references/`：

```
line-follower-tuning/
├── SKILL.md                      ← 主入口：工作流 + 症状路由 + 硬性禁止事项
└── references/
    ├── diagnostics.md            ← 逐条判据 + 实测数据（症状定位时读）
    ├── math.md                   ← 公式与估算，换车时按这个重算
    ├── motor-foc.md              ← 电机层（不转/飞车/堵转/标定）
    └── checklist.md              ← 检查表
```

`SKILL.md` 的 YAML frontmatter 只有 `name` 和 `description` 两个字段，
是这个格式的通用最小集。

## 它解决的问题

这类问题**不适合**让通用模型自由发挥：

| 用户说 | 通用模型会给 | 这个技能会给 |
|---|---|---|
| "车会摆" | "试试降低 Kp 或加低通滤波" | "先给我逐帧记录。同时算 `sinθ = 限幅/v` —— 如果摆幅角度超过这个值，D 已经被钳死了，调 Kp 没用" |
| "弯里摆" | "降低过弯速度" | "算 `e_req = (v/R)/0.9007/Kp`：R292@0.42 需要 29.6 mm，而你视场只有 ±40 mm ⇒ 74%，光待着就没余量了。加前馈 `0.90·v/R`，不要降速" |
| "降速之后更差了" | "再降一点试试" | "`ζ ∝ √v`，降速让阻尼更差。唯一例外是解饱和，你这不是" |

**核心区别**：它把"调参"变成"算"。

## 怎么用

### 方式 1：WorkBuddy / Claude Code（Agent Skills）

把整个 `line-follower-tuning` 目录放到 skills 目录下即可自动加载：

```bash
# WorkBuddy（用户级，所有项目可用）
cp -r line-follower-tuning ~/.workbuddy/skills/

# 或项目级
cp -r line-follower-tuning <your-project>/.workbuddy/skills/

# Claude Code
cp -r line-follower-tuning <your-project>/.claude/skills/
```

然后直接问：

> "我的循迹车在直线上会左右画龙，帮我看看"

agent 会自动匹配到 `description` 里描述的场景并加载这个技能。

### 方式 2：任何支持"系统提示词"的客户端

把 `SKILL.md` 的正文（去掉 frontmatter）粘进系统提示词 / 自定义 instructions，
再把 `references/` 里的文件一起提供给它（或作为可检索的知识库）。

### 方式 3：直接用自然语言

不需要任何工程手段，直接把 `references/diagnostics.md` 全文贴给任何 AI：

> "这是我的循迹车调试手册，请基于它分析我的问题：___"

`diagnostics.md` 是自包含的——每一条都有判据、机理、实测数字、修法。

## 它不会做什么

- **不会替你测数据。** 没有逐帧记录时它会一直要数据，这是刻意的。
- **不会给"试试看"的建议。** 每条改动都必须有算式支撑。
- **不会移植具体数值。** 文档里的数字来自一台具体的车
  （轮径 30.09 mm / 轮距 144 mm / 6.7 mm 探头间距 / ±40 mm 视场 / 200 Hz 环）。
  换成你的车，结论照样成立，**数值必须按 `references/math.md` 重算**。

## 想改进它

这份技能包是从一次真实项目里沉淀出来的（37 m 赛道，跑了两个多月，拿了 92.5 分）。
如果你踩到了文档里没写的坑，欢迎补一条进去——**格式照抄现有条目**：
判据 / 机理 / 实测数字 / 修法，四样缺一不可。

只有"结论"没有"为什么"的条目，对下一个人的价值接近零。
