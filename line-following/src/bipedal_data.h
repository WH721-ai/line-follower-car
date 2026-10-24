/**
 * bipedal_data.h —— 机器人共享数据结构
 *
 * 提炼自课程第六~九课的同名头文件（第五课原工程缺失此文件，导致编译必失败）。
 * 第一版只保留双轮控制所需的部分：舵机通道定义 + 位姿/运动指令两个结构体。
 * 腿部逆解相关的结构体（IKparam/coordinate/controlparam 等）暂不搬入，
 * 后续需要腿控时再从第九课补回。
 */
#ifndef _BIPEDAL_DATA_h
#define _BIPEDAL_DATA_h

// 限幅函数（SimpleFOC 风格）
#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

// PCA9685 舵机通道分配：左前 / 左后 / 右前 / 右后
#define LFSERVO_CH 3
#define LRSERVO_CH 4
#define RFSERVO_CH 5
#define RRSERVO_CH 6

// 位姿反馈（IMU + 电机编码器融合后的车体状态）
typedef struct
{
    float pitch;    // 俯仰角 rad（本机摆放映射后：前倾为正，需上台架核对符号）
    float roll;     // 横滚角 rad
    float yaw;      // 偏航角 rad（纯积分，会漂移，仅作调试参考，不进控制环）
    float GyroX;    // 陀螺仪角速度 rad/s
    float GyroY;
    float GyroZ;    // 偏航角速度 rad/s（偏航率环反馈。本映射下：车头左转 → 正值）
    float height;   // 腿高（第一版不用，恒为 0）
    float speedAvg; // 左右轮平均线速度（轮速平均）
} robotposeparam;

// 运动指令（上层给定的目标量）
// 注意 v2.0.0 起的单位约定：forward 为车体线速度 [m/s]，turn 为车体偏航角速度 [rad/s]
// （v1.0.0 曾沿用课程的"电机 rad/s"，改为物理量后可直接用 rw/lw 做差速运动学换算）
typedef struct
{
    uint8_t mode;        // 预留
    float forward;       // 目标车体线速度 [m/s]（正 = 前进）
    float turn;          // 目标偏航角速度 [rad/s]（正 = 左转，需先用 'g' 命令验证符号）
    float updown;        // 腿高指令（第一版不用）
    float roll;          // 滚转指令（第一版不用）
    float throttleLimit; // 油门限制（预留）
    float turnLimit;     // 转向限制（预留）
    float forwardLimit;  // 前后倾限制（预留）
    float rollLimit;     // 侧倾限制（预留）
    uint8_t heightest;   // 高度上限（预留）
    uint8_t lowest;      // 高度下限（预留）
} robotmotionparam;

#endif // _BIPEDAL_DATA_h
