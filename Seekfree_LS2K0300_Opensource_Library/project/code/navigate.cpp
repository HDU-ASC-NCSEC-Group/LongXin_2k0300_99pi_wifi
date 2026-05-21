#include "zf_common_headfile.h"
#include <math.h>

// -----------------------------------------------------------------------------------
// 变量申明
uint8_t text_cnt = 0;
float text_angle = 0;

//------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------
// 函数简介     数据处理函数，小车前进避障，只需要前方点的数据
// 使用示例     data_process();
//------------------------------------------------------------------------------------

void avoid(void)
{
    volatile uint8_t i = 0;
    volatile uint8_t calculation_angle_cnt = 0;
    float angle_sum = 0;
    uint8_t flag_avoid = 0;
    static uint16_t distance = 330;          // 避障阈值 mm

    const float FRONT_ANGLE = 270.0f;        // ★ 雷达 0° 对应的方向？这里按正前方 = 270° 设定

    for(i = 0; i < 50; i++)
    {
        uint16_t d = PointDataProcess[i].distance;
        if(d > 0 && d < distance)
        {
            float raw_angle = PointDataProcess[i].angle;

            // 1. 计算相对于正前方的偏离角（范围 -180° ~ 180°）
            float diff = raw_angle - FRONT_ANGLE;
            if(diff > 180.0f)       diff -= 360.0f;
            else if(diff < -180.0f)  diff += 360.0f;

            // 2. 累加偏离角（左侧为负，右侧为正）
            angle_sum += diff;
            calculation_angle_cnt++;
        }
    }

    if(calculation_angle_cnt == 0)          // 无障碍
    {
        Motor_Move_Straight(2500);
        flag_avoid = 0;
    }
    else
    {
        float avg_angle = angle_sum / calculation_angle_cnt;

        // avg_angle < 0 → 障碍物偏左 → 需向右转
        if(avg_angle < 0)
        {
            Motor_Spot_Right(3600);
            flag_avoid = 1;             // 右转
        }
        else                             // avg_angle > 0 → 偏右 → 左转
        {
            Motor_Spot_Left(3600);
            flag_avoid = 2;             // 左转
        }
    }

    text_cnt = calculation_angle_cnt;
    text_angle = angle_sum;
    printf("避障状态：%d\n", flag_avoid);
}

//====================================================================================

// -----------------------------------------------------------------------------------
// 函数简介     转向函数，执行转向动作
// 参数说明     
// 使用示例     
// -----------------------------------------------------------------------------------
// int16_t turn_difpwm = 0;               // 转向差速PWM值
// int16_t LeftPWM, RightPWM = 0;         // 左右电机的PWM值

// uint8_t is_angle_turning = 0;          // 是否正在执行转向动作的标志
// float target_angle_increment = 0.0f;   // 目标角度增量
// float initial_yaw = 0.0f;              // 转向开始时的初始角度

// // ------------------ 角度位置PID结构体（单级） ------------------
// Angle_Position_PID angle_pos_pid = {
//     .Kp = 1250.0f,       // 比例系数，需根据实际调试
//     .Ki = 0.0f,       // 积分系数
//     .Kd = 0.0f,       // 微分系数

//     .OutMax = 7000.0f,    // 最大差速PWM输出
//     .OutMin = -7000.0f,   // 最小差速PWM输出
// };

// // ------------------ 内部PID计算 ------------------
// static void AnglePositionPID_Update(Angle_Position_PID *pid)
// {
//     // 计算角度误差，归一化到 [-180, 180]
//     pid->error = pid->target - pid->actual;
//     while (pid->error > 180.0f)  pid->error -= 360.0f;
//     while (pid->error < -180.0f) pid->error += 360.0f;

//     // 积分项（带限幅）
//     if (pid->Ki != 0) {
//         pid->error_integral += pid->error;
//         if (pid->error_integral > pid->integral_max)
//             pid->error_integral = pid->integral_max;
//         if (pid->error_integral < -pid->integral_max)
//             pid->error_integral = -pid->integral_max;
//     }

//     // 微分项（也需归一化，防止yaw环绕时error跳变导致D项尖峰）
//     float error_diff = pid->error - pid->last_error;
//     while (error_diff > 180.0f)  error_diff -= 360.0f;
//     while (error_diff < -180.0f) error_diff += 360.0f;
//     pid->last_error = pid->error;

//     // PID输出
//     pid->output = pid->Kp * pid->error
//                 + pid->Ki * pid->error_integral
//                 + pid->Kd * error_diff;

//     // 输出限幅
//     if (pid->output > pid->OutMax) pid->output = pid->OutMax;
//     if (pid->output < pid->OutMin) pid->output = pid->OutMin;
// }

// // ------------------ 应用差速到电机 ------------------
// static void Apply_Differential_Steer(float diff_pwm)
// {
//     // 直接设置差速，同时保留平均速度（直行分量由其他任务控制）
//     turn_difpwm = (int16_t)diff_pwm;
//     LeftPWM = -turn_difpwm / 2;
//     RightPWM = turn_difpwm / 2;
// }

// // ------------------ 开始角度转向任务 ------------------
// // >0 右转（顺时针） <0 左转（逆时针）
// void Start_Angle_Turn(float angle)
// {
//     if (is_angle_turning) {
//         Stop_Angle_Turn();
//     }

//     is_angle_turning = 1;
//     target_angle_increment = angle;
//     initial_yaw = Yaw_Result;

//     // 初始化位置PID目标
//     angle_pos_pid.target = initial_yaw + angle;
//     while (angle_pos_pid.target > 180.0f)  angle_pos_pid.target -= 360.0f;
//     while (angle_pos_pid.target < -180.0f) angle_pos_pid.target += 360.0f;
//     angle_pos_pid.actual = initial_yaw;
//     angle_pos_pid.error = 0;
//     angle_pos_pid.last_error = 0;
//     angle_pos_pid.error_integral = 0;
//     angle_pos_pid.integral_max = 100.0f;   // 积分上限，可调

//     // 初始化差速为0，避免突变
//     Apply_Differential_Steer(0.0f);
// }

// // ------------------ 停止角度转向任务 ------------------
// void Stop_Angle_Turn(void)
// {
//     is_angle_turning = 0;
//     angle_pos_pid.error_integral = 0;       // 清空积分
//     angle_pos_pid.output = 0.0f;           // 清空输出
//     Apply_Differential_Steer(0.0f);         // 差速回零，停止转向
// }

// // ------------------ 转向任务更新（中断中周期调用） ------------------
// uint8_t Update_Angle_Turn(void)
// {
//     if (!is_angle_turning) return 1;

//     // 更新当前角度
//     angle_pos_pid.actual = Yaw_Result;

//     // 计算位置PID
//     AnglePositionPID_Update(&angle_pos_pid);

//     // 位置环输出直接作为差速PWM，负号用于匹配方向（可现场调整）
//     float diff_output = -angle_pos_pid.output;   // 负号取决于你的电机接线
//     Apply_Differential_Steer(diff_output);

//     // 检查是否到达目标角度（带死区）
//     float angle_error = fabsf(angle_pos_pid.target - Yaw_Result);
//     if (angle_error < 1.0f) {   // 1度误差内认为完成
//         Stop_Angle_Turn();
//         return 1;               // 转向完成
//     }

//     return 0;                   // 转向中
// }

// // ------------------ 状态查询 ------------------
// uint8_t Is_Angle_Turning(void)
// {
//     return is_angle_turning;
// }

// float Get_Angle_Turn_Error(void)
// {
//     return angle_pos_pid.error;
// }

//====================================================================================

//------------------------------------------------------------------------------------
// 函数简介     UWB 跟随函数，基于方位角差速转向 + 距离门控
// 核心思路     对齐商家 STM32 代码 DriveProcess()：Azimuth<0 左侧减速左转，
//              Azimuth>=0 右侧减速右转，Distance>阈值停车，Distance≤阈值前进
// 使用示例     在 pit_callback_10ms() 中 uwb_usart_task() 之后调用 uwb_follow()
// 备注信息     电机 1/2 = 左侧（共用 PWM），电机 3/4 = 右侧（共用 PWM）
//              PWM 范围 -10000~10000，正值前进、负值后退
//------------------------------------------------------------------------------------
void uwb_follow(void)
{
    static uint32_t last_frame_count = 0;
    static uint32_t stale_calls = 0;   // 连续无新帧的调用次数（每次=50ms）

    // ---- 检测是否有新帧到达 ----
    if (g_uwb_frame_count != last_frame_count)
    {
        last_frame_count = g_uwb_frame_count;
        stale_calls = 0;
    }
    else
    {
        stale_calls++;
    }

    // ---- 从未收到过任何 UWB 帧 → 不动 ----
    if (g_uwb_frame_count == 0)
    {
        Motor_Reset_ALL();
        return;
    }

    // ---- 超时保护：4 次调用无新帧 = 200ms → 停车 ----
    if (stale_calls >= UWB_FOLLOW_TIMEOUT_MS / UWB_FOLLOW_CALL_MS)
    {
        Motor_Reset_ALL();
        printf("[FOLLOW] timeout, no new frame for %d calls\r\n", (int)stale_calls);
        return;
    }

    // ---- 距离门控 ----
    float dist = g_uwb_data.distance_m;
    if (dist > UWB_FOLLOW_DIST_M)
    {
        Motor_Reset_ALL();
        return;
    }

    // ---- 差速转向 ----
    float azim = g_uwb_data.azimuth_f;

    int pwmL = UWB_FOLLOW_BASE_SPEED;
    int pwmR = UWB_FOLLOW_BASE_SPEED;

    int steer = (int)(std::fabs(azim) * UWB_FOLLOW_STEER_COEFF);

    if (azim < 0.0f)
    {
        pwmR = UWB_FOLLOW_BASE_SPEED - steer;
    }
    else
    {
        pwmL = UWB_FOLLOW_BASE_SPEED - steer;
    }

    // 下限钳位（PWM 不为负）
    if (pwmL < 0) pwmL = 0;
    if (pwmR < 0) pwmR = 0;

    // 输出
    Motor_Set(1, pwmL);
    Motor_Set(2, pwmL);
    Motor_Set(3, pwmR);
    Motor_Set(4, pwmR);

    printf("[FOLLOW] dist=%.2fm azim=%.1f steer=%d L=%d R=%d\r\n",
           dist, azim, steer, pwmL, pwmR);
}




// //------------------------------------------------------------------------------------
// // 函数简介     UWB 信标跟随 + 雷达避障
// // 使用示例     在 10ms 定时中断中调用 uwb_follow();
// // 备注信息
// //   Motor_Set 输入范围: -10000 ~ 10000
// //   跟随采用差速转向（完全参照商家 Followingcar1.2 原始值方案）：
// //   - raw_azi 还原自帧内 s16 azimuth 原始值
// //   - 标签在左侧 (raw_azi < 0) → 左轮减速 → 车向左转
// //   - 标签在右侧 (raw_azi > 0) → 右轮减速 → 车向右转
// //   - 差分量 = |raw_azi| × 50（= 商家 abs(Azimuth) × Dis）
// //   避障优先级 > 跟随优先级
// //------------------------------------------------------------------------------------
// 避障+跟随函数
// void uwb_process(void)
// {
//     static uint16_t  uwb_timeout_cnt   = 0;
//     static uint32_t  last_frame_count  = 0;
//     static uint8_t   obs_dbg_tick      = 0;

//     int16_t pwm_left  = 0;
//     int16_t pwm_right = 0;

//     // ---- 读取 UWB 数据 ----
//     float  dist_m   = g_uwb_data.distance_m;           // 滤波后距离 (米)
//     int32  dist_raw = g_uwb_data.distance;              // 原始距离 (mm)
//     float  raw_azi  = g_uwb_data.azimuth_deg * 100.0f; // 还原帧内原始 s16 值

//     // ---- UWB 数据刷新与超时检测 ----
//     if (g_uwb_frame_count != last_frame_count) {
//         last_frame_count = g_uwb_frame_count;
//         uwb_timeout_cnt = 0;
//     } else {
//         uwb_timeout_cnt++;
//     }

//     if (uwb_timeout_cnt > FOLLOW_UWB_TIMEOUT_CNT) {
//         Motor_Reset_ALL();
//         if (uwb_timeout_cnt == FOLLOW_UWB_TIMEOUT_CNT + 1) {
//             printf("[FOLLOW] UWB 超时，已停机\r\n");
//         }
//         return;
//     }

//     // ---- 距离边界检查 ----
//     if (dist_m > FOLLOW_MAX_DIST_M || dist_m < FOLLOW_MIN_DIST_M || dist_raw == 0) {
//         Motor_Reset_ALL();
//         return;
//     }

//     // ---- 雷达障碍物检测（前方 220°~320°）----
//     uint8_t  obs_near       = 0;
//     uint8_t  obs_far        = 0;
//     float    obs_angle_sum  = 0;
//     uint8_t  obs_level      = 0;   // 0=无, 1=常规绕行, 2=紧急避障
//     uint16_t obs_min_dist   = 9999;

//     for (int i = 0; i < 50; i++) {
//         uint16_t d = PointDataProcess[i].distance;
//         if (d == 0) continue;

//         float a = PointDataProcess[i].angle;
//         float a_rel = a - RADAR_FORWARD_ANGLE;
//         if (a_rel > 180.0f)       a_rel -= 360.0f;
//         else if (a_rel < -180.0f) a_rel += 360.0f;

//         // 只检测前方 ±50° = 雷达 220°~320°
//         if (fabsf(a_rel) > FOLLOW_OBS_ANGLE_RANGE) continue;

//         if (d < obs_min_dist) obs_min_dist = d;

//         if (d < FOLLOW_OBS_NEAR_MM) {
//             obs_level = 2;
//             obs_near++;
//             obs_angle_sum += a_rel;
//         }
//         else if (d < FOLLOW_OBS_AVOID_MM) {
//             if (obs_level < 2) obs_level = 1;
//             obs_far++;
//             obs_angle_sum += a_rel;
//         }
//     }

//     uint8_t total_obs = obs_near + obs_far;

//     // ---- 运动决策（优先级：紧急避障 > 常规绕行 > UWB 跟随）----

//     if (obs_level == 2) {
//         // === 紧急避障：< 200mm，原地转向 ===
//         float obs_avg = (total_obs > 0) ? (obs_angle_sum / total_obs) : 0;

//         if (obs_avg > 0) {
//             pwm_left  = -(FOLLOW_BASE_SPEED / 2);
//             pwm_right = FOLLOW_BASE_SPEED;
//         } else {
//             pwm_left  = FOLLOW_BASE_SPEED;
//             pwm_right = -(FOLLOW_BASE_SPEED / 2);
//         }

//         printf("[FOLLOW] !!紧急 d_min=%umm near=%d far=%d avg=%.1f° L=%d R=%d\r\n",
//                obs_min_dist, obs_near, obs_far, obs_avg, pwm_left, pwm_right);

//     } else if (obs_level == 1) {
//         // === 常规避障：200~330mm，减速绕行 ===
//         float obs_avg = obs_angle_sum / total_obs;

//         if (obs_avg > 0) {
//             pwm_left  = FOLLOW_BASE_SPEED;
//             pwm_right = FOLLOW_BASE_SPEED / 3;
//         } else {
//             pwm_left  = FOLLOW_BASE_SPEED / 3;
//             pwm_right = FOLLOW_BASE_SPEED;
//         }

//         printf("[FOLLOW] !绕行 d_min=%umm far=%d avg=%.1f° L=%d R=%d\r\n",
//                obs_min_dist, obs_far, obs_avg, pwm_left, pwm_right);

//     } else {
//         // === 纯 UWB 跟随（商家原始值差速方案）===

//         if (fabsf(raw_azi) < FOLLOW_RAW_DEADZONE) {
//             pwm_left  = FOLLOW_BASE_SPEED;
//             pwm_right = FOLLOW_BASE_SPEED;
//         }
//         else if (raw_azi < 0) {
//             // 标签在左侧 → 左轮减速
//             int16_t diff = (int16_t)(fabsf(raw_azi) * FOLLOW_DIS_GAIN);
//             if (diff > FOLLOW_BASE_SPEED) diff = FOLLOW_BASE_SPEED;
//             pwm_left  = FOLLOW_BASE_SPEED - diff;
//             pwm_right = FOLLOW_BASE_SPEED;
//         }
//         else {
//             // 标签在右侧 → 右轮减速
//             int16_t diff = (int16_t)(fabsf(raw_azi) * FOLLOW_DIS_GAIN);
//             if (diff > FOLLOW_BASE_SPEED) diff = FOLLOW_BASE_SPEED;
//             pwm_left  = FOLLOW_BASE_SPEED;
//             pwm_right = FOLLOW_BASE_SPEED - diff;
//         }

//         // 跟随模式：每 10 次 (100ms) 打印
//         static uint8_t follow_tick = 0;
//         if (++follow_tick >= 10) {
//             follow_tick = 0;
//             printf("[FOLLOW] d=%.1fm raw=%.0f(%.1f°) L=%d R=%d\r\n",
//                    dist_m, raw_azi, g_uwb_data.azimuth_deg, pwm_left, pwm_right);
//         }
//     }

//     // ---- PWM 限幅 [-10000, 10000] ----
//     #define PWM_LIMIT 10000
//     if (pwm_left  >  PWM_LIMIT) pwm_left  =  PWM_LIMIT;
//     if (pwm_left  < -PWM_LIMIT) pwm_left  = -PWM_LIMIT;
//     if (pwm_right >  PWM_LIMIT) pwm_right =  PWM_LIMIT;
//     if (pwm_right < -PWM_LIMIT) pwm_right = -PWM_LIMIT;

//     // ---- 输出到电机 ----
//     // 左前1  右前2 / 左后4  右后3
//     // PWM共用：1和4→MOTOR1_PWM，2和3→MOTOR2_PWM
//     Motor_Set(1, pwm_left);
//     Motor_Set(4, pwm_left);
//     Motor_Set(2, pwm_right);
//     Motor_Set(3, pwm_right);

//     // ---- 雷达诊断打印（每 2 秒一次）----
//     if (++obs_dbg_tick >= 200) {
//         obs_dbg_tick = 0;
//         int radar_pts = 0;
//         for (int i = 0; i < 50; i++) {
//             if (PointDataProcess[i].distance > 0) radar_pts++;
//         }
//         if (radar_pts > 0) {
//             printf("[RADAR] %d valid points, front(min)=%dmm\r\n",
//                    radar_pts, obs_min_dist < 9999 ? obs_min_dist : 0);
//         } else {
//             printf("[RADAR] NO valid points! Check LiDAR connection.\r\n");
//         }
//     }
// }
