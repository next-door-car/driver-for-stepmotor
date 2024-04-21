#include "stm32f10x.h"

/*! \brief 以给定的步数移动步进电机
 *  通过计算加速到最大速度，以给定的步数开始减速
 *  如果加速度和减速度很小，步进电机会移动很慢，还没达到最大速度就要开始减速
 *  \param step   移动的步数 (正数为顺时针，逆数为逆时针).  就是脉冲总数
 *  \param accel  加速度,如果取值为100，实际值为100*0.01*rad/sec^2=1rad/sec^2   就是指1 rad/s^2
 *  \param decel  减速度,如果取值为100，实际值为100*0.01*rad/sec^2=1rad/sec^2
 *  \param speed  最大速度,如果取值为100，实际值为100*0.01*rad/sec=1rad/sec
 */

struct StepperState {
    // 方向：CW（顺时针）或 CCW（逆时针）
    uint8_t dir;
    // 加速度计数值，用于跟踪电机加速度状态
    int32_t accel_count;
    // 运行状态：ACCEL（加速）、RUN（恒速）、DECEL（减速）
    uint8_t run_state;
    // 步进延时，用于控制电机每一步的间隔时间
    int32_t step_delay;
    // 最小步进延时，与最大速度相关
    int32_t min_delay;
    // 开始减速时的步数
    uint32_t decel_start;
    // 减速值，用于计算减速位置
    int32_t decel_val;
};
void stepper_move_T(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed)
{
    // 假设 srd 是一个已定义的结构体，用于存储步进电机相关的状态信息
    struct StepperState srd;
    
    // 初始化结构体成员变量
    srd.dir = CW; // 或者 CCW，根据实际情况设置初始方向
    srd.accel_count = 0; // 初始化加速度计数值
    srd.run_state = ACCEL; // 或者其他状态，根据实际情况设置初始运行状态
    srd.step_delay = 0; // 初始化步进延时
    srd.min_delay = 0; // 初始化最小步进延时
    srd.decel_start = 0; // 初始化开始减速时的步数
    srd.decel_val = 0; // 初始化减速值

    // 达到最大速度时的步数.
    unsigned int max_s_lim;
    // 必须开始减速的步数(如果还没加速到达最大速度时)。
    unsigned int accel_lim;

    /* 根据步数和正负判断 */
    if (step == 0)
    {
        return; // 若步数为0，无需移动，直接返回
    }
    else if (step < 0) // 逆时针
    {
        srd.dir = CCW; // 假设 CCW 定义为逆时针方向
        step = -step; // 转换为正数便于后续计算
    }
    else // 顺时针
    {
        srd.dir = CW; // 假设 CW 定义为顺时针方向
    }

    // 输出电机方向
    MOTOR_DIR(srd.dir); // 假设 MOTOR_DIR 是一个宏或函数，用于设置电机方向

    // 如果只移动一步
    if (step == 1)
    {
        // 只移动一步
        srd.accel_count = -1;
        // 减速状态
        srd.run_state = DECEL;
        // 短延时
        srd.step_delay = 1000;
        // 配置电机为运行状态
        status.running = TRUE; // 假设 status 是一个全局或外部可见的结构体，包含电机运行状态信息
    }

    // 步数不为零才移动
    else if (step != 0)
    {
        // 假设 A_T_x10 和 T1_FREQ_148 已经定义，分别表示与电机相关的常数和频率
        srd.min_delay = (int32_t)(A_T_x10 / speed); //最小的时间间隔

        // 假设 A_SQ 已定义为与电机加速度平方相关的常数
        srd.step_delay = (int32_t)((T1_FREQ_148 * sqrt(A_SQ / accel)) / 10);  //相邻两步之间的时间间隔

        // 计算达到最大速度所需的步数
        max_s_lim = (uint32_t)(speed * speed / (A_x200 * accel / 10));

        if (max_s_lim == 0)
        {
            max_s_lim = 1; // 至少移动一步才能达到最大速度
        }

        // 计算开始减速前的步数
        accel_lim = (uint32_t)(step * decel / (accel + decel));

        if (accel_lim == 0)
        {
            accel_lim = 1; // 至少加速一步后开始减速
        }

        if (accel_lim <= max_s_lim)  //没有到设定的最大速度就必须减速
        {
            srd.decel_val = accel_lim - step;   //减速的步数的负数值(总步数)
        }
        else
        {
            srd.decel_val = -(max_s_lim * accel / decel);  //根据高相同使用比例计算
        }

        // 当只剩下一步时必须减速
        if (srd.decel_val == 0)
        {
            srd.decel_val = -1;
        }

        srd.decel_start = step + srd.decel_val;  //减速前的所有步数统计

        // 判断是否需要加速阶段
        if (srd.step_delay <= srd.min_delay)  //所需的相邻两步之间的时间间隔 小于 最小的时间间隔
        {
            srd.step_delay = srd.min_delay;
            srd.run_state = RUN;   //恒速阶段
        }
        else
        {
            srd.run_state = ACCEL;  //进入加速阶段
        }

        // 复位加速度计数值
        srd.accel_count = 0;
        status.running = TRUE; // 设置电机运行状态为真
    }

    // 获取当前计数值
    uint32_t tim_count = TIM2->CNT;

    // 在当前计数值基础上设置定时器比较值
    TIM2->CCR2 = tim_count + srd.step_delay;  //将CNT 的值加上相邻两步之间的时间间隔

    // 使能定时器通道（假设 MOTOR_PUL_CHANNEL_x 为 2）
    TIM_OC2Init(TIM2, TIM_OCMode_PWM1, TIM_OutputState_Enable, srd.step_delay, TIM_OCPolarity_High);

    // 启动电机（假设 MOTOR_EN 是一个宏或函数，用于控制电机使能）
    MOTOR_EN(ON);

    // 使能定时器
    TIM_Cmd(TIM2, ENABLE);
}