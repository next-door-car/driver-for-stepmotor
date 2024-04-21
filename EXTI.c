/**
* @brief  速度决策函数
* @note   在中断中使用，每进一次中断，决策一次
* @param  None
* @retval None
*/
// 运行状态：ACCEL（加速）、RUN（恒速）、DECEL（减速）

void speed_decision(void)
{
    uint32_t tim_count = 0;
    uint32_t tmp = 0;

    // 保存新（下）一个延时周期
    uint16_t new_step_delay = 0;

    // 加速过程中最后一次延时（脉冲周期）.
    static uint16_t last_accel_delay = 0;

    // 总移动步数计数器
    static uint32_t step_count = 0;

    static int32_t rest = 0;

    // 定时器使用翻转模式，需要进入两次中断才输出一个完整脉冲
    static uint8_t i = 0;

    // 检查定时器 CCx 中断是否触发
    if (TIM_GetITStatus(TIM2, MOTOR_TIM_IT_CCx) != RESET)
    {
        // 清除定时器 CCx 中断标志
        TIM_ClearITPendingBit(TIM2, MOTOR_TIM_IT_CCx);

        // 获取当前定时器计数值
        tim_count = TIM_GetCounter(TIM2);

        // 设置比较值，用于产生下个脉冲
        tmp = tim_count + srd.step_delay;
        TIM_SetCompare(TIM2, TIM_CHANNEL_2, tmp); // 修改为使用TIM2的第2通道（CH2）

        // 增加定时器中断次数计数值
        i++;

        // 如果已进入两次中断（即输出一个完整脉冲）
        if (i == 2)
        {
            // 重置定时器中断次数计数值
            i = 0;

            // 根据电机运行状态进行相应处理
            switch (srd.run_state)
            {
                // 步进电机停止状态
                case STOP:
                    // 清零步数计数器
                    step_count = 0;

                    // 清零余值
                    rest = 0;

                    // 关闭定时器通道
                    TIM_CCxCmd(TIM2, TIM_CHANNEL_2, TIM_CCx_Disable); // 修改为关闭TIM2的第2通道（CH2）

                    // 清除定时器 CCx 中断标志
                    TIM_ClearITPendingBit(TIM2, MOTOR_TIM_FLAG_CCx);

                    // 更新运行状态为停止
                    status.running = FALSE;
                    break;

                // 步进电机加速状态
                case ACCEL:
                    // 增加总移动步数
                    step_count++;

                    // 增加加速计数器
                    srd.accel_count++;

                    // 计算新(下)一步脉冲周期(时间间隔)
                    new_step_delay = srd.step_delay - (((2 * srd.step_delay) + rest) / (4 * srd.accel_count + 1));

                    // 计算余数，用于减少误差
                    rest = ((2 * srd.step_delay) + rest) % (4 * srd.accel_count + 1);

                    // 检查是否应开始减速
                    if (step_count >= srd.decel_start)
                    {
                        // 更新加速计数器为减速值
                        srd.accel_count = srd.decel_val;

                        // 切换到减速状态
                        srd.run_state = DECEL;
                    }
                    // 检查是否达到期望的最大速度
                    else if (new_step_delay <= srd.min_delay)
                    {
                        // 记录最后一次加速的延时
                        last_accel_delay = new_step_delay;

                        // 将新步长设置为最小延时
                        new_step_delay = srd.min_delay;

                        // 清零余值
                        rest = 0;

                        // 切换到最大速度运行状态
                        srd.run_state = RUN;
                    }
                    break;

                // 步进电机最大速度运行状态
                case RUN:
                    // 增加总移动步数
                    step_count++;

                    // 将新步长设置为最小延时
                    new_step_delay = srd.min_delay;

                    // 检查是否需要开始减速
                    if (step_count >= srd.decel_start)
                    {
                        // 更新加速计数器为减速值
                        srd.accel_count = srd.decel_val;

                        // 使用最后一次加速的延时作为开始减速的延时
                        new_step_delay = last_accel_delay;

                        // 切换到减速状态
                        srd.run_state = DECEL;
                    }
                    break;

                // 步进电机减速状态
                case DECEL:
                    // 增加总移动步数
                    step_count++;

                    // 增加加速计数器
                    srd.accel_count++;

                    // 计算新(下)一步脉冲周期(时间间隔)
                    new_step_delay = srd.step_delay - (((2 * srd.step_delay) + rest) / (4 * srd.accel_count + 1));

                    // 计算余数，用于减少误差
                    rest = ((2 * srd.step_delay) + rest) % (4 * srd.accel_count + 1);

                    // 检查是否为最后一步
                    if (srd.accel_count >= 0)
                    {
                        // 切换到停止状态
                        srd.run_state = STOP;
                    }
                    break;
            }

            // 更新下一次脉冲周期时间间隔
            srd.step_delay = new_step_delay;
        }
    }
}