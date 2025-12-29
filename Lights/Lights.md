# Tail Light Control Board 
- This Version controls two LED boards with one PCB, while the old does individual control
- This way, we enforce synchronization between left and right tail lights

## Callback Status Update
- Find out what PWM completion updates?
    - Each hdma (DMA_HandleTypeDef) channel is assigned to a void* parent to the peripheral it's responsible for, in our case htim3 channel1
    - Dataflow: MX_TIM3_Init -> HAL_TIM_PWM_INIT -> __HAL_LINKDMA 
        - Links DMA channel to TIMx + channel y and the other way around too
    - Difference between TIM3_CH1/TRIG and TIM3_CH4/UP 
        - TIM3_CH1/TRIG: Triggers the DMA stream upon CNT == CCR1 or the trigger event
        - TIM3_CH4/UP: Triggers the DMA stream upon CNT == CCR4 or when CNT overflows
            - UP is used for burst mode where you want to update multiple channels at once
        - We might be able to disable up via TIM3's registers, but the safest option is to use a different channel that does not have UP at all
        - Confusion: Trigger -> Sends 1 element??
            - When CNT == CCR? but the old code never set that so I am confused
        - Fixed: Used a different channel that only has TIM3_CH3 on its own
        - TRIG will not happen since slave/trig source is not configured
    - Conclusion: Used TIM3_CH3 and TIM3_CH1 to Avoid Dealing with UP for the time being
- Definitions:
    - RIGHT_BLINK: Toggles Between TURN_SIG and OFF_COLOR for the RIGHT
    - LEFT_BLINK: Toggles Between TURN_SIG and OFF_CLOR for the LEFt
    - RIGHT_BLINK_FLAG: Indicates whether the next transition should be OFF_COLOR OR TURN_SIG
    - LEFT_BLINK_FLAG:  Indicates whether the next transition should be OFF_COLOR OR TURN_SIG
- Task: Which State updates when DMA finishes?
    - ChannelState?
    - HDMA->state?
    - Check: DMA Start -> DMA_START_IT
    - Found: HAL_DMA_IRQHandler sets hdma->State = HAL_DMA_STATE_READY;
    - Found: TIM_DMADelayPulseCplt (Complete Callback function called), this happens after hdma->State = HAL_DMA_STATE_READY
        - Sets  TIM_CHANNEL_STATE_SET(htim, TIM_CHANNEL_X, HAL_TIM_CHANNEL_STATE_READY)
        - Then, HAL_TIM_PWM_PulseFinishedCallback gets called
    - Conclusion: Check both in HAL_TIM_PWM_PulseFinishedCallback

- Task: uint16_t vs uint32_t for buffer size
    - Problem: Buffer has to be uint16_t to fit everything into the SRAM, but HAL_TIM_PWM_Start_DMA takes uint32_t*
    - What we have right now: DMA width is 16bit, so it should be fine
    - Each tick does CNT ==? CCR, and both are 16 bit (technically 32 bit but first 16 bit are reserved)
    - Check: Confirm whether memory access is by 16bit
        - Find out whether there is typecast in between that avoids accessing by uint32_t, we want access by uint16_t

- Task: Finish updateDash() function
