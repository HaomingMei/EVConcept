# Tail Light Control Board 
- This Version controls two LED boards with one PCB, while the old does individual control
- This way, we enforce synchronization between left and right tail lights

## Callback Status Update
- Find out what PWM completion updates?
    - ChannelState?
    - HDMA->state
    - Each hdma (DMA_HandleTypeDef) channel is assigned to a void* parent to the peripheral it's responsible for, in our case htim3 channel1
    - Dataflow: MX_TIM3_Init -> HAL_TIM_PWM_INIT -> __HAL_LINKDMA 
        - Links DMA channel to TIMx + channel y and the other way around too
    - Difference between TIM3_CH1/TRIG and TIM3_CH4/UP 
        - TIM3_CH1/TRIG: Triggers the DMA stream upon CNT == CCR1 or the trigger event
        - TIM3_CH4/UP: Triggers the DMA stream upon CNT == CCR4 or when CNT overflows
        - We cannot disable UP, so the two event will not be sync
        - Confusion: Trigger -> Sends 1 element??
            - When CNT == CCR? but the old code never set that so I am confused
        - Fixed: Used a different channel that only has TIM3_CH3 on its own
        - TRIG will not happen since slave/trig source is not configured