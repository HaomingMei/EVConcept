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