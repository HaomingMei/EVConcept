# Brushed Motor Notes
## PCB Unit Testing
- Setup: Just the PCB + Programmer + Laptop
- PCB Unit Test
    - Continunity 
        - MCU
            - MCU R2 to Pad ❌ -> ✅
            - MCU DBGLED1 to Pad ❌ -> ✅
            - MCU DBGLED2 to Pad ❌ -> ✅
        - CAN Transceiver 3.3V to Pad ❌ -> ✅
        - U3 (Voltage Translator) T1 to Pad ❌ -> ✅
        - Others ✅
        - Soldered 6 NMOS ✅

- MCU Programmable?
    - See if MCU is programmable and has stable ]current draw
        - No, device not found.
        - Could be mcu issue
    - Resistance 
        - All mcu pins seem fine, maybe 2 and 3 not but I am not sure
- Relay Verfiication
    - Test all 4 possible states with 48V input

- PWM Verification: Run PWM test case to check stability
    - Run PWM Test Case

-