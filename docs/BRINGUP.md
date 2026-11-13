# Bench bring-up (first power on real hardware)

Nothing here has been run on a board yet. Do these in order, with a current-limited supply (start at
12 V, 1 A limit) and the motor unloaded.

1. **Pin map.** Check every line of `port/stm32g431/board.h` against your schematic: PWM pins and their
   alternate functions, op-amp inputs and ADC channels, hall and encoder pins, USART2 TX/RX/DE.
2. **Clock and timebase.** Flash, attach a debugger, confirm SYSCLK = 170 MHz (MCO pin or TIM2 count vs a
   stopwatch) and that the ADC IRQ fires at 20 kHz (breakpoint counter or a GPIO toggle).
3. **ADC offsets.** With the bridge off, read `i_off[]` after boot: each within +-150 counts of 2048, else
   the ADC_OFFSET fault latches. Check the bus-voltage reading against a multimeter (scale factor).
4. **ISR time.** Toggle a spare GPIO at the start and end of `ADC1_2_IRQHandler`; on a scope the high time
   must stay well under 50 us (target < 15 us). This is the first number to put in the README.
5. **PWM and dead time, motor disconnected.** Enable with zero command: all three phases at ~47 % duty,
   complementary outputs with 200 ns dead time, no shoot-through (scope high and low gates together).
6. **Halls and encoder.** Turn the shaft by hand: `jointctl status` position follows; halls step through
   six codes (never 000 or 111).
7. **Calibrate** (motor connected, unloaded): `jointctl calibrate 1`. R and L should be close to the
   datasheet; hall edges about 60 deg apart; `jointctl save 1`.
8. **First spin:** `jointctl spin 1 --vel 5`, then 20, then 100 rad/s. Watch phase current on a current probe:
   it should be sinusoidal.
9. **Protections, one at a time:** pull the encoder connector while spinning (ENCODER warning, keeps
   running); stop the host (DAMP after 100 ms); unplug the supply (UNDERVOLT); brake from speed on the bench
   supply (bus stays below 30 V).
10. **Bus:** two joints, `jointctl scan`, then the ROS 2 bridge at 500 Hz; check `/diagnostics`.
