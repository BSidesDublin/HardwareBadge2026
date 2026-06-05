# Badge Hardware Reference


## Hardware

| | |
|---|---|
| MCU | STM32L053R8Tx — Cortex-M0+, 64 KB flash, 8 KB RAM |
| Touch | 6 capacitive strings (TSC) |
| LEDs | 7x yellow, PWM-controlled |
| Sound | Piezo buzzer, two octaves (D5–D7) |
| Power | CR2032 or USB, auto-detected |
| USB | USB-C, crystal-less (HSI48 + CRS) |

## Memory Map

| Region | Start | Size |
|---|---|---|
| Bootloader | `0x08000000` | 16 KB |
| Application | `0x08004000` | 48 KB |
| RAM | `0x20000000` | 8 KB |
| Data EEPROM | `0x08080000` | 2 KB |

The top 4 bytes of RAM (`0x20001FFC`) hold the DFU boot flag — shared between bootloader and application, survives soft reset, used by dfu runtime.

## Boot Sequence

1. Power on — bootloader runs from `0x08000000`
2. Checks DFU flag at `0x20001FFC` and strings 1+6 (touch)
3. If either is set: stay in DFU mode (TinyUSB DFU class)
4. Otherwise: jump to application at `0x08004000`
5. Application sets `SCB->VTOR`, re-enables interrupts (bootloader left `PRIMASK=1`), inits peripherals

## Pinout

### LEDs (PWM)

| LED | Pin | Timer / CH | | LED | Pin | Timer / CH |
|-----|------|------------|-|-----|------|------------|
| 0 | PB11 | TIM2 CH4 | | 4 | PC7 | TIM22 CH2 |
| 1 | PB13 | TIM21 CH1 | | 5 | PA15 | TIM2 CH1 |
| 2 | PB14 | TIM21 CH2 | | 6 | PB3 | TIM2 CH2 |
| 3 | PC6 | TIM22 CH1 | | Buz | PB10 | TIM2 CH3 |

TIM2's ARR is shared between the buzzer and LEDs 0, 5, 6 — changing the buzzer frequency affects those LED PWM periods.

### Touch Sensing (TSC)

| String | Pin | TSC Group | TSC IO |
|--------|-----|-----------|--------|
| 0 | PA1 | Group 1 | IO2 |
| 1 | PA2 | Group 1 | IO3 |
| 2 | PA3 | Group 1 | IO4 |
| 3 | PA5 | Group 2 | IO2 |
| 4 | PA6 | Group 2 | IO3 |
| 5 | PA7 | Group 2 | IO4 |


### SWD Test Points (J1)

5-pad array near the USB connector. Pad closest to USB receptacle is 3V3.

| Pad | 1 | 2 | 3 | 4 | 5 |
|-----|---|---|---|---|---|
| Signal | 3V3 | SWDIO | SWCLK | GND | BOOT0 |

### USB

USB-C connector with ESD protection (USBLC6-2SC6). Crystal-less — HSI48 is auto-trimmed using USB SOF packets via the Clock Recovery System (CRS).

## Key Components

| Part | Value | Designator |
|---|---|---|
| MCU | STM32L053R8T6D | U1 |
| LDO | AP2112K-3.3 (600 mA) | U2 |
| USB-C | TYPE-C-31-M-12 | J2 |
| ESD | USBLC6-2SC6 | CR1 |
| Buzzer | MLT_8530 piezo | BZ1 |
| LEDs | SM0603UYC (593 nm yellow) | D1-D7 |
| NPN | SS8050-G | Q1 |
