/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Application — CDC hello world + TSC + LEDs + buzzer
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "touchsensing.h"
#include "tsc.h"
#include "usb.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tusb.h"
#include "tsl_user.h"
#include <string.h>
#include <stddef.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define APP_BASE_ADDR  0x08004000UL
#define DFU_BOOT_FLAG  0xB00710ADUL
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint32_t dfu_boot_flag __attribute__((section(".dfu_flag")));

uint8_t evil_mode = 0;
static uint32_t pm_tsc_acquire(int bank);

static bool     usb_connected    = false;
static bool     usb_probing      = true;
static uint32_t usb_probe_start  = 0;
static uint32_t last_touch_time  = 0;

static struct {
    uint32_t baseline;    /* ECS-tracked reference (untouched count) */
    uint32_t raw;         /* last raw TSC measurement */
    int16_t  delta;       /* baseline - raw (positive = touch) */
    uint8_t  deb_detect;  /* consecutive detect samples */
    uint8_t  deb_release; /* consecutive release samples */
    uint8_t  touched;     /* debounced state: 0=released, 1=touched */
} touch_ch[6];
static uint8_t touch_mask = 0;

const char ctf_flag1[] __attribute__((used,section(".rodata"))) = "flag{str1ngs_4re_fr33}";

#define MORSE_FLAG_KEY 0x55U
static const uint8_t morse_flag_enc[] = {
    0x3d,0x34,0x27,0x25,0x38,0x34,0x26,0x21,0x30,0x27
};
#define MORSE_FLAG_LEN (sizeof(morse_flag_enc))
static const uint8_t melody_target[6] = {0, 2, 4, 5, 4, 2};
static uint8_t melody_buf[6];
static uint8_t melody_idx = 0;
static bool morse_active = false;

#define CTF4_KEY 0x55U
static const uint8_t ctf4_flag_enc[] = {
    0x33,0x39,0x34,0x32,0x2e,0x21,0x64,0x38,0x64,0x3b,
    0x32,0x0a,0x61,0x21,0x21,0x34,0x36,0x3e,0x28
};
#define CTF4_FLAG_LEN (sizeof(ctf4_flag_enc))
static const uint8_t ctf4_pin_enc[] = { 0x62,0x66,0x62,0x66 };
#define CTF4_PIN_LEN 4

static const uint8_t morse_table[36] = {
    /* A */ 0x48, /* B */ 0x90, /* C */ 0x94, /* D */ 0x70,
    /* E */ 0x20, /* F */ 0x84, /* G */ 0x78, /* H */ 0x80,
    /* I */ 0x40, /* J */ 0x8E, /* K */ 0x74, /* L */ 0x88,
    /* M */ 0x58, /* N */ 0x50, /* O */ 0x7C, /* P */ 0x8C,
    /* Q */ 0x9A, /* R */ 0x68, /* S */ 0x60, /* T */ 0x30,
    /* U */ 0x64, /* V */ 0x82, /* W */ 0x6C, /* X */ 0x92,
    /* Y */ 0x96, /* Z */ 0x98,
    /* 0 */ 0xBF, /* 1 */ 0xAF, /* 2 */ 0xA7, /* 3 */ 0xA3,
    /* 4 */ 0xA1, /* 5 */ 0xA0, /* 6 */ 0xB0, /* 7 */ 0xB8,
    /* 8 */ 0xBC, /* 9 */ 0xBE,
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ---- LED helpers ----------------------------------------------------------
 * LED index → timer channel mapping:
 *   0  TIM2_CH4   PB11
 *   1  TIM21_CH1  PB13
 *   2  TIM21_CH2  PB14
 *   3  TIM22_CH1  PC6
 *   4  TIM22_CH2  PC7
 *   5  TIM2_CH1   PA15
 *   6  TIM2_CH2   PB3
 * Buzzer: TIM2_CH3  PB10  (frequency set by ARR; duty=50% for tone)
 */
#define LED_COUNT   7
#define MAX_BRIGHT  100

static uint8_t led_duty[7];
static uint8_t max_brightness = 100;  /* 30 on battery, 100 on USB */

typedef void (*anim_fn_t)(uint32_t tick);
static uint32_t  anim_tick    = 0;
static uint8_t   idle_mode    = 0;
static uint32_t  mode_start   = 0;

static const uint16_t note_arr_low[6]  = { 27240, 24268, 21620, 18181, 16196, 13616 }; /* D5-D6 */
static const uint16_t note_arr_high[6] = { 13616, 12134, 10810,  9090,  8098,  6808 }; /* D6-D7 */
static uint16_t note_arr_custom[6];               /* user-settable frequencies */
static const uint16_t *note_arr = note_arr_low;   


#define _FH(h,c) (((h)^(uint32_t)(c))*0x01000193UL)
#define CFG_MAGIC ( \
    _FH(_FH(_FH(_FH(_FH(_FH(_FH(_FH(_FH(_FH(_FH( \
    _FH(_FH(_FH(_FH(_FH(_FH(_FH(_FH(0x811C9DC5UL, \
    __DATE__[0]),__DATE__[1]),__DATE__[2]),__DATE__[3]), \
    __DATE__[4]),__DATE__[5]),__DATE__[6]),__DATE__[7]), \
    __DATE__[8]),__DATE__[9]),__DATE__[10]), \
    __TIME__[0]),__TIME__[1]),__TIME__[2]),__TIME__[3]), \
    __TIME__[4]),__TIME__[5]),__TIME__[6]),__TIME__[7]))

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint8_t  brightness;     /* 0-100, default 55 */
    uint8_t  idle_mode;      /* 0-15, default 0 */
    uint8_t  play_mode;      /* 0-2, default 0 */
    uint8_t  scale;          /* 0=low, 1=high, default 0 */
    uint16_t speed;          /* 10-2000, x100 multiplier, default 100 (x1.00) */
    uint16_t cycle_mask;     /* bitmask of anims to cycle, 0=single, 0x7FFF=all */
    uint16_t debounce_ms;    /* 50-2000, default 400 */
    uint16_t maxhold_ms;     /* 1000-30000, default 7000 */
    uint16_t timeout_ds;     /* 10-6000, deciseconds, default 50 (5.0s) */
    uint16_t shutdown_min;   /* 0-1440, default 90 (0=disabled) */
    uint8_t  buzzer_on;      /* 0/1, default 1 */
    uint8_t  volume;         /* 0-100, default 50 */
    uint8_t  tremolo;        /* 0-5, default 0 */
    uint8_t  vibrato;        /* 0-5, default 0 */
    uint8_t  powersave;      /* 0/1, default 1 */
    uint8_t  sleep_anim;     /* 0/1, default 1 */
    uint8_t  wake_scan_anim; /* 0/1, default 1 */
    uint8_t  wake_touch_anim;/* 0/1, default 1 */
    uint16_t custom_notes[6]; /* custom ARR values, 0=use scale default */
    uint8_t  touch_detect;   /* detect threshold % of baseline (1-50, default 10) */
    uint8_t  touch_release;  /* release threshold % of baseline (1-50, default 6) */
    uint8_t  touch_min;      /* min absolute detect threshold /10 (1-255, default 20 → 200) */
    uint8_t  touch_deb;      /* per-channel debounce samples (1-10, default 4) */
    uint8_t  resolve_deb;    /* note resolve debounce samples (1-10, default 3) */
    char     name[21];       /* owner name, default "" */
    uint8_t  ctf_solved;     /* bitmask of solved challenges */
    uint8_t  evil_enabled;   /* 0/1, default 0 — requires ctf_solved & 0x08 */
    uint8_t  checksum;       /* XOR checksum */
} badge_config_t;

static badge_config_t cfg __attribute__((aligned(4)));

static uint8_t cfg_checksum(void)
{
    const uint8_t *p = (const uint8_t *)&cfg;
    uint8_t xor = 0;
    for (size_t i = 0; i < offsetof(badge_config_t, checksum); i++)
        xor ^= p[i];
    return xor;
}

static void cfg_defaults(void)
{
    memset(&cfg, 0, sizeof(cfg));
    cfg.ctf_solved  = 0x0;
    cfg.magic       = CFG_MAGIC;
    cfg.brightness  = 55;
    cfg.idle_mode   = 0;
    cfg.play_mode   = 0;
    cfg.scale       = 0;
    cfg.speed       = 200;
    cfg.cycle_mask  = 0x5FFFU;  /* all except flicker (bit 13) */
    cfg.debounce_ms = 5;
    cfg.maxhold_ms  = 7000;
    cfg.timeout_ds  = 450;     /* 45.0 seconds */
    cfg.shutdown_min= 120;
    cfg.buzzer_on   = 1;
    cfg.volume      = 100;
    cfg.tremolo     = 0;
    cfg.vibrato     = 0;
    cfg.powersave   = 1;
    cfg.sleep_anim  = 1;
    cfg.wake_scan_anim  = 1;
    cfg.wake_touch_anim = 1;
    cfg.touch_detect  = 10;
    cfg.touch_release = 6;
    cfg.touch_min     = 20;  /*200 */
    cfg.touch_deb     = 3;
    cfg.resolve_deb   = 3;
    cfg.evil_enabled  = 0;
    cfg.checksum    = cfg_checksum();
}

static void cfg_load(void)
{
    memcpy(&cfg, (const void *)DATA_EEPROM_BASE, sizeof(cfg));
    if (cfg.magic != CFG_MAGIC || cfg.checksum != cfg_checksum())
        cfg_defaults();
}

static void cfg_save(void)
{
    cfg.checksum = cfg_checksum();
    const uint32_t *src = (const uint32_t *)&cfg;
    const uint32_t *dst = (const uint32_t *)DATA_EEPROM_BASE;
    uint32_t words = (sizeof(cfg) + 3) / 4;

    /* Disable interrupts during EEPROM write — NVM is shared with code flash*/
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    HAL_FLASHEx_DATAEEPROM_Unlock();
    for (uint32_t i = 0; i < words; i++) {
        if (src[i] != dst[i])
            HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD,
                DATA_EEPROM_BASE + i * 4, src[i]);
    }
    HAL_FLASHEx_DATAEEPROM_Lock();
    __set_PRIMASK(primask);
}

static void cfg_apply(void)
{
    max_brightness = cfg.brightness;
    idle_mode      = cfg.idle_mode;
    /* Check if any custom notes are set */
    int has_custom = 0;
    for (int i = 0; i < 6; i++) if (cfg.custom_notes[i]) { has_custom = 1; break; }
    if (has_custom) {
        const uint16_t *base = cfg.scale ? note_arr_high : note_arr_low;
        for (int i = 0; i < 6; i++)
            note_arr_custom[i] = cfg.custom_notes[i] ? cfg.custom_notes[i] : base[i];
        note_arr = note_arr_custom;
    } else {
        note_arr = cfg.scale ? note_arr_high : note_arr_low;
    }
}

static void led_set(int led, int duty)
{
  led = 6-led;
    if (led < 0 || led > 6) return;
    if (duty > 100) duty = 100;
    if (duty < 0)   duty = 0;
    if (duty > max_brightness) duty = max_brightness;
    led_duty[led] = (uint8_t)duty;
    /* Gamma 2.0 correction for typical 0603 LEDs */
    duty = duty * duty / 100;

    switch (led) {
    case 0: TIM2->CCR4  = (uint32_t)TIM2->ARR  * (uint32_t)duty / 100; break;
    case 1: TIM21->CCR1 = TIM21->ARR * duty / 100; break;
    case 2: TIM21->CCR2 = TIM21->ARR * duty / 100; break;
    case 3: TIM22->CCR1 = TIM22->ARR * duty / 100; break;
    case 4: TIM22->CCR2 = TIM22->ARR * duty / 100; break;
    case 5: TIM2->CCR1  = (uint32_t)TIM2->ARR  * (uint32_t)duty / 100; break;
    case 6: TIM2->CCR2  = (uint32_t)TIM2->ARR  * (uint32_t)duty / 100; break;
    }
}

static void buzzer_set_note(int string)
{
    if (string < 0 || string > 5) {
        TIM2->CCR3 = 0;  /* buzzer off */
        return;
    }
    uint16_t arr = note_arr[string];
    TIM2->ARR  = arr;
    TIM2->CCR3 = (uint32_t)arr * cfg.volume / 200;
    /* Recalculate LEDs 0, 5, 6 that share TIM2 ARR (with gamma) */
    TIM2->CCR4 = (uint32_t)arr * (led_duty[0] * led_duty[0] / 100) / 100;
    TIM2->CCR1 = (uint32_t)arr * (led_duty[5] * led_duty[5] / 100) / 100;
    TIM2->CCR2 = (uint32_t)arr * (led_duty[6] * led_duty[6] / 100) / 100;
    TIM2->EGR  = TIM_EGR_UG;  /* prevents 32-bit counter wraparound glitch */
}

static const uint8_t sine_lut[16] = {
    0, 5, 10, 20, 31, 45, 59, 73, 85, 93, 98, 100, 100, 98, 93, 85
};

static const uint8_t qsin[32] = {
     0,  5, 10, 15, 20, 25, 30, 35, 40, 44, 49, 53, 58, 62, 66, 70,
    74, 77, 81, 84, 87, 89, 92, 94, 96, 97, 99,100,100,100,100,100
};
static int isin(uint8_t phase) {
    uint8_t q = phase >> 6;
    uint8_t idx = phase & 0x3F;
    if (q & 1) idx = 63 - idx;
    int val = (int)qsin[idx >> 1];
    return (q >= 2) ? -val : val;
}
static inline int isin_pos(uint8_t phase) {
    return (isin(phase) + 100) / 2;
}

static int ripple_center = -1;

typedef enum { ST_IDLE, ST_PLAYING, ST_RELEASE_DECAY } badge_state_t;
static badge_state_t badge_state = ST_IDLE;
static uint32_t state_enter_tick = 0;
static uint32_t last_string_change = 0;
static uint8_t  decay_intensity = 0;
static int prev_key = -1;
static int momentum_dir = 0;
static bool maxhold_locked = false;

static void anim_breathe(uint32_t tick) {
    (void)tick;
    uint32_t period = 4000U * 100U / cfg.speed;
    uint32_t phase = (HAL_GetTick() % period) * 256U / period;
    uint32_t idx16 = phase * 16U / 256U;
    uint32_t frac = phase * 16U % 256U;
    uint8_t a = sine_lut[idx16 % 16], b = sine_lut[(idx16 + 1) % 16];
    int bri = (int)a + ((int)b - (int)a) * (int)frac / 256;
    bri = 2 + bri * (max_brightness - 2) / 100;
    for (int i = 0; i < LED_COUNT; i++) led_set(i, bri);
}

static void anim_nb_wave(uint32_t tick) {
    (void)tick;
    uint32_t period = 3000U * 100U / cfg.speed;
    uint32_t t = HAL_GetTick() % period;
    uint32_t half = period / 2;
    int pos256 = (t < half) ? (int)(t * 6 * 256 / half) : (int)((period - t) * 6 * 256 / half);
    for (int i = 0; i < LED_COUNT; i++) {
        int d = i * 256 - pos256; if (d < 0) d = -d;
        int bri = 0;
        if (d < 128) bri = max_brightness;
        else if (d < 512) bri = max_brightness * (512 - d) / 384;
        led_set(i, bri);
    }
}

static void anim_sparkle(uint32_t tick) {
    uint32_t t = tick * cfg.speed / 100;
    for (int i = 0; i < LED_COUNT; i++) {
        uint8_t p = (uint8_t)((t + i * 37U) * 26U >> 8);
        int tri = p < 128 ? p * 2 : (255 - p) * 2;
        int bri = tri > 180 ? (tri - 180) * max_brightness / 75 : 0;
        led_set(i, 3 + bri);
    }
}

static void anim_scanner(uint32_t tick) {
    (void)tick;
    static uint8_t trail[7] = {0};
    static uint32_t last_decay = 0;
    uint32_t now = HAL_GetTick();
    uint32_t per = 2400U * 100U / cfg.speed;
    uint32_t t = now % per;
    /* 8.8 fixed-point position, bouncing 0..6 */
    int p256 = (int)(t * 12U * 256U / per);
    if (p256 > 6 * 256) p256 = 12 * 256 - p256;
    /* Decay trail at 50 Hz — decay ≈ 0.80 per step → ~200ms visible trail */
    if (now - last_decay >= 20) {
        last_decay = now;
        for (int i = 0; i < LED_COUNT; i++)
            trail[i] = (uint8_t)((uint16_t)trail[i] * 205U / 256U);
    }
    int base = p256 >> 8;
    int frac = p256 & 0xFF;
    uint8_t v0 = (uint8_t)((100U * (256U - (uint32_t)frac)) >> 8);
    uint8_t v1 = (uint8_t)((100U * (uint32_t)frac) >> 8);
    if (base < 7 && v0 > trail[base]) trail[base] = v0;
    if (base < 6 && v1 > trail[base + 1]) trail[base + 1] = v1;
    for (int i = 0; i < LED_COUNT; i++)
        led_set(i, trail[i] * max_brightness / 100);
}

static void anim_pulse(uint32_t tick) {
    (void)tick;
    uint32_t per = 3000U * 100U / cfg.speed;
    uint32_t now = HAL_GetTick();
    for (int i = 0; i < LED_COUNT; i++) {
        uint8_t phase = (uint8_t)(now * 256U / per + (uint32_t)i * 256U / LED_COUNT);
        int bri = isin_pos(phase) * max_brightness / 100;
        led_set(i, bri);
    }
}

static void anim_travel(uint32_t tick) {
    (void)tick;
    uint32_t per = 2000U * 100U / cfg.speed;
    uint32_t now = HAL_GetTick();
    for (int i = 0; i < LED_COUNT; i++) {
        /* k = 2π/3.5LEDs → 73/256 per LED, ~2 full waves across strip */
        uint8_t phase = (uint8_t)(now * 256U / per + (uint32_t)i * 73U);
        int bri = isin_pos(phase) * max_brightness / 100;
        led_set(i, bri);
    }
}

static void anim_ripple(uint32_t tick) {
    (void)tick;
    uint32_t per = 2000U * 100U / cfg.speed;
    uint32_t now = HAL_GetTick();
    uint32_t t = now % per;
    /* Radius expands: 0 → 5 LEDs in fixed-point 8.4 */
    int r16 = (int)(t * 80U / per);  /* 0..79 in 1/16 LED units */
    /* Temporal decay: amplitude drops as ripple expands */
    int amp = 100 - (int)(t * 100U / per);
    if (amp < 0) amp = 0;
    for (int i = 0; i < LED_COUNT; i++) {
        int d16 = (i - 3) * 16; if (d16 < 0) d16 = -d16;
        /* Gaussian-ish pulse at radius: exp(-|d-r|²/σ²) */
        int diff = r16 - d16;
        int bri = 0;
        if (diff > -16 && diff < 16) {
            int g = 16 - (diff < 0 ? -diff : diff);  /* 0..16 */
            bri = g * amp * max_brightness / 1600;
        }
        led_set(i, bri);
    }
}

static void anim_flicker(uint32_t tick) {
    (void)tick;
    static uint8_t flk[7] = {30,50,40,60,35,55,45};
    static uint8_t tgt[7] = {50,30,60,40,55,35,45};
    static uint16_t lfsr = 0xACE1U;
    static uint32_t last_tgt = 0, last_filt = 0;
    uint32_t now = HAL_GetTick();
    /* New random targets every ~100ms */
    uint32_t tgt_step = 100U * 100U / cfg.speed;
    if (now - last_tgt >= tgt_step) {
        last_tgt = now;
        for (int i = 0; i < LED_COUNT; i++) {
            lfsr = (uint16_t)((lfsr >> 1) ^ (-(lfsr & 1u) & 0xB400u));
            tgt[i] = (uint8_t)(lfsr % 100);
        }
    }
    if (now - last_filt >= 5) {
        last_filt = now;
        for (int i = 0; i < LED_COUNT; i++)
            flk[i] = (uint8_t)((flk[i] * 7U + tgt[i]) / 8U);
    }
    for (int i = 0; i < LED_COUNT; i++)
        led_set(i, flk[i] * max_brightness / 100);
}

static void anim_chase(uint32_t tick) {
    (void)tick;
    uint32_t per = 400U * 100U / cfg.speed;
    uint32_t step = HAL_GetTick() / per;
    static const uint8_t pat[] = {1,1,1,0,0,0};
    for (int i = 0; i < LED_COUNT; i++)
        led_set(i, pat[((uint32_t)i + step) % 6] ? max_brightness : 0);
}

static void anim_gradient(uint32_t tick) {
    (void)tick;
    uint32_t per = 4000U * 100U / cfg.speed;
    uint32_t now = HAL_GetTick();
    uint8_t phase = (uint8_t)(now * 256U / per);
    int wave = isin_pos(phase);  /* 0..100 oscillation */
    /* LED 3 (center) always at max. Others blend between L→R and R→L gradients */
    for (int i = 0; i < LED_COUNT; i++) {
        int dist = i - 3; if (dist < 0) dist = -dist;  /* 0..3 from center */
        int base = max_brightness * (4 - dist) / 4;     /* center=max, edges=25% */
        int bias = (i < 3) ? wave : (100 - wave);       /* 0..100 */
        int bri = base + (max_brightness - base) * bias / 200;
        if (bri < max_brightness / 6) bri = max_brightness / 6;
        led_set(i, bri);
    }
}

static void anim_segments(uint32_t tick) {
    (void)tick;
    static const uint8_t seg[] = {0,0,1,1,1,2,2};
    uint32_t per = 4000U * 100U / cfg.speed;
    uint32_t now = HAL_GetTick();
    for (int i = 0; i < LED_COUNT; i++) {
        uint8_t phase = (uint8_t)(now * 256U / per + seg[i] * 85U);  /* 85 ≈ 256/3 */
        int bri = isin_pos(phase) * max_brightness / 100;
        if (bri < 2) bri = 2;
        led_set(i, bri);
    }
}

static void anim_glitter(uint32_t tick) {
    (void)tick;
    static uint8_t glt[7] = {0};
    static uint16_t lfsr = 0xDEADU;
    static uint32_t last = 0;
    uint32_t now = HAL_GetTick();
    /* Decay at 50Hz: factor ≈ 0.88 → ~150ms flash duration */
    if (now - last >= 20) {
        last = now;
        for (int i = 0; i < LED_COUNT; i++)
            glt[i] = (uint8_t)((uint16_t)glt[i] * 225U / 256U);
        /* Spark probability ~12% per step */
        lfsr = (uint16_t)((lfsr >> 1) ^ (-(lfsr & 1u) & 0xB400u));
        if ((lfsr & 0x7) == 0)
            glt[(lfsr >> 3) % LED_COUNT] = 100;
    }
    for (int i = 0; i < LED_COUNT; i++)
        led_set(i, glt[i] * max_brightness / 100);
}

static void anim_pingpong(uint32_t tick) {
    (void)tick;
    uint32_t per = 1800U * 100U / cfg.speed;
    uint32_t now = HAL_GetTick();
    uint32_t t = now % per;
    /* Sub-pixel position of block center, bouncing 1..5 */
    int p256 = (int)(t * 8U * 256U / per);
    if (p256 > 4 * 256) p256 = 8 * 256 - p256;
    p256 += 256;  /* offset to range 1..5 */
    for (int i = 0; i < LED_COUNT; i++) {
        int d = i * 256 - p256; if (d < 0) d = -d;
        int bri = 0;
        if (d < 128) bri = max_brightness;
        else if (d < 384) bri = max_brightness * (384 - d) / 256;
        led_set(i, bri);
    }
}

static void anim_interfere(uint32_t tick) {
    (void)tick;
    uint32_t now = HAL_GetTick();
    uint32_t p1 = 2000U * 100U / cfg.speed;
    uint32_t p2 = 2700U * 100U / cfg.speed;
    for (int i = 0; i < LED_COUNT; i++) {
        uint8_t ph1 = (uint8_t)(now * 256U / p1 + (uint32_t)i * 37U);
        uint8_t ph2 = (uint8_t)(now * 256U / p2 + (uint32_t)i * 55U);
        int w1 = isin(ph1);  /* -100..+100 */
        int w2 = isin(ph2);
        int bri = (w1 + w2 + 200) * max_brightness / 400;
        led_set(i, bri);
    }
}

static void anim_reflect(uint32_t tick) {
    (void)tick;
    uint32_t per = 2500U * 100U / cfg.speed;
    uint32_t now = HAL_GetTick();
    /* sin(ωt+ki) + sin(ωt-ki) = 2·sin(ωt)·cos(ki) */
    uint8_t t_phase = (uint8_t)(now * 256U / per);
    int time_val = isin(t_phase);  /* -100..+100 */
    for (int i = 0; i < LED_COUNT; i++) {
        /* k ≈ 42/256 per LED → ~6 LEDs per wavelength */
        uint8_t x_phase = (uint8_t)((uint32_t)i * 42U);
        int space_val = isin((uint8_t)(x_phase + 64U));  /* cos = sin(x+π/2) */
        int bri = (time_val * space_val / 100 + 100) * max_brightness / 200;
        led_set(i, bri);
    }
}

static void play_radiate(uint32_t tick) {
    (void)tick;
    static const uint8_t rad_pct[] = { 100, 60, 25, 8 };
    int center = ripple_center;
    uint8_t scale = decay_intensity ? decay_intensity : 255;
    uint32_t since = HAL_GetTick() - last_string_change;
    int flash = since < 200 ? (int)(100 + 55 * (200 - since) / 200) : 100;
    for (int i = 0; i < LED_COUNT; i++) {
        int dist = i - center; if (dist < 0) dist = -dist;
        int bri = dist < 4 ? max_brightness * rad_pct[dist] / 100 : 0;
        if (dist == 0) bri = bri * flash / 100;
        led_set(i, bri * scale / 255);
    }
}

static uint16_t vu_width16 = 0;
static void play_vu_mirror(uint32_t tick) {
    (void)tick;
    uint8_t scale = decay_intensity ? decay_intensity : 255;
    if (badge_state == ST_RELEASE_DECAY) {
        if (vu_width16 > 64) vu_width16 -= 64; else vu_width16 = 0;
    } else {
        if (vu_width16 < 3 * 256) vu_width16 += 16; else vu_width16 = 3 * 256;
    }
    for (int i = 0; i < LED_COUNT; i++) {
        int dist = i - 3; if (dist < 0) dist = -dist;
        int d256 = dist * 256;
        int bri = d256 <= (int)vu_width16 ? max_brightness * ((int)vu_width16 + 256 - d256) / ((int)vu_width16 + 256) : 0;
        led_set(i, bri * scale / 255);
    }
}

static int16_t momentum_pos = 0;
static int16_t momentum_vel = 0;
static void play_momentum(uint32_t tick) {
    (void)tick;
    if (momentum_dir != 0) {
        momentum_vel += (int16_t)(momentum_dir * 64);
        if (momentum_vel > 512) momentum_vel = 512;
        if (momentum_vel < -512) momentum_vel = -512;
        momentum_dir = 0;
    } else if (momentum_vel == 0 && badge_state == ST_PLAYING) {
        momentum_vel = (ripple_center > 3) ? -32 : 32;
    }
    momentum_pos += momentum_vel;
    int fric = badge_state == ST_RELEASE_DECAY ? 230 : 248;
    momentum_vel = (int16_t)(momentum_vel * fric / 256);
    if (momentum_pos < 0) { momentum_pos = 0; momentum_vel = (int16_t)(-(momentum_vel * 205 / 256)); }
    if (momentum_pos > 1536) { momentum_pos = 1536; momentum_vel = (int16_t)(-(momentum_vel * 205 / 256)); }
    int head = momentum_pos >> 8;
    uint8_t frac = (uint8_t)(momentum_pos & 0xFF);
    uint8_t scale = decay_intensity ? decay_intensity : 255;
    for (int i = 0; i < LED_COUNT; i++) {
        int b = 0;
        if (i == head) b = max_brightness * (255 - frac) / 255;
        else if (i == head + 1) b = max_brightness * frac / 255;
        else { int tr = momentum_vel > 0 ? head - 1 : head + 1;
               if (i == tr) b = max_brightness * 30 / 100; }
        led_set(i, b * scale / 255);
    }
}

static const anim_fn_t play_anims[] = { play_radiate, play_vu_mirror, play_momentum };

static void fx_update(void) {
    if (prev_key < 0) return;
    uint16_t base_arr = note_arr[prev_key];
    if (cfg.vibrato > 0) {
        int p = (int)(HAL_GetTick() / (40U - cfg.vibrato * 5U) % 200U);
        int tri = (p < 100 ? p : 200 - p) - 50;
        int mod = tri * (int)base_arr * cfg.vibrato / 5000;
        uint16_t new_arr = (uint16_t)((int)base_arr + mod);
        TIM2->ARR = new_arr;
        TIM2->CCR4 = (uint32_t)new_arr * (led_duty[0] * led_duty[0] / 100) / 100;
        TIM2->CCR1 = (uint32_t)new_arr * (led_duty[5] * led_duty[5] / 100) / 100;
        TIM2->CCR2 = (uint32_t)new_arr * (led_duty[6] * led_duty[6] / 100) / 100;
        base_arr = new_arr;
    }
    if (cfg.tremolo > 0) {
        int p = (int)(HAL_GetTick() / (60U - cfg.tremolo * 8U) % 200U);
        int tri = p < 100 ? p : 200 - p;
        uint8_t depth = cfg.tremolo * 10;
        uint32_t base_ccr = (uint32_t)base_arr * cfg.volume / 200;
        TIM2->CCR3 = base_ccr * (uint32_t)(100 - depth + tri * depth / 100) / 100;
    }
}

/* Bouncing-ball wave across all LEDs with exponential tail decay.
 * fade_factor: 0–255 per step (higher = longer tail; e.g. 160 ≈ nice fade)
 * loops: number of full left→right→left sweeps */
static void led_wave(uint8_t tail_len, uint8_t fade_factor,
                     uint32_t delay_ms, uint32_t loops)
{
    int pos = 0, dir = 1;
    uint32_t total_steps = (uint32_t)(LED_COUNT - 1) * 2U * loops;

    for (uint32_t step = 0; step < total_steps; step++) {
        for (int i = 0; i < LED_COUNT; i++) {
            tud_task();
            int dist = i - pos;
            if (dist < 0) dist = -dist;
            uint8_t brightness = 0;
            if (dist == 0) {
                brightness = MAX_BRIGHT;
            } else if (dist < tail_len) {
                uint16_t val = MAX_BRIGHT;
                for (int k = 0; k < dist; k++) {
                    val = (uint16_t)((val * fade_factor) >> 8);
                    tud_task();
                }
                brightness = (uint8_t)val;
            }
            led_set(i, brightness);
        }
        HAL_Delay(delay_ms);
        tud_task();
        pos += dir;
        if (pos >= LED_COUNT - 1) { pos = LED_COUNT - 1; dir = -1; }
        else if (pos <= 0)        { pos = 0;              dir =  1; }
    }
    HAL_Delay(delay_ms);
    for (int i = 0; i < LED_COUNT; i++) led_set(i, 0);
}

/* ---- String / CDC output helpers ------------------------------------------*/
static int u32_to_str(uint32_t val, char *buf)
{
    char tmp[10];
    int i = 0;
    if (val == 0) { buf[0] = '0'; return 1; }
    while (val) { tmp[i++] = '0' + (val % 10); val /= 10; }
    for (int j = 0; j < i; j++) buf[j] = tmp[i - 1 - j];
    return i;
}
static void cdc_print(const char *s)
{
    uint32_t len = (uint32_t)strlen(s);
    if (tud_cdc_write_available() < len) tud_cdc_write_flush();
    tud_cdc_write(s, len);
}
static void cdc_println(const char *s)
{
    cdc_print(s);
    cdc_print("\r\n");
}
static void cdc_print_u32(uint32_t val)
{
    char buf[10];
    int len = u32_to_str(val, buf);
    tud_cdc_write(buf, (uint32_t)len);
}

/* ---- CTF melody detection + morse ---------------------------------------- */
static void melody_record(uint8_t key) {
    melody_buf[melody_idx] = key;
    melody_idx = (melody_idx + 1U) % 6U;
    for (uint8_t i = 0; i < 6; i++) {
        if (melody_buf[(melody_idx + i) % 6U] != melody_target[i]) return;
    }
    if (!morse_active) {
        morse_active = true;
        cfg.ctf_solved |= 0x04;  /* challenge 3 solved */
        cfg_save();
        cdc_println("listen with your eyes");
    }
}

static uint8_t touch_scan(void);  /* forward decl */

/* Blocking morse loop — never returns. Player must power-cycle.
 * Buzzer mirrors LED when any string is touched; checked once per element. */
__attribute__((noreturn))
static void morse_play_loop(void) {
    uint8_t mbright = (uint8_t)(max_brightness < 10 ? 10 : max_brightness);

    /* All LEDs off */
    for (int i = 0; i < 7; i++) led_set(i, 0);

    /* Victory tone: ascending D, F#, A */
    static const uint8_t victory_notes[] = {0, 2, 4};
    for (int i = 0; i < 3; i++) {
        buzzer_set_note(victory_notes[i]);
        led_set(3, mbright);
        HAL_Delay(150);
    }
    buzzer_set_note(-1);
    led_set(3, 0);
    HAL_Delay(300);

    /* Infinite morse loop */
    for (;;) {
        for (uint8_t ci = 0; ci < MORSE_FLAG_LEN; ci++) {
            char c = (char)(morse_flag_enc[ci] ^ MORSE_FLAG_KEY);

            int idx = -1;
            if (c >= 'a' && c <= 'z') idx = (int)(c - 'a');
            else if (c >= '0' && c <= '9') idx = 26 + (int)(c - '0');

            if (idx < 0) {
                HAL_Delay(700);
                continue;
            }

            uint8_t enc = morse_table[idx];
            uint8_t len = (enc >> 5) & 0x07;

            for (uint8_t ei = 0; ei < len; ei++) {
                uint16_t dur = (enc >> (4 - ei)) & 1 ? 600 : 200;
                /* LED on + buzzer if touching */
                led_set(3, mbright);
                touch_scan();
                if (touch_mask) buzzer_set_note(2);
                HAL_Delay(dur);
                /* LED off + buzzer off */
                led_set(3, 0);
                buzzer_set_note(-1);
                HAL_Delay(200);  /* inter-element gap */
            }
            HAL_Delay(400);  /* letter gap (600 total - 200 already) */
        }
        HAL_Delay(1400);  /* word gap before restart */
    }
}

/* ---- Mode / playmode name tables ----------------------------------------- */
static const char * const mode_names[] = {
"breathe", "wave", "sparkle", "scanner", "pulse",
"travel", "ripple", "chase", "segments", "glitter",
"pingpong", "interfere", "reflect", "flicker", "gradient",
"off"
};
#define MODE_COUNT 16
#define ANIM_COUNT (MODE_COUNT - 1)  /* excludes "off" */

static const char * const playmode_names[] = {
"radiate", "vu", "momentum"
};
#define PLAYMODE_COUNT 3

/* ---- CLI command handlers --------------------------------------------------*/
static void cmd_help([[maybe_unused]] const char *arg)
{
    cdc_println("");
    cdc_println(" Animations");
    cdc_println("  idle [name]      15 animations (idle to list)");
    cdc_println("  play [name]      radiate vu momentum");
    cdc_println("  brightness [N]   LED max (0-100)");
    cdc_println("  speed [N]        x0.10-x20.0 or presets");
    cdc_println("  cycle [all|off]  auto-cycle animations");
    cdc_println(" Music");
    cdc_println("  scale [lo|hi]    octave D5-D6 / D6-D7");
    cdc_println("  volume [N]       buzzer volume (0-100)");
    cdc_println("  buzzer [on|off]  buzzer enable");
    cdc_println("  note [0-5]       play a note");
    cdc_println(" Badge");
    cdc_println("  name [text]");
    cdc_println("  status           diagnostics");
    cdc_println("  version          firmware info");
    cdc_println("  id               unique device ID");
    cdc_println(" System");
    cdc_println("  pin <code>");
    cdc_println("  advanced         power, touch, sound & more");
    cdc_println("  reset            restore all defaults");
    if (cfg.ctf_solved & 0x08 && cfg.evil_enabled)
        cdc_println("  evil             with great power comes...");

    cdc_println("");
    cdc_println(" 4 flags are hidden in this badge");
}

static void cmd_dfu(const char *arg)
{
    (void)arg;
    dfu_boot_flag = DFU_BOOT_FLAG;
    NVIC_SystemReset();
}

/* Uptime counter in seconds, incremented in main loop */
static uint32_t uptime_sec = 0;
/* Last detected key, updated by main loop */
static int cli_detected_key = -1;

static void cmd_status(const char *arg)
{
    (void)arg;
    static const char * const state_names[] = { "idle", "playing", "decay" };
    cdc_print("  state: "); cdc_println(state_names[badge_state]);
    cdc_print("  idle:  "); cdc_println(mode_names[idle_mode % MODE_COUNT]);
    cdc_print("  play:  "); cdc_println(playmode_names[cfg.play_mode % PLAYMODE_COUNT]);
    cdc_print("  uptime: "); cdc_print_u32(uptime_sec); cdc_println("s");
    cdc_print("  touch:");
    for (int i = 0; i < 6; i++) {
        int16_t d = touch_ch[i].delta;
        cdc_print(" ");
        if (d < 0) { cdc_print("-"); d = -d; }
        cdc_print_u32((uint32_t)d);
        if (touch_ch[i].touched) cdc_print("*");
    }
    cdc_println("");
    cdc_print("  note: ");
    if (cli_detected_key >= 0) cdc_print_u32((uint32_t)cli_detected_key);
    else cdc_print("none");
    cdc_println("");
    cdc_print("  power: "); cdc_println(usb_connected ? "USB" : "battery");
    cdc_print("  brightness: "); cdc_print_u32(max_brightness); cdc_println("");
    cdc_println("  Song: D F# B D B F#");
}

static int match_name(const char *arg, const char * const *names, int count)
{
    if (!arg || !*arg) return -1;
    if (*arg >= '0' && *arg <= '9') {
        int v = *arg - '0';
        return (v < count) ? v : -2;
    }
    for (int i = 0; i < count; i++)
        if (strncmp(arg, names[i], strlen(arg)) == 0) return i;
    return -2;  /* invalid */
}

static void cmd_speed(const char *arg);  /* forward decl — implemented below with picker */
static void cli_show_picker(int cmd_idx); /* forward decl — defined after cmd_args[] */

static void cmd_mode(const char *arg)
{
    if (!arg || !*arg) {
        cli_show_picker(1);  /* idle is command index 1 */
        return;
    }
    int m = match_name(arg, mode_names, MODE_COUNT);
    if (m >= 0) {
        idle_mode = (uint8_t)m;
        cfg.idle_mode = idle_mode;
        cfg.cycle_mask = 0;  /* stop cycling — user picked specific anim */
        cfg_save();
        mode_start = HAL_GetTick();
    }
    cdc_print("  idle: ");
    cdc_println(mode_names[idle_mode % MODE_COUNT]);
    if (m == -2) {
        cli_show_picker(1);
    }
}

static void cmd_note(const char *arg)
{
    if (arg && *arg >= '0' && *arg <= '5') {
        int n = *arg - '0';
        buzzer_set_note(n);
        uint32_t tick = HAL_GetTick();
        while ((HAL_GetTick() - tick) < 500U) {
            HAL_Delay(1);
            tud_task();
        }
        buzzer_set_note(-1);
        cdc_print("  note: "); cdc_print_u32((uint32_t)n); cdc_println("");
    } else {
        cdc_println("  note [0-5]");
    }
}

static void cmd_scale(const char *arg)
{
    if (!arg || !*arg) {
        cli_show_picker(5);  /* scale is command index 5 */
        return;
    }
    if (arg[0] == 'h' || (arg[0] >= '0' && arg[0] <= '9' && arg[0] == '2')) {
        note_arr = note_arr_high; cfg.scale = 1; cfg_save();
    } else if (arg[0] == 'l' || (arg[0] >= '0' && arg[0] <= '9' && arg[0] == '1')) {
        note_arr = note_arr_low; cfg.scale = 0; cfg_save();
    }
    cdc_print("  scale: ");
    cdc_println(cfg.scale ? "high (D6-D7)" : "low (D5-D6)");
}

static void cmd_brightness(const char *arg)
{
    if (arg && *arg >= '0' && *arg <= '9') {
        int val = 0;
        const char *p = arg;
        while (*p >= '0' && *p <= '9') val = val * 10 + (*p++ - '0');
        if (val > 100) val = 100;
        max_brightness = (uint8_t)val;
        cfg.brightness = max_brightness;
        cfg_save();
    }
    cdc_print("  brightness: ");
    cdc_print_u32(max_brightness);
    cdc_println("");
}

static void cmd_version(const char *arg)
{
    (void)arg;
    cdc_println("  BSides Dublin Badge v0.9");
}

static void cmd_id(const char *arg)
{
    (void)arg;
    /* STM32 96-bit UID at UID_BASE: three 32-bit words */
    static const char hex[] = "0123456789ABCDEF";
    uint32_t words[3];
    words[0] = HAL_GetUIDw0();
    words[1] = HAL_GetUIDw1();
    words[2] = HAL_GetUIDw2();
    for (int w = 0; w < 3; w++) {
        uint32_t v = words[w];
        char nibbles[8];
        for (int i = 7; i >= 0; i--) {
            nibbles[i] = hex[v & 0xFu];
            v >>= 4;
        }
        tud_cdc_write(nibbles, 8);
        if (w < 2) cdc_print("-");
    }
    cdc_println("");
}

static void cmd_playmode(const char *arg)
{
    if (!arg || !*arg) {
        cli_show_picker(2);  /* play is command index 2 */
        return;
    }
    int m = match_name(arg, playmode_names, PLAYMODE_COUNT);
    if (m >= 0) {
        cfg.play_mode = (uint8_t)m;
        cfg_save();
    }
    cdc_print("  play: ");
    cdc_println(playmode_names[cfg.play_mode % PLAYMODE_COUNT]);
    if (m == -2) {
        cli_show_picker(2);
    }
}

static void cmd_volume(const char *arg)
{
    if (arg && *arg >= '0' && *arg <= '9') {
        int val = 0;
        const char *p = arg;
        while (*p >= '0' && *p <= '9') val = val * 10 + (*p++ - '0');
        if (val > 100) val = 100;
        cfg.volume = (uint8_t)val;
        cfg_save();
    }
    cdc_print("  volume: ");
    cdc_print_u32(cfg.volume);
    cdc_println("");
}

static void cmd_buzzer(const char *arg)
{
    if (!arg || !*arg) {
        cli_show_picker(7);  /* buzzer is command index 7 */
        return;
    }
    if (arg[0] == 'o') {
        cfg.buzzer_on = (uint8_t)(arg[1] == 'n' ? 1 : 0);
        cfg_save();
    } else if (arg[0] == '1') {
        cfg.buzzer_on = 1; cfg_save();
    } else if (arg[0] == '2') {
        cfg.buzzer_on = 0; cfg_save();
    }
    cdc_print("  buzzer: ");
    cdc_println(cfg.buzzer_on ? "on" : "off");
}

static void cmd_name(const char *arg)
{
    if (arg && *arg) {
        int i = 0;
        while (arg[i] && i < 20) {
            if (arg[i] >= 0x20 && arg[i] <= 0x7E)
                cfg.name[i] = arg[i];
            else
                cfg.name[i] = '?';
            i++;
        }
        cfg.name[i] = '\0';
        cfg_save();
    }
    cdc_print("  name: ");
    cdc_println(cfg.name[0] ? cfg.name : "(none)");
}

static void cfg_print_onoff(const char *label, uint8_t val)
{
    cdc_print(label);
    cdc_println(val ? "on" : "off");
}

static void cmd_reset(const char *arg)
{
    (void)arg;
    cfg_defaults();
    cfg_save();
    cfg_apply();
    cdc_println("  all settings restored to defaults");
}

static void cli_show_cycle_picker(void);  /* forward decl */

static void cmd_cycle(const char *arg)
{
    while (arg && *arg == ' ') arg++;
    if (arg && arg[0] == 'a') {
        cfg.cycle_mask = (uint16_t)((1U << ANIM_COUNT) - 1);
        cfg_save();
        cdc_println("  cycling: all");
        return;
    }
    if (arg && arg[0] == 'o' && arg[1] == 'f') {
        cfg.cycle_mask = 0;
        cfg_save();
        cdc_println("  cycling: off");
        return;
    }
    if (arg && ((*arg >= '0' && *arg <= '9') || *arg == ',')) {
        uint16_t mask = 0;
        const char *p = arg;
        while (*p) {
            while (*p == ',' || *p == ' ') p++;
            if (*p >= '0' && *p <= '9') {
                int v = 0;
                while (*p >= '0' && *p <= '9') v = v * 10 + (*p++ - '0');
                v--;
                if (v >= 0 && v < ANIM_COUNT) mask |= (uint16_t)(1U << v);
            } else break;
        }
        cfg.cycle_mask = mask;
        cfg_save();
        cdc_print("  cycling:");
        for (int i = 0; i < ANIM_COUNT; i++)
            if (mask & (1U << i)) { cdc_print(" "); cdc_print(mode_names[i]); }
        cdc_println("");
        return;
    }
    /* No args: show interactive picker */
    cli_show_cycle_picker();
}

static void cmd_tremolo(const char *arg)
{
    if (arg && *arg >= '0' && *arg <= '9') {
        int val = *arg - '0';
        if (val > 5) val = 5;
        cfg.tremolo = (uint8_t)val; cfg_save();
    }
    cdc_print("  tremolo: "); cdc_print_u32(cfg.tremolo); cdc_println("");
}

static void cmd_vibrato(const char *arg)
{
    if (arg && *arg >= '0' && *arg <= '9') {
        int val = *arg - '0';
        if (val > 5) val = 5;
        cfg.vibrato = (uint8_t)val; cfg_save();
    }
    cdc_print("  vibrato: "); cdc_print_u32(cfg.vibrato); cdc_println("");
}

static void cmd_speed(const char *arg)
{
    if (arg && ((*arg >= '0' && *arg <= '9') || *arg == 'x' || *arg == '.')) {
        const char *p = arg;
        if (*p == 'x') p++;
        int whole = 0, frac = 0, fdiv = 1;
        while (*p >= '0' && *p <= '9') whole = whole * 10 + (*p++ - '0');
        if (*p == '.') {
            p++;
            while (*p >= '0' && *p <= '9' && fdiv < 100) {
                frac = frac * 10 + (*p++ - '0');
                fdiv *= 10;
            }
        }
        int val = whole * 100 + frac * (100 / fdiv);
        if (val < 10) val = 10;
        if (val > 2000) val = 2000;
        cfg.speed = (uint16_t)val;
        cfg_save();
    } else if (!arg || !*arg) {
        cli_show_picker(4);  /* speed is command index 4 */
        return;
    }
    cdc_print("  speed: x");
    cdc_print_u32(cfg.speed / 100);
    cdc_print(".");
    cdc_print_u32((cfg.speed / 10) % 10);
    cdc_print_u32(cfg.speed % 10);
    cdc_println("");
}

static void cmd_advanced(const char *arg)
{
    while (arg && *arg == ' ') arg++;
    if (!arg || !*arg || (arg[0]=='h' && arg[1]=='e')) {
        /* show advanced help */
        cdc_println("");
        cdc_println(" Power");
        cdc_println("  timeout [N.N]    sleep seconds");
        cdc_println("  shutdown [N]     standby minutes");
        cdc_println("  powersave [on|off]");
        cdc_println(" Touch");
        cdc_println("  debounce [N]     change delay ms");
        cdc_println("  maxhold [N]      max touch ms");
        cdc_println("  tdetect [N]      detect % (1-50)");
        cdc_println("  trelease [N]     release % (1-50)");
        cdc_println("  tmin [N]         min threshold (10-2550)");
        cdc_println("  tdeb [N]         channel debounce (1-10)");
        cdc_println("  rdeb [N]         note resolve deb (1-10)");
        cdc_println(" Sound");
        cdc_println("  tremolo [N]      depth (0-5)");
        cdc_println("  vibrato [N]      depth (0-5)");
        cdc_println("  notes [f1,...,f6] custom Hz per string");
        cdc_println(" Animations");
        cdc_println("  sleepanim [on|off]");
        cdc_println("  wakescan [0|1|2] off/blink/beat");
        cdc_println("  wakeanim [on|off]");
        cdc_println(" System");
        cdc_println("  dfu              reboot to bootloader");
        if (cfg.ctf_solved & 0x08)
            cdc_println("  evil [on|off]    for educational purposes, ofcourse...");
        cdc_println("");
        cdc_println(" advanced <param> <value>");
        return;
    }
    if (arg[0]=='r' && arg[1]=='e') {
        /* reset advanced settings only */
        cfg.debounce_ms = 400; cfg.maxhold_ms = 7000;
        cfg.timeout_ds = 50; cfg.shutdown_min = 90;
        cfg.powersave = 1; cfg.sleep_anim = 1;
        cfg.wake_scan_anim = 1; cfg.wake_touch_anim = 1;
        cfg.touch_detect = 10; cfg.touch_release = 6; cfg.touch_min = 20;
        cfg.touch_deb = 4; cfg.resolve_deb = 3;
        cfg_save();
        cdc_println("  advanced settings restored");
        return;
    }
    /* find subcommand value */
    const char *sub = arg;
    const char *val = arg;
    while (*val && *val != ' ') val++;
    while (*val == ' ') val++;

    int ival = 0;
    { const char *p = val;
        while (*p >= '0' && *p <= '9') ival = ival * 10 + (*p++ - '0'); }

    if (sub[0]=='t' && sub[1]=='i') {
        if (*val) {
            int whole = 0, tenths = 0;
            const char *p = val;
            while (*p >= '0' && *p <= '9') whole = whole * 10 + (*p++ - '0');
            if (*p == '.') { p++; if (*p >= '0' && *p <= '9') tenths = *p - '0'; }
            int ds = whole * 10 + tenths;
            if (ds < 10) ds = 10; if (ds > 6000) ds = 6000;
            cfg.timeout_ds = (uint16_t)ds; cfg_save();
        }
        cdc_print("  timeout: "); cdc_print_u32(cfg.timeout_ds / 10);
        cdc_print("."); cdc_print_u32(cfg.timeout_ds % 10); cdc_println("s");
    } else if (sub[0]=='s' && sub[1]=='h') {
        if (*val) {
            if (ival > 1440) ival = 1440;
            cfg.shutdown_min = (uint16_t)ival; cfg_save();
        }
        cdc_print("  shutdown: "); cdc_print_u32(cfg.shutdown_min); cdc_println("min");
    } else if (sub[0]=='p' && sub[1]=='o') {
        if (*val && val[0]=='o') { cfg.powersave = (uint8_t)(val[1]=='n' ? 1 : 0); cfg_save(); }
        cfg_print_onoff("  powersave: ", cfg.powersave);
    } else if (sub[0]=='d' && sub[1]=='e') {
        if (*val) {
            if (ival < 50) ival = 50; if (ival > 2000) ival = 2000;
            cfg.debounce_ms = (uint16_t)ival; cfg_save();
        }
        cdc_print("  debounce: "); cdc_print_u32(cfg.debounce_ms); cdc_println("ms");
    } else if (sub[0]=='m' && sub[1]=='a') {
        if (*val) {
            if (ival < 1000) ival = 1000; if (ival > 30000) ival = 30000;
            cfg.maxhold_ms = (uint16_t)ival; cfg_save();
        }
        cdc_print("  maxhold: "); cdc_print_u32(cfg.maxhold_ms); cdc_println("ms");
    } else if (sub[0]=='s' && sub[1]=='l') {
        if (*val && val[0]=='o') { cfg.sleep_anim = (uint8_t)(val[1]=='n' ? 1 : 0); cfg_save(); }
        cfg_print_onoff("  sleepanim: ", cfg.sleep_anim);
    } else if (sub[0]=='w' && sub[1]=='a' && sub[2]=='k' && sub[3]=='e' && sub[4]=='s') {
        if (*val && *val >= '0' && *val <= '2') { cfg.wake_scan_anim = (uint8_t)(*val - '0'); cfg_save(); }
        cdc_print("  wakescan: "); cdc_print_u32(cfg.wake_scan_anim);
        cdc_println(cfg.wake_scan_anim==0?" (off)":cfg.wake_scan_anim==1?" (blink)":" (beat)");
    } else if (sub[0]=='w' && sub[1]=='a' && sub[2]=='k' && sub[3]=='e' && sub[4]=='a') {
        if (*val && val[0]=='o') { cfg.wake_touch_anim = (uint8_t)(val[1]=='n' ? 1 : 0); cfg_save(); }
        cfg_print_onoff("  wakeanim: ", cfg.wake_touch_anim);
    } else if (sub[0]=='t' && sub[1]=='d' && sub[2]=='e' && sub[3]=='t') {
        if (*val) {
            if (ival < 1) ival = 1; if (ival > 50) ival = 50;
            cfg.touch_detect = (uint8_t)ival; cfg_save();
        }
        cdc_print("  tdetect: "); cdc_print_u32(cfg.touch_detect); cdc_println("%");
    } else if (sub[0]=='t' && sub[1]=='d' && sub[2]=='e' && sub[3]=='b') {
        if (*val) {
            if (ival < 1) ival = 1; if (ival > 10) ival = 10;
            cfg.touch_deb = (uint8_t)ival; cfg_save();
        }
        cdc_print("  tdeb: "); cdc_print_u32(cfg.touch_deb); cdc_println("");
    } else if (sub[0]=='t' && sub[1]=='r' && sub[2]=='e' && sub[3]=='l') {
        if (*val) {
            if (ival < 1) ival = 1; if (ival > 50) ival = 50;
            cfg.touch_release = (uint8_t)ival; cfg_save();
        }
        cdc_print("  trelease: "); cdc_print_u32(cfg.touch_release); cdc_println("%");
    } else if (sub[0]=='t' && sub[1]=='m') {
        if (*val) {
            if (ival < 10) ival = 10; if (ival > 2550) ival = 2550;
            cfg.touch_min = (uint8_t)(ival / 10); cfg_save();
        }
        cdc_print("  tmin: "); cdc_print_u32((uint32_t)cfg.touch_min * 10U); cdc_println("");
    } else if (sub[0]=='r' && sub[1]=='d') {
        if (*val) {
            if (ival < 1) ival = 1; if (ival > 10) ival = 10;
            cfg.resolve_deb = (uint8_t)ival; cfg_save();
        }
        cdc_print("  rdeb: "); cdc_print_u32(cfg.resolve_deb); cdc_println("");
    } else if (sub[0]=='t' && sub[1]=='r') {
        cmd_tremolo(val);
    } else if (sub[0]=='v' && sub[1]=='i') {
        cmd_vibrato(val);
    } else if (sub[0]=='d' && sub[1]=='f') {
        cmd_dfu(val);
    } else if (sub[0]=='n' && sub[1]=='o') {
        /* notes f1,f2,...,f6 — parse comma/space-separated Hz values */
        if (*val) {
            const char *p = val;
            for (int i = 0; i < 6 && *p; i++) {
                while (*p == ',' || *p == ' ') p++;
                if (*p >= '0' && *p <= '9') {
                    /* Parse float freq: whole + optional .frac */
                    uint32_t whole = 0;
                    while (*p >= '0' && *p <= '9') whole = whole * 10 + (uint32_t)(*p++ - '0');
                    uint32_t frac = 0, fdiv = 1;
                    if (*p == '.') { p++;
                        while (*p >= '0' && *p <= '9' && fdiv < 100) {
                            frac = frac * 10 + (uint32_t)(*p++ - '0'); fdiv *= 10;
                        }
                    }
                    /* freq_x100 = whole*100 + frac*(100/fdiv) */
                    uint32_t freq_x100 = whole * 100 + frac * (100 / fdiv);
                    /* ARR = 16000000/freq - 1 = 1600000000/(freq_x100) - 1 */
                    if (freq_x100 >= 100) {
                        uint32_t arr = 1600000000UL / freq_x100 - 1;
                        if (arr > 65535) arr = 65535;
                        if (arr < 100) arr = 100;
                        cfg.custom_notes[i] = (uint16_t)arr;
                    }
                } else break;
            }
            cfg_save(); cfg_apply();
        }
        /* Display current frequencies */
        cdc_print("  notes:");
        for (int i = 0; i < 6; i++) {
            uint16_t arr = note_arr[i];
            uint32_t freq_x10 = 160000000UL / ((uint32_t)arr + 1);
            cdc_print(" "); cdc_print_u32(freq_x10 / 10);
            cdc_print("."); cdc_print_u32(freq_x10 % 10);
        }
        cdc_println(" Hz");
    } else if (sub[0]=='e' && sub[1]=='v') {
        if (!(cfg.ctf_solved & 0x08)) {
            cdc_println("  [locked]");
            return;
        }
        if (*val && val[0]=='o') {
            cfg.evil_enabled = (uint8_t)(val[1]=='n' ? 1 : 0);
            cfg_save();
        }
        cfg_print_onoff("  evil: ", cfg.evil_enabled);
    } else {
        cdc_print("  unknown: "); cdc_println(sub);
    }
}

/* ---- Evil mode (rubber ducky HID keyboard) --------------------------------*/
#include "class/hid/hid.h"

/* EEPROM payload area: starts right after config struct (word-aligned) */
#define EVIL_PAYLOAD_BASE  (DATA_EEPROM_BASE + ((sizeof(badge_config_t) + 3U) & ~3U))
#define EVIL_PAYLOAD_MAX   (DATA_EEPROM_BASE + 2048U - EVIL_PAYLOAD_BASE)

/* Bytecode opcodes */
#define EV_END      0x00
#define EV_DELAY    0x01  /* +2 bytes: ms big-endian */
#define EV_KEY      0x02  /* +2 bytes: modifier, keycode */
#define EV_STRING   0x03  /* +1 byte len, then len ASCII chars */
#define EV_KEYDOWN  0x04  /* +2 bytes: modifier, keycode */
#define EV_KEYUP    0x05  /* release all */

/* Compact ASCII-to-HID table: bit7=shift, bits6-0=keycode (US layout) */
static const uint8_t ascii_to_hid[128] = {
0,0,0,0,0,0,0,0,                                           /* 0x00-0x07 */
0x2A,0x2B,0x28,0,0,0x28,0,0,                               /* 0x08-0x0F: BS,TAB,LF,,,CR */
0,0,0,0,0,0,0,0, 0,0,0,0x29,0,0,0,0,                      /* 0x10-0x1F: ..ESC.. */
0x2C,                                                        /* 0x20 space */
0x9E,0xB4,0xA0,0xA1,0xA2,0xA4,0x34,                        /* !  "  #  $  %  &  ' */
0xA6,0xA7,0xA5,0xB0,0x36,0x2D,0x37,0x38,                   /* (  )  *  +  ,  -  .  / */
0x27,0x1E,0x1F,0x20,0x21,0x22,0x23,0x24,0x25,0x26,         /* 0-9 */
0xB3,0x33,0xB6,0x2E,0xB7,0xB8,                             /* :  ;  <  =  >  ? */
0x9F,                                                        /* @ */
0x84,0x85,0x86,0x87,0x88,0x89,0x8A,0x8B,0x8C,0x8D,         /* A-J */
0x8E,0x8F,0x90,0x91,0x92,0x93,0x94,0x95,0x96,0x97,         /* K-T */
0x98,0x99,0x9A,0x9B,0x9C,0x9D,                              /* U-Z */
0x2F,0x31,0x30,0xA3,0xAD,0x35,                              /* [  \  ]  ^  _  ` */
0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,         /* a-j */
0x0E,0x0F,0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,         /* k-t */
0x18,0x19,0x1A,0x1B,0x1C,0x1D,                              /* u-z */
0xAF,0xB1,0xB0,0xA0,0,                                      /* {  |  }  ~  DEL */
};

static void evil_send_key(uint8_t modifier, uint8_t keycode)
{
    uint8_t kc[6] = {keycode, 0, 0, 0, 0, 0};
    while (!tud_hid_ready()) tud_task();
    tud_hid_keyboard_report(0, modifier, kc);
    HAL_Delay(10);
    while (!tud_hid_ready()) tud_task();
    tud_hid_keyboard_report(0, 0, NULL);  /* release */
    HAL_Delay(10);
}

static void evil_send_char(uint8_t ch)
{
    if (ch >= 128) return;
    uint8_t entry = ascii_to_hid[ch];
    if (!entry) return;
    uint8_t mod = (entry & 0x80) ? KEYBOARD_MODIFIER_LEFTSHIFT : 0;
    evil_send_key(mod, entry & 0x7F);
}

/* Execute bytecode payload from EEPROM — never returns */
static void evil_run(void)
{
    /* Visual indicator: center LEDs */
    for (int i = 0; i < 7; i++) led_set(i, 0);
    led_set(3, 100); led_set(4, 100);

    /* Wait for host to mount + settle */
    while (!tud_mounted()) tud_task();
    /* Settle delay — keep servicing USB so host doesn't timeout */
    { uint32_t t0 = HAL_GetTick();
        while ((HAL_GetTick() - t0) < 500U) tud_task(); }

    const uint8_t *p   = (const uint8_t *)EVIL_PAYLOAD_BASE;
    const uint8_t *end = (const uint8_t *)(DATA_EEPROM_BASE + 2048U);

    while (p < end) {
        tud_task();
        uint8_t op = *p++;
        if (op == EV_END || op == 0xFF) break;
        switch (op) {
            case EV_DELAY: {
                uint16_t ms = (uint16_t)(*p++ << 8);
                ms |= *p++;
                uint32_t t0 = HAL_GetTick();
                while ((HAL_GetTick() - t0) < ms) tud_task();
                break;
            }
            case EV_KEY:
                evil_send_key(p[0], p[1]);
                p += 2;
                break;
            case EV_STRING: {
                uint8_t len = *p++;
                for (uint8_t i = 0; i < len && p < end; i++)
                    evil_send_char(*p++);
                break;
            }
            case EV_KEYDOWN: {
                uint8_t kc[6] = {p[1], 0, 0, 0, 0, 0};
                while (!tud_hid_ready()) tud_task();
                tud_hid_keyboard_report(0, p[0], kc);
                HAL_Delay(10);
                p += 2;
                break;
            }
            case EV_KEYUP:
                while (!tud_hid_ready()) tud_task();
                tud_hid_keyboard_report(0, 0, NULL);
                HAL_Delay(10);
                break;
            default:
                goto done;  /* unknown opcode — stop */
        }
    }
done:
    /* Payload complete — LEDs off, idle forever */
    for (int i = 0; i < 7; i++) led_set(i, 0);
    while (1) tud_task();
}

/* HID callbacks required by TinyUSB */
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id,
                               hid_report_type_t report_type,
                               uint8_t *buffer, uint16_t reqlen)
{
    (void)instance; (void)report_id; (void)report_type;
    (void)buffer; (void)reqlen;
    return 0;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
                           hid_report_type_t report_type,
                           uint8_t const *buffer, uint16_t bufsize)
{
    (void)instance; (void)report_id; (void)report_type;
    (void)buffer; (void)bufsize;
}

/* ---- Evil mode: EEPROM write helpers ------------------------------------- */
static uint32_t evil_write_addr;  /* current write pointer during evil set */

static void evil_eeprom_write_byte(uint8_t byte)
{
    if (evil_write_addr >= DATA_EEPROM_BASE + 2048U) return;
    HAL_FLASHEx_DATAEEPROM_Unlock();
    HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_BYTE,
                                   evil_write_addr++, byte);
    HAL_FLASHEx_DATAEEPROM_Lock();
}

/* ---- Evil mode: DuckyScript → bytecode compiler -------------------------- */

/* Single-key command lookup */
static const struct { const char *name; uint8_t keycode; } duck_keys[] = {
{ "ENTER",      HID_KEY_ENTER },
{ "TAB",        HID_KEY_TAB },
{ "ESCAPE",     HID_KEY_ESCAPE },
{ "BACKSPACE",  HID_KEY_BACKSPACE },
{ "DELETE",     HID_KEY_DELETE },
{ "SPACE",      HID_KEY_SPACE },
{ "UPARROW",    HID_KEY_ARROW_UP },
{ "DOWNARROW",  HID_KEY_ARROW_DOWN },
{ "LEFTARROW",  HID_KEY_ARROW_LEFT },
{ "RIGHTARROW", HID_KEY_ARROW_RIGHT },
{ "CAPSLOCK",   HID_KEY_CAPS_LOCK },
{ "PRINTSCREEN",HID_KEY_PRINT_SCREEN },
{ "PAUSE",      HID_KEY_PAUSE },
{ "INSERT",     HID_KEY_INSERT },
{ "HOME",       HID_KEY_HOME },
{ "END",        HID_KEY_END },
{ "PAGEUP",     HID_KEY_PAGE_UP },
{ "PAGEDOWN",   HID_KEY_PAGE_DOWN },
};
#define DUCK_KEY_COUNT ((int)(sizeof(duck_keys) / sizeof(duck_keys[0])))

/* Convert single char to HID keycode (for modifier+key combos) */
static uint8_t char_to_keycode(char c)
{
    if (c >= 'a' && c <= 'z') return (uint8_t)(HID_KEY_A + (c - 'a'));
    if (c >= 'A' && c <= 'Z') return (uint8_t)(HID_KEY_A + (c - 'A'));
    if (c >= '1' && c <= '9') return (uint8_t)(HID_KEY_1 + (c - '1'));
    if (c == '0') return HID_KEY_0;
    return 0;
}

/* Saved bytecode of last compiled line (for REPEAT) */
static uint8_t last_line_bc[68];
static uint8_t last_line_bc_len = 0;

/* Compile one DuckyScript line to bytecode, write to EEPROM.
 * Returns 1 on success, 0 on error (full / unknown command). */
static int evil_compile_line(const char *line)
{
    while (*line == ' ') line++;
    if (!*line) return 1;  /* empty line */

    /* Save write position to capture this line's bytecode for REPEAT */
    uint32_t line_start_addr = evil_write_addr;
    last_line_bc_len = 0;

    /* Case-insensitive prefix matching helpers */
#define MATCH(s) (strncmp(line, s, sizeof(s)-1) == 0)
#define ARGAFTER(s) (line + sizeof(s) - 1 + (line[sizeof(s)-1] == ' ' ? 1 : 0))

    if (MATCH("REM")) {
        return 1;  /* comment — skip */
    }
    if (MATCH("STRING ")) {
        const char *text = line + 7;
        uint8_t len = (uint8_t)strlen(text);
        if (len > 255) len = 255;
        evil_eeprom_write_byte(EV_STRING);
        evil_eeprom_write_byte(len);
        for (uint8_t i = 0; i < len; i++)
            evil_eeprom_write_byte((uint8_t)text[i]);
    } else if (MATCH("DELAY ")) {
        int ms = 0;
        const char *p = line + 6;
        while (*p >= '0' && *p <= '9') ms = ms * 10 + (*p++ - '0');
        if (ms > 65535) ms = 65535;
        evil_eeprom_write_byte(EV_DELAY);
        evil_eeprom_write_byte((uint8_t)(ms >> 8));
        evil_eeprom_write_byte((uint8_t)(ms & 0xFF));
    } else if (MATCH("REPEAT ")) {
        int n = 0;
        const char *p = line + 7;
        while (*p >= '0' && *p <= '9') n = n * 10 + (*p++ - '0');
        if (n > 100) n = 100;
        /* Re-emit the previous line's bytecode n times */
        for (int r = 0; r < n; r++) {
            for (uint8_t i = 0; i < last_line_bc_len; i++)
                evil_eeprom_write_byte(last_line_bc[i]);
        }
        return 1;  /* don't update last_line_bc */
    } else if (line[0] == 'F' && line[1] >= '1' && line[1] <= '9') {
        /* F1-F12 */
        int fnum = 0;
        const char *p = line + 1;
        while (*p >= '0' && *p <= '9') fnum = fnum * 10 + (*p++ - '0');
        if (fnum >= 1 && fnum <= 12) {
            evil_eeprom_write_byte(EV_KEY);
            evil_eeprom_write_byte(0);
            evil_eeprom_write_byte((uint8_t)(HID_KEY_F1 + fnum - 1));
        } else return 0;
    } else if (MATCH("GUI") || MATCH("WINDOWS") || MATCH("SUPER")) {
        const char *rest = ARGAFTER("GUI");
        if (MATCH("WINDOWS")) rest = ARGAFTER("WINDOWS");
        if (MATCH("SUPER")) rest = ARGAFTER("SUPER");
        evil_eeprom_write_byte(EV_KEY);
        evil_eeprom_write_byte(KEYBOARD_MODIFIER_LEFTGUI);
        evil_eeprom_write_byte(*rest ? char_to_keycode(*rest) : 0);
    } else if (MATCH("CTRL-ALT ") || MATCH("CONTROL-ALT ")) {
        const char *rest = MATCH("CTRL-ALT ") ? line + 9 : line + 12;
        evil_eeprom_write_byte(EV_KEY);
        evil_eeprom_write_byte(KEYBOARD_MODIFIER_LEFTCTRL | KEYBOARD_MODIFIER_LEFTALT);
        evil_eeprom_write_byte(char_to_keycode(*rest));
    } else if (MATCH("CTRL-SHIFT ") || MATCH("CONTROL-SHIFT ")) {
        const char *rest = MATCH("CTRL-SHIFT ") ? line + 11 : line + 14;
        evil_eeprom_write_byte(EV_KEY);
        evil_eeprom_write_byte(KEYBOARD_MODIFIER_LEFTCTRL | KEYBOARD_MODIFIER_LEFTSHIFT);
        evil_eeprom_write_byte(char_to_keycode(*rest));
    } else if (MATCH("CTRL ") || MATCH("CONTROL ")) {
        const char *rest = MATCH("CTRL ") ? line + 5 : line + 8;
        evil_eeprom_write_byte(EV_KEY);
        evil_eeprom_write_byte(KEYBOARD_MODIFIER_LEFTCTRL);
        evil_eeprom_write_byte(char_to_keycode(*rest));
    } else if (MATCH("ALT ")) {
        evil_eeprom_write_byte(EV_KEY);
        evil_eeprom_write_byte(KEYBOARD_MODIFIER_LEFTALT);
        evil_eeprom_write_byte(char_to_keycode(line[4]));
    } else if (MATCH("SHIFT ")) {
        evil_eeprom_write_byte(EV_KEY);
        evil_eeprom_write_byte(KEYBOARD_MODIFIER_LEFTSHIFT);
        evil_eeprom_write_byte(char_to_keycode(line[6]));
    } else {
        /* Try single-key commands */
        int found = 0;
        for (int i = 0; i < DUCK_KEY_COUNT; i++) {
            if (strcmp(line, duck_keys[i].name) == 0) {
                evil_eeprom_write_byte(EV_KEY);
                evil_eeprom_write_byte(0);
                evil_eeprom_write_byte(duck_keys[i].keycode);
                found = 1;
                break;
            }
        }
        if (!found) return 0;
    }

#undef MATCH
#undef ARGAFTER

    /* Capture this line's bytecode for REPEAT */
    uint32_t bc_len = evil_write_addr - line_start_addr;
    if (bc_len <= sizeof(last_line_bc)) {
        last_line_bc_len = (uint8_t)bc_len;
        const uint8_t *src = (const uint8_t *)line_start_addr;
        for (uint8_t i = 0; i < last_line_bc_len; i++)
            last_line_bc[i] = src[i];
    }
    return 1;
}

/* Evil mode: script upload state */
static int evil_set_mode = 0;  /* 1 = accepting DuckyScript lines */

/* ---- Evil mode: CLI command ---------------------------------------------- */
static void cmd_evil(const char *arg)
{
    if (!(cfg.ctf_solved & 0x08)) return;  /* shouldn't reach here, but guard */

    while (arg && *arg == ' ') arg++;

    if (!arg || !*arg) {
        /* Show current payload */
        const uint8_t *p = (const uint8_t *)EVIL_PAYLOAD_BASE;
        if (*p == EV_END || *p == 0xFF) {
            cdc_println("  (no payload)");
        } else {
            cdc_println("  payload:");
            const uint8_t *end = (const uint8_t *)(DATA_EEPROM_BASE + 2048U);
            while (p < end) {
                uint8_t op = *p++;
                if (op == EV_END || op == 0xFF) break;
                cdc_print("  ");
                switch (op) {
                    case EV_DELAY:
                        cdc_print("DELAY ");
                        cdc_print_u32((uint32_t)(*p << 8) | *(p+1));
                        p += 2;
                        break;
                    case EV_KEY:
                        if (p[0] == 0) {
                            /* Try to find key name */
                            int found = 0;
                            for (int i = 0; i < DUCK_KEY_COUNT; i++) {
                                if (duck_keys[i].keycode == p[1]) {
                                    cdc_print(duck_keys[i].name);
                                    found = 1;
                                    break;
                                }
                            }
                            if (!found) {
                                cdc_print("KEY 0x");
                                static const char hex[] = "0123456789ABCDEF";
                                char hx[3] = {hex[p[1]>>4], hex[p[1]&0xF], 0};
                                cdc_print(hx);
                            }
                        } else {
                            if (p[0] & KEYBOARD_MODIFIER_LEFTCTRL)  cdc_print("CTRL+");
                            if (p[0] & KEYBOARD_MODIFIER_LEFTSHIFT) cdc_print("SHIFT+");
                            if (p[0] & KEYBOARD_MODIFIER_LEFTALT)   cdc_print("ALT+");
                            if (p[0] & KEYBOARD_MODIFIER_LEFTGUI)   cdc_print("GUI+");
                            /* Print the key letter */
                            if (p[1] >= HID_KEY_A && p[1] <= HID_KEY_Z) {
                                char ch = (char)('a' + (p[1] - HID_KEY_A));
                                tud_cdc_write(&ch, 1);
                            } else {
                                cdc_print("0x");
                                static const char hex[] = "0123456789ABCDEF";
                                char hx[3] = {hex[p[1]>>4], hex[p[1]&0xF], 0};
                                cdc_print(hx);
                            }
                        }
                        p += 2;
                        break;
                    case EV_STRING:
                        cdc_print("STRING ");
                        { uint8_t len = *p++;
                            for (uint8_t i = 0; i < len && p < end; i++) {
                                char ch = (char)*p++;
                                tud_cdc_write(&ch, 1);
                            }
                        }
                        break;
                    case EV_KEYDOWN:
                        cdc_print("KEYDOWN ");
                        cdc_print_u32(p[0]); cdc_print(" ");
                        cdc_print_u32(p[1]);
                        p += 2;
                        break;
                    case EV_KEYUP:
                        cdc_print("KEYUP");
                        break;
                    default:
                        cdc_print("??? ");
                        goto show_done;
                }
                cdc_println("");
            }
        show_done:
            cdc_print("  (");
            cdc_print_u32((uint32_t)(p - (const uint8_t *)EVIL_PAYLOAD_BASE));
            cdc_print("/");
            cdc_print_u32((uint32_t)EVIL_PAYLOAD_MAX);
            cdc_println(" bytes)");
        }
        return;
    }

    if (arg[0] == 'c' && arg[1] == 'l') {
        /* evil clear */
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        HAL_FLASHEx_DATAEEPROM_Unlock();
        HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_BYTE,
                                       EVIL_PAYLOAD_BASE, EV_END);
        HAL_FLASHEx_DATAEEPROM_Lock();
        __set_PRIMASK(primask);
        cdc_println("  payload cleared");
        return;
    }

    if (arg[0] == 's' && arg[1] == 'e') {
        /* evil set — enter script upload mode */
        /* Erase payload area first byte */
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        HAL_FLASHEx_DATAEEPROM_Unlock();
        HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_BYTE,
                                       EVIL_PAYLOAD_BASE, EV_END);
        HAL_FLASHEx_DATAEEPROM_Lock();
        __set_PRIMASK(primask);
        evil_write_addr = EVIL_PAYLOAD_BASE;
        last_line_bc_len = 0;
        evil_set_mode = 1;
        cdc_println("  enter DuckyScript (END to finish):");
        cdc_print("evil> ");
        tud_cdc_write_flush();
        return;
    }

    if (arg[0] == 'h') {
        /* evil help */
        cdc_println("");
        cdc_println(" Evil mode (USB Rubber Ducky):");
        cdc_println("  Arm: advanced evil on, hold strings 3+4, plug in");
        cdc_println("");
        cdc_println(" DuckyScript commands:");
        cdc_println("  STRING <text>     Type text");
        cdc_println("  DELAY <ms>        Wait milliseconds");
        cdc_println("  ENTER TAB ESCAPE BACKSPACE DELETE SPACE");
        cdc_println("  UPARROW DOWNARROW LEFTARROW RIGHTARROW");
        cdc_println("  GUI <key>         Win/Super+key");
        cdc_println("  CTRL <key>        Ctrl+key");
        cdc_println("  ALT <key>         Alt+key");
        cdc_println("  SHIFT <key>       Shift+key");
        cdc_println("  CTRL-ALT <key>    Ctrl+Alt+key");
        cdc_println("  CTRL-SHIFT <key>  Ctrl+Shift+key");
        cdc_println("  F1-F12            Function keys");
        cdc_println("  REPEAT <n>        Repeat previous line");
        cdc_println("  REM <text>        Comment (ignored)");
        cdc_println("");
        cdc_println(" Upload: evil set   (type script, END to finish)");
        cdc_println(" View:   evil");
        cdc_println(" Clear:  evil clear");
        return;
    }

    if (arg[0] == 'h' && arg[1] == 'e' && arg[2] == 'x' && arg[3] == ' ') {
        /* evil hex <bytes> — raw bytecode upload */
        const char *p = arg + 4;
        evil_write_addr = EVIL_PAYLOAD_BASE;
        while (*p) {
            while (*p == ' ') p++;
            if (!*p) break;
            uint8_t byte = 0;
            for (int n = 0; n < 2 && *p; n++) {
                byte <<= 4;
                char c = *p++;
                if (c >= '0' && c <= '9') byte |= (uint8_t)(c - '0');
                else if (c >= 'a' && c <= 'f') byte |= (uint8_t)(c - 'a' + 10);
                else if (c >= 'A' && c <= 'F') byte |= (uint8_t)(c - 'A' + 10);
            }
            evil_eeprom_write_byte(byte);
        }
        evil_eeprom_write_byte(EV_END);
        cdc_println("  payload written (raw)");
        return;
    }

    cdc_println("  evil [set|clear|help|hex <data>]");
}

/* ---- CTF challenge 4: timing side-channel pin check -----------------------*/
static void cmd_pin(const char *arg)
{
    if (!arg || !*arg) {
        cdc_println("  usage: pin <code>");
        return;
    }
    uint32_t start = HAL_GetTick();
    /* Length gate — correct length costs a small extra check */
    uint8_t len = 0;
    while (arg[len]) len++;
    if (len != CTF4_PIN_LEN) {
        /* wrong length — fast reject, ~1-2ms */
        uint32_t elapsed = HAL_GetTick() - start;
        cdc_print("  auth: failed [");
        cdc_print_u32(elapsed);
        cdc_println("ms]");
        return;
    }
    /* Per-digit comparison with deliberate timing leak */
    bool ok = true;
    for (uint8_t i = 0; i < CTF4_PIN_LEN; i++) {
        if ((arg[i] ^ CTF4_KEY) != ctf4_pin_enc[i]) {
            ok = false;
            break;
        }
        uint32_t wait_end = HAL_GetTick() + 47;
        while (HAL_GetTick() < wait_end) tud_task();
    }
    uint32_t elapsed = HAL_GetTick() - start;
    if (ok) {
        char fbuf[CTF4_FLAG_LEN + 1];
        for (uint8_t i = 0; i < CTF4_FLAG_LEN; i++)
            fbuf[i] = (char)(ctf4_flag_enc[i] ^ CTF4_KEY);
        fbuf[CTF4_FLAG_LEN] = '\0';
        cdc_print("  auth: ok [");
        cdc_print_u32(elapsed);
        cdc_println("ms]");
        cdc_print("  "); cdc_println(fbuf);
        cfg.ctf_solved |= 0x08;
        cfg_save();
    } else {
        cdc_print("  auth: failed [");
        cdc_print_u32(elapsed);
        cdc_println("ms]");
    }
}

/* ---- Command dispatch table -----------------------------------------------*/
typedef void (*cmd_fn_t)(const char *arg);
static const struct { const char *name; cmd_fn_t fn; } commands[] = {
{ "help",       cmd_help },
{ "idle",       cmd_mode },
{ "play",       cmd_playmode },
{ "brightness", cmd_brightness },
{ "speed",      cmd_speed },
{ "scale",      cmd_scale },
{ "volume",     cmd_volume },
{ "buzzer",     cmd_buzzer },
{ "note",       cmd_note },
{ "name",       cmd_name },
{ "status",     cmd_status },
{ "version",    cmd_version },
{ "id",         cmd_id },
{ "cycle",      cmd_cycle },
{ "advanced",   cmd_advanced },
{ "reset",      cmd_reset },
{ "pin",        cmd_pin },
};

/* ---- CLI line editor (VT100-compatible) ---------------------------------- */
#define CMD_COUNT ((int)(sizeof(commands) / sizeof(commands[0])))
#define CLI_BUFLEN 64

/* Argument name lists for tab completion and ghost suggestions */
static const char * const idle_args[] = {
"breathe", "wave", "sparkle", "scanner", "pulse",
"travel", "ripple", "chase", "segments", "glitter",
"pingpong", "interfere", "reflect", "flicker", "gradient",
"off", NULL
};
static const char * const play_args[] = { "radiate", "vu", "momentum", NULL };
static const char * const scale_args[] = { "low", "high", NULL };
static const char * const onoff_args[] = { "on", "off", NULL };
static const char * const speed_args[] = {
"x0.25", "x0.50", "x0.75", "x1.00", "x1.50", "x2.00", "x4.00", NULL
};
static const uint16_t speed_values[] = { 25, 50, 75, 100, 150, 200, 400 };
#define SPEED_PRESET_COUNT 7

/* advanced subcommand names for completion */
static const char * const adv_sub_names[] = {
"help", "reset", "timeout", "shutdown", "powersave",
"debounce", "maxhold", "tdetect", "trelease", "tmin",
"tdeb", "rdeb", "tremolo", "vibrato", "notes",
"sleepanim", "wakescan", "wakeanim", "dfu", "evil", NULL
};

/* Parallel to commands[] — NULL means no string arg completion */
static const char * const *cmd_args[] = {
NULL,        /* help */
idle_args,   /* idle */
play_args,   /* play */
NULL,        /* brightness */
speed_args,  /* speed */
scale_args,  /* scale */
NULL,        /* volume */
onoff_args,  /* buzzer */
NULL,        /* note */
NULL,        /* name */
NULL,        /* status */
NULL,        /* version */
NULL,        /* id */
NULL,        /* cycle */
adv_sub_names, /* advanced */
NULL,        /* reset */
NULL,        /* pin */
};

/* Interactive picker state */
#define PICKER_NONE   0
#define PICKER_SINGLE 1  /* ≤9 items: single digit, immediate */
#define PICKER_ENTER  2  /* >9 items: multi-digit, wait for Enter */
#define PICKER_CYCLE  3  /* cycle selection: comma/space list, wait for Enter */
static int  picker_mode = PICKER_NONE;
static int  picker_cmd_idx = -1;
static int  picker_count = 0;
static char picker_buf[24];
static int  picker_buf_len = 0;

/* Get current value index for a picker command */
static int picker_current(int cmd_idx)
{
    if (cmd_idx == 1) return (int)idle_mode;           /* idle */
    if (cmd_idx == 2) return (int)cfg.play_mode;       /* play */
    if (cmd_idx == 5) return (int)cfg.scale;            /* scale */
    if (cmd_idx == 7) return (int)cfg.buzzer_on;        /* buzzer */
    if (cmd_idx == 4) {                                 /* speed — find closest preset */
        int best = 0, bdist = 9999;
        for (int i = 0; i < SPEED_PRESET_COUNT; i++) {
            int d = (int)cfg.speed - (int)speed_values[i];
            if (d < 0) d = -d;
            if (d < bdist) { bdist = d; best = i; }
        }
        return best;
    }
    return -1;
}

static void cli_show_picker(int cmd_idx)
{
    const char * const *args = cmd_args[cmd_idx];
    if (!args) return;
    int count = 0;
    while (args[count]) count++;
    int current = picker_current(cmd_idx);
    for (int i = 0; i < count; i++) {
        cdc_print("  "); cdc_print_u32((uint32_t)(i + 1)); cdc_print(". ");
        cdc_print(args[i]);
        if (i == current) cdc_print("  <--");
        cdc_println("");
    }
    cdc_print("  select [1-"); cdc_print_u32((uint32_t)count); cdc_print("]: ");
    tud_cdc_write_flush();
    picker_cmd_idx = cmd_idx;
    picker_count = count;
    picker_buf_len = 0;
    picker_mode = (count > 9) ? PICKER_ENTER : PICKER_SINGLE;
}

static void cli_show_cycle_picker(void)
{
    for (int i = 0; i < ANIM_COUNT; i++) {
        cdc_print("  "); cdc_print_u32((uint32_t)(i + 1));
        if (i < 9) cdc_print(".  "); else cdc_print(". ");
        cdc_print(mode_names[i]);
        if (cfg.cycle_mask & (1U << i)) cdc_print("  *");
        cdc_println("");
    }
    cdc_print("  select (all/off/1,3,5): ");
    tud_cdc_write_flush();
    picker_mode = PICKER_CYCLE;
    picker_cmd_idx = -1;
    picker_count = ANIM_COUNT;
    picker_buf_len = 0;
}

static char    cli_buf[CLI_BUFLEN];
static int     cli_len  = 0;   /* chars in buffer */
static int     cli_cur  = 0;   /* cursor position */
static uint8_t cli_esc  = 0;   /* 0=normal, 1=ESC, 2=CSI, 3=CSI+digit */
static char    cli_esc_par = 0;

static int  cli_ghost_len = 0;  /* length of dim suggestion currently displayed */

/* ---- Command history (circular buffer of 5) ------------------------------ */
#define HIST_COUNT 5
static char hist_buf[HIST_COUNT][CLI_BUFLEN];
static int  hist_len[HIST_COUNT];  /* length of each stored command */
static int  hist_head = 0;         /* next slot to write */
static int  hist_size = 0;         /* number of stored entries (0..5) */
static int  hist_pos  = -1;        /* -1 = not browsing; 0..hist_size-1 */

static void hist_push(void)
{
    if (cli_len == 0 || cli_len >= CLI_BUFLEN) return;
    /* Don't store duplicates of the last command */
    if (hist_size > 0) {
        int prev = (hist_head - 1 + HIST_COUNT) % HIST_COUNT;
        if (hist_len[prev] == cli_len &&
            memcmp(hist_buf[prev], cli_buf, (size_t)cli_len) == 0)
            return;
    }
    memcpy(hist_buf[hist_head], cli_buf, (size_t)cli_len);
    hist_len[hist_head] = cli_len;
    hist_head = (hist_head + 1) % HIST_COUNT;
    if (hist_size < HIST_COUNT) hist_size++;
}

/* Forward declaration */
static void cli_clear_ghost(void);

static void cli_replace_line(const char *text, int len)
{
    cli_clear_ghost();
    /* Move cursor to start of input */
    for (int i = 0; i < cli_cur; i++) tud_cdc_write("\b", 1);
    /* Write new text */
    tud_cdc_write(text, (uint32_t)len);
    /* Clear leftover chars */
    for (int i = len; i < cli_len; i++) tud_cdc_write(" ", 1);
    for (int i = len; i < cli_len; i++) tud_cdc_write("\b", 1);
    memcpy(cli_buf, text, (size_t)len);
    cli_len = cli_cur = len;
}

static void cli_clear_ghost(void)
{
    /* Erase ghost chars after cli_len, then return cursor */
    for (int i = 0; i < cli_ghost_len; i++) tud_cdc_write(" ", 1);
    for (int i = 0; i < cli_ghost_len; i++) tud_cdc_write("\b", 1);
    cli_ghost_len = 0;
}

static void cli_show_ghost(void)
{
    cli_clear_ghost();
    if (cli_cur != cli_len || cli_len == 0) return;

    /* Find first space — determines if we're completing command or argument */
    int sp = -1;
    for (int i = 0; i < cli_len; i++) {
        if (cli_buf[i] == ' ') { sp = i; break; }
    }

    if (sp < 0) {
        /* No space: suggest command name */
        for (int i = 0; i < CMD_COUNT; i++) {
            if (strncmp(cli_buf, commands[i].name, (size_t)cli_len) == 0) {
                const char *tail = commands[i].name + cli_len;
                int tlen = (int)strlen(tail);
                if (tlen > 0) {
                    tud_cdc_write("\033[90m", 5);
                    tud_cdc_write(tail, (uint32_t)tlen);
                    tud_cdc_write("\033[0m", 4);
                    for (int j = 0; j < tlen; j++) tud_cdc_write("\b", 1);
                    cli_ghost_len = tlen;
                }
                break;
            }
        }
    } else {
        /* Space found: suggest argument */
        /* Find which command */
        char cmd_word[CLI_BUFLEN];
        memcpy(cmd_word, cli_buf, (size_t)sp);
        cmd_word[sp] = '\0';
        int cmd_idx = -1;
        for (int i = 0; i < CMD_COUNT; i++) {
            if (strcmp(cmd_word, commands[i].name) == 0) { cmd_idx = i; break; }
        }
        if (cmd_idx < 0 || cmd_idx >= CMD_COUNT) return;
        const char * const *args = cmd_args[cmd_idx];
        if (!args) return;

        const char *prefix = cli_buf + sp + 1;
        int plen = cli_len - sp - 1;
        if (plen <= 0) return;

        for (int i = 0; args[i]; i++) {
            if (strncmp(prefix, args[i], (size_t)plen) == 0) {
                const char *tail = args[i] + plen;
                int tlen = (int)strlen(tail);
                if (tlen > 0) {
                    tud_cdc_write("\033[90m", 5);
                    tud_cdc_write(tail, (uint32_t)tlen);
                    tud_cdc_write("\033[0m", 4);
                    for (int j = 0; j < tlen; j++) tud_cdc_write("\b", 1);
                    cli_ghost_len = tlen;
                }
                break;
            }
        }
    }
}

static void cli_accept_ghost(void)
{
    if (cli_ghost_len == 0 || cli_cur != cli_len) return;

    /* Accept ghost: append ghost_len chars to buffer */
    /* The ghost text is already on screen; move cursor right by ghost_len */
    if (cli_len + cli_ghost_len >= CLI_BUFLEN - 1) return;

    /* Re-derive the ghost tail text to copy into buffer */
    int sp = -1;
    for (int i = 0; i < cli_len; i++) {
        if (cli_buf[i] == ' ') { sp = i; break; }
    }

    if (sp < 0) {
        /* Command word ghost */
        for (int i = 0; i < CMD_COUNT; i++) {
            if (strncmp(cli_buf, commands[i].name, (size_t)cli_len) == 0) {
                const char *tail = commands[i].name + cli_len;
                int tlen = (int)strlen(tail);
                if (tlen > 0 && cli_len + tlen < CLI_BUFLEN - 1) {
                    cli_clear_ghost();
                    memcpy(cli_buf + cli_len, tail, (size_t)tlen);
                    tud_cdc_write(tail, (uint32_t)tlen);
                    cli_len += tlen;
                    cli_cur = cli_len;
                }
                break;
            }
        }
    } else {
        /* Argument ghost */
        char cmd_word[CLI_BUFLEN];
        memcpy(cmd_word, cli_buf, (size_t)sp);
        cmd_word[sp] = '\0';
        int cmd_idx = -1;
        for (int i = 0; i < CMD_COUNT; i++) {
            if (strcmp(cmd_word, commands[i].name) == 0) { cmd_idx = i; break; }
        }
        if (cmd_idx < 0 || cmd_idx >= CMD_COUNT) return;
        const char * const *args = cmd_args[cmd_idx];
        if (!args) return;

        const char *prefix = cli_buf + sp + 1;
        int plen = cli_len - sp - 1;
        if (plen <= 0) return;

        for (int i = 0; args[i]; i++) {
            if (strncmp(prefix, args[i], (size_t)plen) == 0) {
                const char *tail = args[i] + plen;
                int tlen = (int)strlen(tail);
                if (tlen > 0 && cli_len + tlen < CLI_BUFLEN - 1) {
                    cli_clear_ghost();
                    memcpy(cli_buf + cli_len, tail, (size_t)tlen);
                    tud_cdc_write(tail, (uint32_t)tlen);
                    cli_len += tlen;
                    cli_cur = cli_len;
                }
                break;
            }
        }
    }
}

static void cli_prompt(void)
{
    cdc_print("> ");
    tud_cdc_write_flush();
}

static void cli_insert(char ch)
{
    if (cli_len >= CLI_BUFLEN - 1) return;
    cli_clear_ghost();
    memmove(cli_buf + cli_cur + 1, cli_buf + cli_cur, (size_t)(cli_len - cli_cur));
    cli_buf[cli_cur] = ch;
    cli_len++;
    tud_cdc_write(cli_buf + cli_cur, (uint32_t)(cli_len - cli_cur));
    cli_cur++;
    for (int i = 0; i < cli_len - cli_cur; i++) tud_cdc_write("\b", 1);
    cli_show_ghost();
}

static void cli_backspace(void)
{
    if (cli_cur <= 0) return;
    cli_clear_ghost();
    cli_cur--;
    memmove(cli_buf + cli_cur, cli_buf + cli_cur + 1, (size_t)(cli_len - cli_cur - 1));
    cli_len--;
    tud_cdc_write("\b", 1);
    tud_cdc_write(cli_buf + cli_cur, (uint32_t)(cli_len - cli_cur));
    tud_cdc_write(" \b", 2);
    for (int i = 0; i < cli_len - cli_cur; i++) tud_cdc_write("\b", 1);
    cli_show_ghost();
}

static void cli_delete_at(void)
{
    if (cli_cur >= cli_len) return;
    cli_clear_ghost();
    memmove(cli_buf + cli_cur, cli_buf + cli_cur + 1, (size_t)(cli_len - cli_cur - 1));
    cli_len--;
    tud_cdc_write(cli_buf + cli_cur, (uint32_t)(cli_len - cli_cur));
    tud_cdc_write(" \b", 2);
    for (int i = 0; i < cli_len - cli_cur; i++) tud_cdc_write("\b", 1);
    cli_show_ghost();
}

static void cli_tab(void)
{
    if (cli_len == 0) return;
    cli_clear_ghost();

    /* Find first space */
    int sp = -1;
    for (int i = 0; i < cli_len; i++) {
        if (cli_buf[i] == ' ') { sp = i; break; }
    }

    if (sp < 0) {
        /* Complete command name */
        int mc = 0, lm = -1;
        for (int i = 0; i < CMD_COUNT; i++) {
            if (strncmp(cli_buf, commands[i].name, (size_t)cli_len) == 0) { mc++; lm = i; }
        }
        if (mc == 1) {
            const char *name = commands[lm].name;
            int nlen = (int)strlen(name);
            for (int i = 0; i < cli_cur; i++) tud_cdc_write("\b", 1);
            int old_len = cli_len;
            memcpy(cli_buf, name, (size_t)nlen);
            cli_buf[nlen] = ' ';
            cli_len = cli_cur = nlen + 1;
            tud_cdc_write(cli_buf, (uint32_t)cli_len);
            for (int i = cli_len; i < old_len; i++) tud_cdc_write(" ", 1);
            for (int i = cli_len; i < old_len; i++) tud_cdc_write("\b", 1);
        } else if (mc > 1) {
            tud_cdc_write("\r\n", 2);
            for (int i = 0; i < CMD_COUNT; i++) {
                if (strncmp(cli_buf, commands[i].name, (size_t)cli_len) == 0) {
                    cdc_print(commands[i].name); cdc_print("  ");
                }
            }
            tud_cdc_write("\r\n", 2);
            cdc_print("> ");
            tud_cdc_write(cli_buf, (uint32_t)cli_len);
            for (int i = 0; i < cli_len - cli_cur; i++) tud_cdc_write("\b", 1);
        }
    } else {
        /* Complete argument */
        char cmd_word[CLI_BUFLEN];
        memcpy(cmd_word, cli_buf, (size_t)sp);
        cmd_word[sp] = '\0';
        int cmd_idx = -1;
        for (int i = 0; i < CMD_COUNT; i++) {
            if (strcmp(cmd_word, commands[i].name) == 0) { cmd_idx = i; break; }
        }
        if (cmd_idx < 0 || cmd_idx >= CMD_COUNT) return;
        const char * const *args = cmd_args[cmd_idx];
        if (!args) return;

        const char *prefix = cli_buf + sp + 1;
        int plen = cli_len - sp - 1;

        int mc = 0, lm = -1;
        for (int i = 0; args[i]; i++) {
            if (plen == 0 || strncmp(prefix, args[i], (size_t)plen) == 0) { mc++; lm = i; }
        }
        if (mc == 1) {
            const char *n = args[lm];
            int nl = (int)strlen(n);
            /* Erase from after the space */
            for (int i = 0; i < plen; i++) tud_cdc_write("\b", 1);
            int old_len = cli_len;
            memcpy(cli_buf + sp + 1, n, (size_t)nl);
            cli_buf[sp + 1 + nl] = ' ';
            cli_len = cli_cur = sp + 1 + nl + 1;
            tud_cdc_write(cli_buf + sp + 1, (uint32_t)(cli_len - sp - 1));
            for (int i = cli_len; i < old_len; i++) tud_cdc_write(" ", 1);
            for (int i = cli_len; i < old_len; i++) tud_cdc_write("\b", 1);
        } else if (mc > 1) {
            tud_cdc_write("\r\n", 2);
            for (int i = 0; args[i]; i++) {
                if (plen == 0 || strncmp(prefix, args[i], (size_t)plen) == 0) {
                    cdc_print(args[i]); cdc_print("  ");
                }
            }
            tud_cdc_write("\r\n", 2);
            cdc_print("> ");
            tud_cdc_write(cli_buf, (uint32_t)cli_len);
        }
    }
}

static void cli_execute(void)
{
    cli_clear_ghost();
    tud_cdc_write("\r\n", 2);
    tud_cdc_write_flush();
    /* Trim trailing spaces */
    while (cli_len > 0 && cli_buf[cli_len - 1] == ' ') cli_len--;
    cli_buf[cli_len] = '\0';

    if (cli_len > 0) {
        hist_push();
        hist_pos = -1;
        char *arg = memchr(cli_buf, ' ', (size_t)cli_len);
        if (arg) { *arg = '\0'; arg++; }

        int found = 0;
        for (int i = 0; i < CMD_COUNT; i++) {
            if (strcmp(cli_buf, commands[i].name) == 0) {
                commands[i].fn(arg);
                found = 1;
                break;
            }
        }
        if (!found && (cfg.ctf_solved & 0x08) && strcmp(cli_buf, "evil") == 0) {
            cmd_evil(arg);
            found = 1;
        }
        if (!found) {
            cdc_print("  unknown: ");
            cdc_print(cli_buf);
            cdc_println(" (try 'help')");
        }
    }

    tud_cdc_write_flush();
    cli_len = cli_cur = 0;
    cli_prompt();
}

static void cli_process_char(char ch)
{
    if (cli_esc == 1) {
        cli_esc = (uint8_t)((ch == '[') ? 2 : 0);
        return;
    }
    if (cli_esc == 2) {
        cli_esc = 0;
        switch (ch) {
            case 'C':
                if (cli_ghost_len > 0 && cli_cur == cli_len) { cli_accept_ghost(); }
                else if (cli_cur < cli_len) { cli_cur++; tud_cdc_write("\033[C", 3); }
                break;
            case 'D': if (cli_cur > 0)       { cli_cur--; tud_cdc_write("\033[D", 3); } break;
            case 'A': /* up arrow — history back */
                if (hist_size > 0) {
                    int next = (hist_pos < 0) ? 0 : hist_pos + 1;
                    if (next < hist_size) {
                        hist_pos = next;
                        int idx = (hist_head - 1 - hist_pos + HIST_COUNT * 2) % HIST_COUNT;
                        cli_replace_line(hist_buf[idx], hist_len[idx]);
                    }
                }
                break;
            case 'B': /* down arrow — history forward */
                if (hist_pos > 0) {
                    hist_pos--;
                    int idx = (hist_head - 1 - hist_pos + HIST_COUNT * 2) % HIST_COUNT;
                    cli_replace_line(hist_buf[idx], hist_len[idx]);
                } else if (hist_pos == 0) {
                    hist_pos = -1;
                    cli_replace_line("", 0);
                }
                break;
            case 'H': while (cli_cur > 0)    { cli_cur--; tud_cdc_write("\b", 1); } break;
            case 'F':
                if (cli_ghost_len > 0 && cli_cur == cli_len) { cli_accept_ghost(); }
                else if (cli_cur < cli_len) { tud_cdc_write(cli_buf + cli_cur, (uint32_t)(cli_len - cli_cur)); cli_cur = cli_len; }
                break;
            case '3': case '1': case '4':
                cli_esc = 3; cli_esc_par = ch; break;
            default: break;
        }
        return;
    }
    if (cli_esc == 3) {
        cli_esc = 0;
        if (ch == '~') {
            if (cli_esc_par == '3') cli_delete_at();
            else if (cli_esc_par == '1') { while (cli_cur > 0) { cli_cur--; tud_cdc_write("\b", 1); } }
            else if (cli_esc_par == '4') { if (cli_cur < cli_len) { tud_cdc_write(cli_buf + cli_cur, (uint32_t)(cli_len - cli_cur)); cli_cur = cli_len; } }
        }
        return;
    }

    /* Evil set: DuckyScript line-by-line input */
    if (evil_set_mode) {
        if (ch == '\r' || ch == '\n') {
            cdc_print("\r\n");
            cli_buf[cli_len] = '\0';
            if (cli_len == 0 || strcmp(cli_buf, "END") == 0 || strcmp(cli_buf, "end") == 0) {
                /* Finish: write END opcode */
                evil_eeprom_write_byte(EV_END);
                evil_set_mode = 0;
                cdc_print("  payload saved (");
                cdc_print_u32((uint32_t)(evil_write_addr - EVIL_PAYLOAD_BASE));
                cdc_print("/");
                cdc_print_u32((uint32_t)EVIL_PAYLOAD_MAX);
                cdc_println(" bytes)");
            } else {
                if (!evil_compile_line(cli_buf)) {
                    cdc_print("  error: "); cdc_println(cli_buf);
                }
                cdc_print("evil> ");
            }
            tud_cdc_write_flush();
            cli_len = cli_cur = 0;
        } else if (ch == '\b' || ch == 0x7F) {
            if (cli_len > 0) {
                cli_len--; cli_cur--;
                tud_cdc_write("\b \b", 3); tud_cdc_write_flush();
            }
        } else if (ch == 0x1B) {
            /* ESC: cancel */
            evil_eeprom_write_byte(EV_END);
            evil_set_mode = 0;
            cdc_println("\r\n  cancelled");
            cli_len = cli_cur = 0;
            cli_prompt(); tud_cdc_write_flush();
        } else if (ch >= 0x20 && ch <= 0x7E && cli_len < CLI_BUFLEN - 1) {
            cli_buf[cli_len++] = ch;
            cli_cur = cli_len;
            tud_cdc_write(&ch, 1); tud_cdc_write_flush();
        }
        return;
    }

    /* Picker / cycle input mode */
    if (picker_mode == PICKER_SINGLE) {
        /* ≤9 items: single digit, immediate */
        int sel = ch - '1';
        if (sel >= 0 && sel < picker_count) {
            const char * const *args = cmd_args[picker_cmd_idx];
            if (picker_cmd_idx == 4 && sel < SPEED_PRESET_COUNT) {
                cfg.speed = speed_values[sel]; cfg_save();
                cdc_println("");
                cdc_print("  speed: x");
                cdc_print_u32(cfg.speed / 100); cdc_print(".");
                cdc_print_u32((cfg.speed / 10) % 10);
                cdc_print_u32(cfg.speed % 10); cdc_println("");
            } else if (args && args[sel]) {
                cdc_println(""); commands[picker_cmd_idx].fn(args[sel]);
            }
        } else { cdc_println(""); }
        picker_mode = PICKER_NONE;
        cli_prompt(); tud_cdc_write_flush();
        return;
    }
    if (picker_mode == PICKER_ENTER || picker_mode == PICKER_CYCLE) {
        if (ch == '\r' || ch == '\n') {
            picker_buf[picker_buf_len] = '\0';
            cdc_println("");
            if (picker_mode == PICKER_ENTER) {
                int sel = 0;
                for (int i = 0; i < picker_buf_len; i++)
                    if (picker_buf[i] >= '0' && picker_buf[i] <= '9')
                        sel = sel * 10 + picker_buf[i] - '0';
                sel--;
                if (sel >= 0 && sel < picker_count && picker_cmd_idx >= 0) {
                    const char * const *args = cmd_args[picker_cmd_idx];
                    if (args && args[sel]) commands[picker_cmd_idx].fn(args[sel]);
                }
            } else {
                /* PICKER_CYCLE: parse "all", "off", or comma/space-separated indices */
                if (picker_buf[0] == 'a') {
                    cfg.cycle_mask = (uint16_t)((1U << ANIM_COUNT) - 1);
                    cfg_save(); cdc_println("  cycling: all");
                } else if (picker_buf[0] == 'o') {
                    cfg.cycle_mask = 0; cfg_save(); cdc_println("  cycling: off");
                } else {
                    uint16_t mask = 0;
                    const char *p = picker_buf;
                    while (*p) {
                        while (*p == ',' || *p == ' ') p++;
                        if (*p >= '0' && *p <= '9') {
                            int v = 0;
                            while (*p >= '0' && *p <= '9') v = v * 10 + (*p++ - '0');
                            v--;
                            if (v >= 0 && v < ANIM_COUNT) mask |= (uint16_t)(1U << v);
                        } else break;
                    }
                    if (mask) {
                        cfg.cycle_mask = mask; cfg_save();
                        cdc_print("  cycling:");
                        for (int i = 0; i < ANIM_COUNT; i++)
                            if (mask & (1U << i)) { cdc_print(" "); cdc_print(mode_names[i]); }
                        cdc_println("");
                    }
                }
            }
            picker_mode = PICKER_NONE;
            cli_prompt(); tud_cdc_write_flush();
        } else if (ch == '\b' || ch == 0x7F) {
            if (picker_buf_len > 0) {
                picker_buf_len--;
                tud_cdc_write("\b \b", 3); tud_cdc_write_flush();
            }
        } else if (ch == 0x1B) {
            cdc_println("");
            picker_mode = PICKER_NONE;
            cli_prompt(); tud_cdc_write_flush();
        } else if (ch >= 0x20 && ch < 0x7F && picker_buf_len < 23) {
            picker_buf[picker_buf_len++] = ch;
            tud_cdc_write(&ch, 1); tud_cdc_write_flush();
        }
        return;
    }

    if (ch == '\r' || ch == '\n')         cli_execute();
    else if (ch == '\b' || ch == 0x7F)    cli_backspace();
    else if (ch == '\t')                  cli_tab();
    else if (ch == 0x1B)                  cli_esc = 1;
    else if (ch >= 0x20 && ch <= 0x7E)    cli_insert(ch);
    /* Control chars silently ignored — no echo of ESC sequences */
}

/* ---- Power management -----------------------------------------------------*/

/* Set LED + buzzer GPIO pins to analog input to cut leakage in STOP */
static void pm_gpio_analog(void)
{
    /* MODER: 11 = analog.  Each field is 2 bits. */
    /* GPIOB: PB3(6), PB10(Buz), PB11(0), PB13(1), PB14(2) */
    GPIOB->MODER |= (3U << (3*2)) | (3U << (10*2)) | (3U << (11*2))
        | (3U << (13*2)) | (3U << (14*2));
    /* GPIOC: PC6(3), PC7(4) */
    GPIOC->MODER |= (3U << (6*2)) | (3U << (7*2));
    /* GPIOA: PA15(5) */
    GPIOA->MODER |= (3U << (15*2));
}

/* Configure RTC wakeup timer for ~4-second intervals using LSI.
 * LSI ≈ 37 kHz.  WUCKSEL=0b000 → ck_rtc/16 ≈ 2313 Hz (no calendar needed).
 * WUTR = 9251 → ~4.0 s interval.
 * EXTI line 20 = RTC wakeup, rising edge trigger + interrupt enable. */
static void pm_rtc_wakeup_init(void)
{
    /* Enable PWR and RTC APB clocks */
    __HAL_RCC_PWR_CLK_ENABLE();

    /* Enable write access to RTC domain (PWR_CR DBP bit) */
    PWR->CR |= PWR_CR_DBP;

    /* Start LSI */
    RCC->CSR |= RCC_CSR_LSION;
    while (!(RCC->CSR & RCC_CSR_LSIRDY)) {}   /* wait for LSI ready */

    /* Select LSI as RTC clock and enable RTC */
    RCC->CSR = (RCC->CSR & ~RCC_CSR_RTCSEL) | RCC_CSR_RTCSEL_LSI;
    RCC->CSR |= RCC_CSR_RTCEN;

    /* Unlock RTC write-protection */
    RTC->WPR = 0xCAU;
    RTC->WPR = 0x53U;

    /* Disable wakeup timer so we can reconfigure it */
    RTC->CR &= ~RTC_CR_WUTE;
    while (!(RTC->ISR & RTC_ISR_WUTWF)) {}    /* wait until writes allowed */

    /* WUCKSEL = 000b → ck_rtc / 16 ≈ 37000/16 ≈ 2313 Hz
     * WUTR = 9251 → (9251+1)/2313 ≈ 4.0 seconds */
    RTC->CR  &= ~RTC_CR_WUCKSEL;              /* bits [2:0] = 000 */
    RTC->WUTR = 9251U;

    /* Clear any pending wakeup flag before enabling */
    RTC->ISR &= ~RTC_ISR_WUTF;

    /* Enable wakeup interrupt and re-enable wakeup timer */
    RTC->CR |= RTC_CR_WUTIE | RTC_CR_WUTE;

    /* Relock RTC */
    RTC->WPR = 0xFFU;

    /* EXTI line 20 (RTC wakeup): rising edge trigger, interrupt unmask */
    EXTI->RTSR |= EXTI_RTSR_RT20;
    EXTI->IMR  |= EXTI_IMR_IM20;

    /* Enable RTC_IRQn in NVIC */
    NVIC_SetPriority(RTC_IRQn, 3U);
    NVIC_EnableIRQ(RTC_IRQn);
}

/* Forward declaration — defined below with other TSC bank code */
static uint32_t pm_tsc_acquire(int bank);

/* cfg.touch_deb and cfg.resolve_deb are now cfg.touch_deb / cfg.resolve_deb */
#define TOUCH_ECS_SHIFT     5  /* baseline IIR alpha ~ 1/32 */

static uint8_t touch_scan(void)
{
    uint8_t mask = 0;
    for (int i = 0; i < 6; i++) {
        touch_ch[i].raw = pm_tsc_acquire(i);
        if (touch_ch[i].baseline == 0) {
            touch_ch[i].baseline = touch_ch[i].raw;
            continue;
        }
        touch_ch[i].delta = (int16_t)((int32_t)touch_ch[i].baseline -
                                      (int32_t)touch_ch[i].raw);
        uint16_t th_det = (uint16_t)(touch_ch[i].baseline * cfg.touch_detect / 100);
        uint16_t min_det = (uint16_t)cfg.touch_min * 10U;
        if (th_det < min_det) th_det = min_det;
        uint16_t th_rel = (uint16_t)(touch_ch[i].baseline * cfg.touch_release / 100);
        if (!touch_ch[i].touched) {
            if (touch_ch[i].delta >= (int16_t)th_det) {
                if (++touch_ch[i].deb_detect >= cfg.touch_deb) {
                    touch_ch[i].touched = 1;
                    touch_ch[i].deb_detect = 0;
                }
                touch_ch[i].deb_release = 0;
            } else {
                touch_ch[i].deb_detect = 0;
                int32_t err = (int32_t)touch_ch[i].raw - (int32_t)touch_ch[i].baseline;
                touch_ch[i].baseline = (uint32_t)((int32_t)touch_ch[i].baseline +
                                                  (err >> TOUCH_ECS_SHIFT));
            }
        } else {
            if (touch_ch[i].delta <= (int16_t)th_rel) {
                if (++touch_ch[i].deb_release >= cfg.touch_deb) {
                    touch_ch[i].touched = 0;
                    touch_ch[i].deb_release = 0;
                }
                touch_ch[i].deb_detect = 0;
            } else {
                touch_ch[i].deb_release = 0;
            }
        }
        if (touch_ch[i].touched) mask |= (1U << i);
    }
    touch_mask = mask;
    return mask;
}

/* Resolve multi-touch to single string */
static int touch_resolve(void)
{
    static int  prev_result = -1;
    static uint8_t deb_cnt = 0;

    if (!touch_mask) { prev_result = -1; deb_cnt = 0; return -1; }

    int lo = -1, hi = -1, best = -1, best_d = 0;
    int sum_id = 0, sum_d = 0;
    for (int i = 0; i < 6; i++) {
        if (!(touch_mask & (1U << i))) continue;
        int d = touch_ch[i].delta; if (d < 0) d = 0;
        if (lo < 0) lo = i;
        hi = i;
        sum_id += i * d;
        sum_d  += d;
        if (d > best_d) { best_d = d; best = i; }
    }
    if (sum_d <= 0) return prev_result;

    int contig = 1;
    for (int i = lo; i <= hi; i++)
        if (!(touch_mask & (1U << i))) { contig = 0; break; }

    int raw = contig ? (sum_id + sum_d / 2) / sum_d : best;

    if (raw == prev_result) {
        deb_cnt = 0;
        return raw;
    }
    if (++deb_cnt >= cfg.resolve_deb) {
        prev_result = raw;
        deb_cnt = 0;
        return raw;
    }
    return prev_result;
}

static const struct { uint32_t io_msk; uint32_t grp_msk; int grp_idx; } tsc_banks[] = {
{ TSC_GROUP1_IO2, TSC_GROUP1, 0 },
{ TSC_GROUP1_IO3, TSC_GROUP1, 0 },
{ TSC_GROUP1_IO4, TSC_GROUP1, 0 },
{ TSC_GROUP2_IO2, TSC_GROUP2, 1 },
{ TSC_GROUP2_IO3, TSC_GROUP2, 1 },
{ TSC_GROUP2_IO4, TSC_GROUP2, 1 },
};

static uint32_t pm_tsc_acquire(int bank)
{
    HAL_TSC_IODischarge(&htsc, ENABLE);
    for (volatile int d = 0; d < 200; d++) {}

    htsc.Instance->IOCCR  = tsc_banks[bank].io_msk;
    htsc.Instance->IOGCSR = tsc_banks[bank].grp_msk;

    HAL_TSC_IODischarge(&htsc, DISABLE);
    HAL_TSC_Start(&htsc);
    HAL_TSC_PollForAcquisition(&htsc);

    uint32_t count = htsc.Instance->IOGXCR[tsc_banks[bank].grp_idx];
    HAL_TSC_Stop(&htsc);
    return count;
}

/* Self-calibrating touch detection: acquire all 6 channels and look for
 * outliers. Touch lowers the count on 1-2 channels. If any channel is
 * >10% below the median of all 6, someone is touching.
 * No stored baselines needed — compares channels against each other. */
static bool pm_touch_detected(void)
{
    uint32_t counts[6];
    for (int i = 0; i < 6; i++)
        counts[i] = pm_tsc_acquire(i);

    for (int i = 0; i < 5; i++)
        for (int j = i + 1; j < 6; j++)
            if (counts[i] > counts[j]) {
                uint32_t t = counts[i]; counts[i] = counts[j]; counts[j] = t;
            }
    uint32_t median = (counts[2] + counts[3]) / 2;

    /* If the lowest channel is >10% below median, it's a touch */
    uint32_t threshold = median / 10;
    if (threshold < 30) threshold = 30;
    return (median > counts[0]) && ((median - counts[0]) > threshold);
}

/* Stop all LED PWM output (zero CCRs, stop channels) */
static void pm_leds_off(void)
{
    for (int i = 0; i < LED_COUNT; i++) led_set(i, 0);
    TIM2->CCR3 = 0;  /* buzzer off */
    HAL_TIM_PWM_Stop(&htim2,  TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim2,  TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim2,  TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim2,  TIM_CHANNEL_4);
    HAL_TIM_PWM_Stop(&htim21, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim21, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim22, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim22, TIM_CHANNEL_2);
}

/* Restart LED PWM after wake */
static void pm_leds_restart(void)
{
    MX_TIM2_Init();
    MX_TIM21_Init();
    MX_TIM22_Init();
    HAL_TIM_PWM_Start(&htim2,  TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2,  TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2,  TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2,  TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim21, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim21, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim22, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim22, TIM_CHANNEL_2);
}

static uint32_t sleep_cycles = 0;

static void pm_enter_stop(void)
{
    pm_leds_off();
    pm_gpio_analog();

    for (;;) {
        RTC->WPR = 0xCAU;
        RTC->WPR = 0x53U;
        RTC->ISR &= ~RTC_ISR_WUTF;
        RTC->WPR = 0xFFU;
        EXTI->PR = EXTI_PR_PIF20;

        HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
        SystemClock_Config();
        MX_TSC_Init();

        if (pm_touch_detected()) {
            MX_TOUCHSENSING_Init();
            tsl_user_Init();
            last_touch_time = HAL_GetTick();
            sleep_cycles = 0;
            pm_leds_restart();
            if (cfg.wake_touch_anim) {
                /* Gentle outward sweep from center */
                for (int r = 0; r <= 3; r++) {
                    for (int i = 0; i < LED_COUNT; i++) {
                        int d = i - 3; if (d < 0) d = -d;
                        if (d <= r) led_set(i, max_brightness * (r+1-d)/(r+1));
                    }
                    HAL_Delay(120);
                }
                HAL_Delay(300);
                /* Fade out */
                for (int s = 10; s >= 0; s--) {
                    for (int i = 0; i < LED_COUNT; i++)
                        led_set(i, led_duty[6-i] * s / 10);
                    HAL_Delay(40);
                }
            }
            return;
        }
        /* Spurious wake — check standby shutdown */
        sleep_cycles++;
        if (cfg.shutdown_min > 0 && sleep_cycles >= (uint32_t)cfg.shutdown_min * 15U) {
            /* Disable RTC wakeup so it doesn't wake us from standby */
            RTC->WPR = 0xCAU;
            RTC->WPR = 0x53U;
            RTC->CR &= ~(RTC_CR_WUTE | RTC_CR_WUTIE);
            RTC->ISR &= ~RTC_ISR_WUTF;
            RTC->WPR = 0xFFU;
            EXTI->PR = EXTI_PR_PIF20;
            /* Clear PWR wakeup flag — required or standby exits immediately */
            PWR->CR |= PWR_CR_CWUF;
            /* Full power off — wake only by power cycle */
            HAL_PWR_EnterSTANDBYMode();
        }
        if (cfg.wake_scan_anim) {
            MX_TIM22_Init();
            HAL_TIM_PWM_Start(&htim22, TIM_CHANNEL_1);
            if (cfg.wake_scan_anim == 2) {
                /* heartbeat: lub-dub */
                TIM22->CCR1 = TIM22->ARR * 15 / 100;
                HAL_Delay(40);
                TIM22->CCR1 = 0;
                HAL_Delay(60);
                TIM22->CCR1 = TIM22->ARR * 10 / 100;
                HAL_Delay(30);
            } else {
                /* blink */
                TIM22->CCR1 = TIM22->ARR * 12 / 100;
                HAL_Delay(50);
            }
            TIM22->CCR1 = 0;
            HAL_TIM_PWM_Stop(&htim22, TIM_CHANNEL_1);
        }
    }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

    /* USER CODE BEGIN 1 */
    SCB->VTOR = APP_BASE_ADDR;
    __enable_irq();   /* bootloader left PRIMASK=1 */
    dfu_boot_flag = 0UL;
    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */
    /* CRS: trim HSI48 from USB SOF */
    __HAL_RCC_CRS_CLK_ENABLE();
    RCC_CRSInitTypeDef crs = {
    .Prescaler             = RCC_CRS_SYNC_DIV1,
    .Source                = RCC_CRS_SYNC_SOURCE_USB,
    .Polarity              = RCC_CRS_SYNC_POLARITY_RISING,
    .ReloadValue           = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000U, 1000U),
    .ErrorLimitValue       = RCC_CRS_ERRORLIMIT_DEFAULT,
    .HSI48CalibrationValue = 0x20U,
};
    HAL_RCCEx_CRSConfig(&crs);
    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_TIM2_Init();
    MX_TIM21_Init();
    MX_TIM22_Init();
    MX_TSC_Init();
    MX_TOUCHSENSING_Init();
    MX_USB_PCD_Init();
    /* USER CODE BEGIN 2 */
    /* USB init */
    __HAL_RCC_USB_CLK_ENABLE();
    NVIC_SetPriority(USB_IRQn, 0U);
    NVIC_EnableIRQ(USB_IRQn);
    cfg_load();
    cfg_apply();

    /* Evil mode boot detection */
    if (cfg.evil_enabled && (cfg.ctf_solved & 0x08)) {
        uint32_t c[6];
        for (int i = 0; i < 6; i++) c[i] = pm_tsc_acquire(i);
        uint32_t avg = (c[0] + c[1] + c[4] + c[5]) / 4;
        uint32_t thresh = avg - avg / 10;
        if (c[2] < thresh && c[3] < thresh) evil_mode = 1;
    }

    /* Start all PWM channels (LEDs 0-6 + buzzer) */
    TIM2->CCR2 = TIM2->ARR;        /* LED 6 full-on briefly */
    TIM2->CCR4 = TIM2->ARR / 2U;   /* LED 0 half-on briefly */
    HAL_TIM_PWM_Start(&htim2,  TIM_CHANNEL_1);   /* LED 5 */
    HAL_TIM_PWM_Start(&htim2,  TIM_CHANNEL_2);   /* LED 6 */
    HAL_TIM_PWM_Start(&htim2,  TIM_CHANNEL_3);   /* Buzzer */
    HAL_TIM_PWM_Start(&htim2,  TIM_CHANNEL_4);   /* LED 0 */
    HAL_TIM_PWM_Start(&htim21, TIM_CHANNEL_1);   /* LED 1 */
    HAL_TIM_PWM_Start(&htim21, TIM_CHANNEL_2);   /* LED 2 */
    HAL_TIM_PWM_Start(&htim22, TIM_CHANNEL_1);   /* LED 3 */
    HAL_TIM_PWM_Start(&htim22, TIM_CHANNEL_2);   /* LED 4 */



    tusb_init();
    led_wave(3U, 160U, 20U, 2U);


    if (evil_mode) {
        evil_run();
    }

    usb_probe_start = HAL_GetTick();
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    uint32_t uptime_tick = HAL_GetTick();

    while (1)
    {
        if (usb_connected || usb_probing) tud_task();

        /* --- Touch scan + multi-touch resolution -------------------------------- */
        touch_scan();
        int detected_key = touch_resolve();

        cli_detected_key = detected_key;

        /* --- Uptime counter (every 1 s) ---------------------------------------- */
        if ((HAL_GetTick() - uptime_tick) >= 1000U) {
            uptime_tick += 1000U;
            uptime_sec++;
        }

        /* --- State machine + animation (every 20 ms) --------------------------- */
        if ((HAL_GetTick() - anim_tick) >= 5U) {
            anim_tick = HAL_GetTick();
            if (morse_active) {
                morse_play_loop(); /* never returns */
            } else switch (badge_state) {
                    case ST_IDLE:
                        if (detected_key >= 0) {
                            if (cfg.buzzer_on) buzzer_set_note(detected_key);
                            prev_key = detected_key;
                            melody_record((uint8_t)detected_key);
                            ripple_center = detected_key;
                            last_string_change = HAL_GetTick();
                            badge_state = ST_PLAYING;
                            state_enter_tick = HAL_GetTick();
                            last_touch_time = HAL_GetTick();
                            vu_width16 = 0;
                            momentum_pos = (int16_t)(ripple_center << 8);
                            momentum_vel = 0;
                        } else {
                            if (idle_mode < ANIM_COUNT) {
                                /* Cycle only if cycle_mask is set */
                                if (cfg.cycle_mask && (HAL_GetTick() - mode_start) > 60000U) {
                                    uint8_t next = idle_mode;
                                    for (int t = 0; t < ANIM_COUNT; t++) {
                                        next = (uint8_t)((next + 1U) % ANIM_COUNT);
                                        if (cfg.cycle_mask & (1U << next)) break;
                                    }
                                    idle_mode = next;
                                    mode_start = HAL_GetTick();
                                }
                                static const anim_fn_t idle_anims[] = {
                                anim_breathe, anim_nb_wave, anim_sparkle,
                                anim_scanner, anim_pulse, anim_travel, anim_ripple,
                                anim_chase, anim_segments, anim_glitter,
                                anim_pingpong, anim_interfere, anim_reflect,
                                anim_flicker, anim_gradient
                            };
                                idle_anims[idle_mode](anim_tick);
                            } else {
                                for (int i = 0; i < LED_COUNT; i++) led_set(i, 0);
                            }
                        }
                        break;
                    case ST_PLAYING:
                        last_touch_time = HAL_GetTick();
                        if (detected_key >= 0) {
                            if (detected_key != prev_key &&
                                (HAL_GetTick() - last_string_change) >= cfg.debounce_ms) {
                                momentum_dir = detected_key - prev_key;
                                if (cfg.buzzer_on) buzzer_set_note(detected_key);
                                prev_key = detected_key;
                                melody_record((uint8_t)detected_key);
                                ripple_center = detected_key;
                                last_string_change = HAL_GetTick();
                            }
                            if ((HAL_GetTick() - state_enter_tick) > cfg.maxhold_ms) {
                                buzzer_set_note(-1);
                                prev_key = -1;
                                decay_intensity = 255;
                                maxhold_locked = true;
                                badge_state = ST_RELEASE_DECAY;
                                state_enter_tick = HAL_GetTick();
                            }
                        } else {
                            buzzer_set_note(-1);
                            decay_intensity = 255;
                            badge_state = ST_RELEASE_DECAY;
                            state_enter_tick = HAL_GetTick();
                        }
                        if (badge_state == ST_PLAYING) {
                            play_anims[cfg.play_mode % 3](anim_tick);
                            if (detected_key >= 0) fx_update();
                        }
                        break;
                    case ST_RELEASE_DECAY: {
                        uint32_t dp = cfg.maxhold_ms > 2000 ? cfg.maxhold_ms : 2000;
                        uint32_t elapsed = HAL_GetTick() - state_enter_tick;
                        if (detected_key < 0) maxhold_locked = false;
                        if (detected_key >= 0 && !maxhold_locked) {
                            if (cfg.buzzer_on) buzzer_set_note(detected_key);
                            prev_key = detected_key;
                            melody_record((uint8_t)detected_key);
                            ripple_center = detected_key;
                            last_string_change = HAL_GetTick();
                            badge_state = ST_PLAYING;
                            state_enter_tick = HAL_GetTick();
                            last_touch_time = HAL_GetTick();
                        } else if (detected_key < 0 && elapsed >= dp) {
                            decay_intensity = 0;
                            prev_key = -1;
                            badge_state = ST_IDLE;
                            state_enter_tick = HAL_GetTick();
                        } else {
                            decay_intensity = (uint8_t)(255 - elapsed * 255 / dp);
                            play_anims[cfg.play_mode % 3](anim_tick);
                        }
                        break;
                    }
                }
        }

        /* --- 3.1 Non-blocking USB probe: check during 5s window --------------- */
        if (usb_probing) {
            if (tud_mounted()) {
                usb_connected = true;
                usb_probing   = false;
            } else if ((HAL_GetTick() - usb_probe_start) > 5000U) {
                usb_probing = false;
                if (cfg.powersave) {
                    USB->BCDR &= ~USB_BCDR_DPPU;
                    USB->CNTR = USB_CNTR_FRES | USB_CNTR_PDWN;
                    NVIC_DisableIRQ(USB_IRQn);
                    __HAL_RCC_USB_CLK_DISABLE();
                    __HAL_RCC_CRS_CLK_DISABLE();
                    RCC->CRRCR &= ~RCC_CRRCR_HSI48ON;
                }
                max_brightness = cfg.brightness;
                pm_rtc_wakeup_init();
                last_touch_time = HAL_GetTick();
            }
        }

        /* --- 3.2 STOP mode: enter after timeout of no touch (battery only) ---- */
        if (!usb_connected && !usb_probing &&
            last_touch_time != 0 &&
            (HAL_GetTick() - last_touch_time) > (uint32_t)cfg.timeout_ds * 100U) {
            if (cfg.sleep_anim) {
                for (int pair = 0; pair < 4; pair++) {
                    for (int s = 7; s >= 0; s--) {
                        if (pair < 3) { led_set(pair, led_duty[6-pair]*s/7); led_set(6-pair, led_duty[pair]*s/7); }
                        else led_set(3, led_duty[3]*s/7);
                        HAL_Delay(19);
                    }
                }
            }
            pm_enter_stop();
        }

        /* --- CDC receive: VT100 line editor with tab completion --------------- */
        if (usb_connected && tud_cdc_available()) {
            char tmp[32];
            uint32_t n = tud_cdc_read(tmp, sizeof(tmp));
            for (uint32_t i = 0; i < n; i++)
                cli_process_char(tmp[i]);
            tud_cdc_write_flush();
        }
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Configure the main internal regulator output voltage
     */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
        |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
    PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
    (void)itf; (void)rts;
    if (dtr) {
        cdc_println("");
        if (cfg.ctf_solved & 0x04) {
            cdc_println(" _  _                 __  __         _");
            cdc_println("| || |__ _ _ _ _ __  |  \\/  |__ _ __| |_ ___ _ _");
            cdc_println("| __ / _` | '_| '_ \\ | |\\/| / _` (_-<  _/ -_) '_|");
            cdc_println("|_||_\\__,_|_| | .__/ |_|  |_\\__,_/__/\\__\\___|_|");
            cdc_println("              |_|");
        } else {
            cdc_println(" ___ ___ _    _          ___       _    _ _");
            cdc_println("| _ ) __(_)__| |___ ___ |   \\ _  _| |__| (_)_ _");
            cdc_println("| _ \\__ \\ / _` / -_|_-< | |) | || | '_ \\ | | ' \\");
            cdc_println("|___/___/_\\__,_\\___/__/ |___/ \\_,_|_.__/_|_|_||_|");
        }
        cdc_println("");
        cdc_print(" Harp v0.9");
        if (cfg.name[0]) {
            cdc_print(" -- ");
            cdc_print(cfg.name);
        }
        cdc_println("");
        if (cfg.ctf_solved & 0x08) {
            cdc_print(" Evil mode: ");
            cdc_println(cfg.evil_enabled ? "ARMED" : "disarmed");
        }
        cdc_println(" Type 'help' for commands, touch the strings to play");
        cdc_println("");
        cli_len = cli_cur = 0;
        hist_pos = -1;
        cli_prompt();
    }
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    (void)file; (void)line;
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
