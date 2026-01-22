/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_hal_tim.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bme280.h"
#include "i2c-lcd.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c4;

UART_HandleTypeDef huart3;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
/* USER CODE BEGIN PV */
static float temperature_ctrl = 0.0f;
static float temperature_tel  = 0.0f;

float temperature, huminidity;
volatile float temp_zadana = 22.0f;
int32_t pressure;

/* --- PID params --- */
static float kp = 11.5f; //main 11.5 aggresive 16.2519f
static float ki = 0.0458f; //main 0.0458 aggresive 0.0604f
static float kd = 0.0f; //main 0.0 aggresive 33.0543

static float pid_i = 0.0f;
static float pid_prev_y = 0.0f;

/* --- output / limiter --- */
static float u_prev = 0.0f;          // dla limitera narastania
static float DU_UP = 6.0f;
static float DU_DN = 20.0f;
static float control_u = 0.0f;       // 0..100 (%)

/* overshoot handling */
static const float OS_MARGIN = 0.15f;   // °C - kiedy uznajemy przebitkę
static const float DU_DN_OS_MULT = 2.0f;


/* integrator unwind on overshoot */
static const float I_UNWIND = 0.85f;

/* --- OPEN safety (hard limit only) --- */
static const float OPEN_HARD_MAX = 65.0f;  // °C (bezpiecznik fizyczny)

/* --- mode --- */
typedef enum {
    MODE_PID = 0,
    MODE_OPEN = 1
} control_mode_t;

static control_mode_t ctrl_mode = MODE_PID;
static float out_manual = 0.0f;

/* --- UART RX line buffer --- */
static char rx_line[64];
static uint8_t rx_len = 0;
static uint8_t rx_ch = 0;

/* --- scheduling --- */
static uint32_t last_pid_ms = 0;
static uint32_t last_tel_ms = 0;

/* setpoint limits */
#define TEMP_MIN 10.0f
#define TEMP_MAX 60.0f

#define TEMP_STEP_BTN   0.1f
#define BTN_DEBOUNCE_MS 120U
/* USER CODE END PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C4_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static float clampf(float x, float lo, float hi);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/

/* USER CODE BEGIN 0 */
static uint32_t last_btn_plus_ms  = 0;
static uint32_t last_btn_minus_ms = 0;

//static void encoder_update(void)
//{
//    int16_t now  = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
//    int16_t diff = now - enc_last;
//    enc_last = now;
//
//    enc_acc += diff;
//
//    while (enc_acc >= ENC_TICKS_PER_STEP)
//    {
//        temp_zadana += TEMP_STEP;
//        enc_acc -= ENC_TICKS_PER_STEP;
//    }
//
//    while (enc_acc <= -ENC_TICKS_PER_STEP)
//    {
//        temp_zadana -= TEMP_STEP;
//        enc_acc += ENC_TICKS_PER_STEP;
//    }
//
//    if (temp_zadana < TEMP_MIN) temp_zadana = TEMP_MIN;
//    if (temp_zadana > TEMP_MAX) temp_zadana = TEMP_MAX;
//}
static float lowpass_ctrl(float x)
{
    static uint8_t init = 0;
    static float y = 0.0f;
    const float alpha = 0.40f;  // szybszy tor dla regulatora

    if (!init) { y = x; init = 1; }
    else { y = y + alpha * (x - y); }

    return y;
}

static float lowpass_tel(float x)
{
    static uint8_t init = 0;
    static float y = 0.0f;
    const float alpha = 0.20f;  // wolniejszy tor do podglądu/telemetrii

    if (!init) { y = x; init = 1; }
    else { y = y + alpha * (x - y); }

    return y;
}

static void adjust_setpoint(float delta)
{
    temp_zadana = clampf(temp_zadana + delta, TEMP_MIN, TEMP_MAX);
}

static void uart_print(const char *s)
{
    HAL_UART_Transmit(&huart3, (uint8_t*)s, (uint16_t)strlen(s), 100);
}

static void uart_printf(const char *fmt, ...)
{
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0) {
        HAL_UART_Transmit(&huart3, (uint8_t*)buf, (uint16_t)n, 100);
    }
}

static void i2c_scan_bus(const char *name, I2C_HandleTypeDef *hi2c)
{
    uart_printf("\r\n=== I2C scan %s ===\r\n", name);

    int found = 0;
    for (uint16_t addr = 1; addr < 128; addr++)
    {
        if (HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(addr << 1), 2, 10) == HAL_OK)
        {
            uart_printf("FOUND: 0x%02X\r\n", addr);
            found = 1;
        }
    }

    if (!found) {
        uart_print("NO DEVICES FOUND\r\n");
    }
}

static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/* PID in "position form" with simple anti-windup by clamping integrator */
static float pid_step(float sp, float y, float dt_s)
{
    if (dt_s <= 1e-6f) dt_s = 1e-6f;

    float e = sp - y;

    float dy = (y - pid_prev_y) / dt_s;
    pid_prev_y = y;

    float p = kp * e;
    float d = -kd * dy;

    if (ki > 1e-6f) {
        pid_i += e * dt_s;

        float i_min = (0.0f   - (p + d)) / ki;
        float i_max = (100.0f - (p + d)) / ki;
        pid_i = clampf(pid_i, i_min, i_max);
    }

    float u = p + (ki * pid_i) + d;
    return clampf(u, 0.0f, 100.0f);
}


static void pid_bumpless_init(float u_now, float sp, float y)
{
    float e = sp - y;
    float p = kp * e;
    // kd masz zwykle 0, ale zostawmy:
    float d = 0.0f;

    if (ki > 1e-6f) {
        pid_i = (u_now - p - d) / ki;
    } else {
        pid_i = 0.0f;
    }
    pid_prev_y = y;
}

static float rate_limit(float u, float du_max)
{
    float du = u - u_prev;
    if (du >  du_max) du =  du_max;
    if (du < -du_max) du = -du_max;
    u_prev += du;
    return u_prev;
}

static float rate_limit_asym(float u, float du_up, float du_dn)
{
    float du = u - u_prev;
    if (du >  du_up) du =  du_up;
    if (du < -du_dn) du = -du_dn;
    u_prev += du;
    return u_prev;
}


static void rate_limit_reset(float u_now)
{
    u_prev = clampf(u_now, 0.0f, 100.0f);
}

/* ---------- UART line handling ---------- */

static void uart_start_rx_it(void)
{
    HAL_UART_Receive_IT(&huart3, &rx_ch, 1);
}

/* Wywoływane z callbacka po odebraniu znaku */
static void uart_handle_char(uint8_t c)
{
    // echo znaków do debug
    // uart_printf("[RX:%c]\r\n", c);

    if (c == '\r') return;

    if (c == '\n') {
        rx_line[rx_len] = '\0';

        uart_printf("LINE='%s'\r\n", rx_line);  // debug: co przyszlo

        float val;

        if (sscanf(rx_line, "SET %f", &val) == 1 ||
            sscanf(rx_line, "SET:%f", &val) == 1 ||
            sscanf(rx_line, "SET=%f", &val) == 1) {

            temp_zadana = clampf(val, TEMP_MIN, TEMP_MAX);
            uart_printf("OK SET %.2f\r\n", temp_zadana);

        }
        else if (sscanf(rx_line, "OUT %f", &val) == 1) {
            out_manual = clampf(val, 0.0f, 100.0f);
            ctrl_mode = MODE_OPEN;
            uart_printf("OK OUT %.1f (OPEN)\r\n", out_manual);

        }
        else if (strncmp(rx_line, "MODE PID", 8) == 0) {
            ctrl_mode = MODE_PID;

            pid_bumpless_init(control_u, temp_zadana, temperature_ctrl);

            // limiter startuje od aktualnego OUT
            rate_limit_reset(control_u);

            uart_print("OK MODE PID\r\n");
        }
        else if (strncmp(rx_line, "MODE OPEN", 9) == 0) {
            ctrl_mode = MODE_OPEN;

            // limiter dopasuj do aktualnego OUT (manual), żeby nie było skoków w logach
            rate_limit_reset(out_manual);

            uart_print("OK MODE OPEN\r\n");
        }

        else if (sscanf(rx_line, "KP %f", &val) == 1) { kp = val; uart_printf("OK KP %.4f\r\n", kp); }
        else if (sscanf(rx_line, "KI %f", &val) == 1) { ki = val; uart_printf("OK KI %.4f\r\n", ki); }
        else if (sscanf(rx_line, "KD %f", &val) == 1) { kd = val; uart_printf("OK KD %.4f\r\n", kd); }
        else if (strncmp(rx_line, "GET", 3) == 0) {
        	uart_printf("T=%.2f SP=%.2f OUT=%.1f\r\n", temperature_tel, temp_zadana, control_u);

        }
        else if (strncmp(rx_line, "HELP", 4) == 0) {
            uart_print("CMD: SET <temp>\r\n");
            uart_print("CMD: OUT <0..100>\r\n");
            uart_print("CMD: MODE PID | MODE OPEN\r\n");
            uart_print("CMD: GET\r\n");
            uart_print("CMD: KP <v> | KI <v> | KD <v>\r\n");
        }
        else if (rx_line[0] != '\0') {
            uart_print("ERR\r\n");
        }


        rx_len = 0;
        return;
    }

    if (rx_len < sizeof(rx_line) - 1) {
        rx_line[rx_len++] = (char)c;
    } else {
        rx_len = 0;
        uart_print("ERR:LINE_OVERFLOW\r\n");
    }
}


static void heater_set_percent(float percent)
{
	percent = clampf(percent, 0.0f, 100.0f);

	    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim4);   // np. 999
	    uint32_t ccr = (uint32_t)((percent * (float)(arr + 1U) / 100.0f) + 0.5f);

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, ccr);
}

static uint8_t is_valid_temp(float t)
{
    return (t > -40.0f && t < 85.0f); // zakres BME280
}
/* USER CODE END 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

int main(void)
{
  /* USER CODE BEGIN 1 */
  // #define MAX_LENGTH 30
  // char text[MAX_LENGTH]; // aktualnie nieużywane
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_I2C4_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  MX_TIM3_Init();
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  heater_set_percent(50.0f);
  uart_start_rx_it();   // HAL_UART_Receive_IT(&huart3, &rx_ch, 1);

  uart_print("\r\nBOOT OK\r\n");
  uart_print("UART=115200 8N1\r\n");

  i2c_scan_bus("I2C4 (LCD)", &hi2c4);
  i2c_scan_bus("I2C1 (BME)", &hi2c1);

  lcd_init();
  lcd_clear();
  lcd_send_string_at(0, 0, "Start BME280...");

  uart_start_rx_it();
  uart_print("Type HELP + Enter\r\n");

  if (bme280_init(&hi2c1,
                  BME280_OVERSAMPLE_X1,
                  BME280_OVERSAMPLE_X1,
                  BME280_OVERSAMPLE_X1,
                  BME280_NORMAL_MODE,
                  BME280_STANDBY_125MS,
                  BME280_FILTER_OFF) != 0)
  {
      lcd_send_string_at(1, 0, "BME init ERROR");
      uart_print("BME init ERROR\r\n");
      Error_Handler();
  }

  lcd_clear();
  lcd_send_string_at(0, 0, "BME280 OK");

  rate_limit_reset(50.0f);
  pid_bumpless_init(50.0f, temp_zadana, temperature);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint32_t last = HAL_GetTick();
  char line[21];

  while (1)
  {


//      if (HAL_GetTick() - last >= 1000)
//      {
//          last = HAL_GetTick();
//
//          temperature = bme280_get_temperature();
//          pressure    = (int32_t)bme280_get_pressure();
//          huminidity  = bme280_get_humidity();
//
//          // UART (ciągły podgląd)
////          uart_printf("T=%.2fC H=%.2f%% P=%ldPa\r\n",
////                      temperature, huminidity, (long)pressure);
//          int16_t cnt = enc_raw();
//
//          uart_printf("CNT=%d diff_acc=%d  Tzad=%.2f  T=%.2f H=%.2f P=%ld\r\n",
//                      (int)cnt, (int)enc_acc, temp_zadana,
//                      temperature, huminidity, (long)pressure);
//
//          // LCD 20x4 (nadpisujemy linie spacjami aby nie migało)
//          snprintf(line, sizeof(line), "Temp: %6.2f C      ", temperature);
//          lcd_send_string_at(0, 0, line);
//
//          snprintf(line, sizeof(line), "Hum : %6.2f %%     ", huminidity);
//          lcd_send_string_at(1, 0, line);
//
////          snprintf(line, sizeof(line), "Pres: %8ld Pa   ", (long)pressure);
////          lcd_send_string_at(2, 0, line);
//
////          snprintf(line, sizeof(line), "ENC CNT:%6d      ", (int)cnt);
////          lcd_send_string_at(2, 0, line);
//
//          snprintf(line, sizeof(line), "ENC:%6d A:%6d", (int)cnt, (int)enc_acc);
//                  lcd_send_string_at(2, 0, line);
//
//          snprintf(line, sizeof(line), "Tzad: %6.2f C      ", temp_zadana);
//          lcd_send_string_at(3, 0, line);
//      }

  uint32_t now_ms = HAL_GetTick();

  /* PID loop every 100 ms */
  if (now_ms - last_pid_ms >= 100) {

      uint32_t dt_ms = now_ms - last_pid_ms;
      last_pid_ms = now_ms;

      float dt_s = (dt_ms > 0) ? (dt_ms / 1000.0f) : 0.1f;

      float t_raw = bme280_get_temperature();

      float t_ctrl = lowpass_ctrl(t_raw);   // do regulacji
      float t_tel  = lowpass_tel(t_raw);    // do logu/LCD

      if (!is_valid_temp(t_ctrl)) {
          control_u = 0.0f;
          heater_set_percent(0.0f);
          uart_print("ERR:INVALID_TEMP\r\n");
          continue;
      }

      temperature = t_ctrl;
      temperature_tel = t_tel;

      if (ctrl_mode == MODE_OPEN) {

          if (temperature >= OPEN_HARD_MAX) {
              control_u = 0.0f;
              heater_set_percent(0.0f);
              uart_print("WARN:OPEN_HARD_MAX\r\n");
          } else {
              control_u = clampf(out_manual, 0.0f, 100.0f);
              heater_set_percent(control_u);
          }

      } else { // MODE_PID

          float sp = temp_zadana;
          float y  = t_ctrl;
          float e  = sp - y;

          float dy = (y - pid_prev_y) / (dt_s > 1e-6f ? dt_s : 1e-6f);
          pid_prev_y = y;

          float p = kp * e;
          float d = -kd * dy;

          // --- integrator ---
          if (ki > 1e-9f) {
              pid_i += e * dt_s;

              // KLUCZ: clamp integratora tak, aby u mogło być w [0..100]
              // (zapobiega ogromnemu ujemnemu I po długim u=0)
              float i_min = (0.0f   - (p + d)) / ki;
              float i_max = (100.0f - (p + d)) / ki;
              pid_i = clampf(pid_i, i_min, i_max);
          } else {
              pid_i = 0.0f;
          }

          float u_unsat = p + (ki * pid_i) + d;
          float u_sat   = clampf(u_unsat, 0.0f, 100.0f);

          // --- dynamiczny limiter: szybciej tnij gdy za gorąco, szybciej grzej gdy za zimno ---
          const float MARGIN = 0.12f;     // 0.1–0.2°C
          float du_up = DU_UP;
          float du_dn = DU_DN;

          if (y > sp + MARGIN) {
              du_dn = DU_DN * 2.0f;       // szybciej w dół gdy przebijasz w górę
          }
          if (y < sp - MARGIN) {
              du_up = DU_UP * 2.0f;       // szybciej w górę gdy przebiłeś w dół (to naprawia Twój problem)
          }

          float u_lim = rate_limit_asym(u_sat, du_up, du_dn);

          // --- back-calculation (tylko jeśli limiter coś zmienił) ---
          if (ki > 1e-9f && (u_lim != u_sat)) {
              // delikatnie, żeby nie pompowało
              const float Kb = 0.5f;
              pid_i += Kb * (u_lim - u_unsat) / ki;
          }

          control_u = u_lim;
          heater_set_percent(control_u);
      }

  }


  if (now_ms - last_tel_ms >= 500) {
      last_tel_ms = now_ms;

      snprintf(line, sizeof(line), "T:%5.2f SP:%5.2f", temperature_tel, temp_zadana);
      lcd_send_string_at(0, 0, line);

      snprintf(line, sizeof(line), "OUT:%5.1f%%        ", control_u);
      lcd_send_string_at(1, 0, line);

      snprintf(line, sizeof(line), "KP:%4.1f KI:%4.2f", kp, ki);
      lcd_send_string_at(2, 0, line);

      snprintf(line, sizeof(line), "KD:%4.2f          ", kd);
      lcd_send_string_at(3, 0, line);

      uart_printf("T=%.2f SP=%.2f OUT=%.1f KP=%.2f KI=%.2f KD=%.2f\r\n",
                  temperature_tel, temp_zadana, control_u, kp, ki, kd);
      uart_printf("DBG_LOOP: SP=%.2f\r\n", temp_zadana);

  }

  HAL_Delay(1);
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00808CD2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C4_Init(void)
{

  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x00808CD2;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  GPIO_InitStruct.Pin  = BTN_PLUS_Pin | BTN_MINUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

static void MX_TIM3_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;

  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 6;

  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 6;

  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM4_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;                 // 72MHz/(71+1)=1MHz
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;                   // 1MHz/(999+1)=1kHz
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;                       // start 0%
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /* ważne: ustawia GPIO AF przez HAL_TIM_MspPostInit */
  HAL_TIM_MspPostInit(&htim4);
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        uart_handle_char(rx_ch);
        HAL_UART_Receive_IT(&huart3, &rx_ch, 1);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t now = HAL_GetTick();

    if (GPIO_Pin == BTN_PLUS_Pin) {
    	HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
        if (now - last_btn_plus_ms >= BTN_DEBOUNCE_MS) {
            last_btn_plus_ms = now;
            adjust_setpoint(+TEMP_STEP_BTN);
        }
    }
    else if (GPIO_Pin == BTN_MINUS_Pin) {
    	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        if (now - last_btn_minus_ms >= BTN_DEBOUNCE_MS) {
            last_btn_minus_ms = now;
            adjust_setpoint(-TEMP_STEP_BTN);
        }
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
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

