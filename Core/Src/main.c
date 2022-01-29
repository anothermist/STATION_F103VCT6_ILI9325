/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include "ili9325.h"
#include "xpt2046.h"
#include "uart.h"

#include "string.h"
#include "bme280.h"
#include "ds3231.h"
#include "at24.h"

#include "fonts/DejaVu_Sans/008_DejaVu_Sans.h"
#include "fonts/DejaVu_Sans/009_DejaVu_Sans.h"
#include "fonts/DejaVu_Sans/010_DejaVu_Sans.h"
#include "fonts/DejaVu_Sans/011_DejaVu_Sans.h"
#include "fonts/DejaVu_Sans/012_DejaVu_Sans.h"
#include "fonts/DejaVu_Sans/014_DejaVu_Sans.h"
#include "fonts/DejaVu_Sans/016_DejaVu_Sans.h"
#include "fonts/DejaVu_Sans/018_DejaVu_Sans.h"
#include "fonts/DejaVu_Sans/020_DejaVu_Sans.h"
#include "fonts/DejaVu_Sans/022_DejaVu_Sans.h"
#include "fonts/DejaVu_Sans/024_DejaVu_Sans.h"
#include "fonts/DejaVu_Sans/026_DejaVu_Sans.h"
#include "fonts/DejaVu_Sans/028_DejaVu_Sans.h"
#include "fonts/DejaVu_Sans/036_DejaVu_Sans.h"
#include "fonts/DejaVu_Sans/048_DejaVu_Sans.h"
#include "fonts/DejaVu_Sans/072_DejaVu_Sans.h"
#include "fonts/DejaVu_Sans/096_DejaVu_Sans.h"
#include "fonts/DejaVu_Sans/112_DejaVu_Sans.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define MAX_PRESSURE 1064
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */
uint8_t  touchIRQ = 0, rtcSet = 1, clearEEPROM = 0, barographViewed = 0, sound = 1, printAlarm = 0, alarm1 = 0;
uint8_t rtcSec, rtcMin, rtcHrs, rtcDay, rtcDate, rtcMonth, rtcYear,
rtcSecA1, rtcMinA1, rtcHrsA1, rtcDayA1, rtcDateA1, rtcMinA2, rtcHrsA2, rtcDayA2, rtcDateA2;
uint16_t touchX, touchY;
uint8_t rtcSecLast = 61, rtcMinLast = 61, rtcHrsLast = 25, rtcDayLast, rtcDateLast, rtcMonthLast, rtcYearLast;
uint16_t pressure, pressureLast;
double temperatureLast, humidityLast, temperature, humidity,
temperatureRemote, temperatureRemoteLast, humidityRemote, humidityRemoteLast, rtcMoon, rtcMoonLast;
uint64_t startHistory;

uint16_t barographHourly[367] = { 25 };
uint16_t barographDaily[367];
uint16_t barographMinimum = 0, barographMaximum = 0, barographAverage = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FSMC_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
double map(double x, double in_min, double in_max, double out_min, double out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint8_t rx_buffer[256];
uint8_t rx_index = 0;
uint8_t rx_data;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
	uint8_t i;
	if (huart->Instance == USART1) {
		if (rx_index == 0) {
			for (i = 0; i < 255; i++) {
				rx_buffer[i] = 0;
			}
		}
		rx_buffer[rx_index++] = rx_data;
		HAL_UART_Receive_IT(&huart1, &rx_data, 1);
	}
}

uint8_t byteL(uint16_t val) {
	return (val & 0xFF);
}

uint8_t byteH(uint16_t val) {
	return ((val >> 8) & 0xFF);
}

uint16_t byteS(uint8_t byteL, uint8_t byteH) {
	return (byteH << 8) | byteL;
}

void bme280(void) {
				temperature = BME280_getTemperature(-1);
				humidity = BME280_getHumidity(-1);
				pressure = (uint16_t)BME280_getPressure();

				if (temperature != temperatureLast && temperature >= -40 && temperature <= 40) {

					char weatherPrintT[4];

					if (temperatureLast >= 10 || (temperatureLast < 0 && temperatureLast > -10)) {
						sprintf(weatherPrintT, "%.1f", temperatureLast);
						LCD_Font(205, 40, weatherPrintT, &DejaVu_Sans_36, 1, BLACK);
					}
					else if (temperatureLast < 10 && temperatureLast > 0) {
						sprintf(weatherPrintT, "%.1f", temperatureLast);
						LCD_Font(231, 40, weatherPrintT, &DejaVu_Sans_36, 1, BLACK);
					}
					else if (temperatureLast <= -10) {
						sprintf(weatherPrintT, "%2d", (int8_t)temperatureLast);
						LCD_Font(205, 40, weatherPrintT, &DejaVu_Sans_36, 1, BLACK);
					}

					if (temperature >= 10 || (temperature < 0 && temperature > -10)) {
						sprintf(weatherPrintT, "%.1f", temperature);
						LCD_Font(205, 40, weatherPrintT, &DejaVu_Sans_36, 1, ORANGE);
					}
					else if (temperature < 10 && temperature > 0) {
						sprintf(weatherPrintT, "%.1f", temperature);
						LCD_Font(231, 40, weatherPrintT, &DejaVu_Sans_36, 1, ORANGE);
					}
					else if (temperature <= -10) {
						sprintf(weatherPrintT, "%2d", (int8_t)temperature);
						LCD_Font(205, 40, weatherPrintT, &DejaVu_Sans_36, 1, ORANGE);
					}

//					for (double i = 0.0; i < 400; i++) {
//						if (temperature > i / 10) LCD_Line(747, 470 - i, 763, 470 - i, 1, GRAY);
//						else LCD_Line(747, 470 - i, 763, 470 - i, 1, 0x101010);
//					}

					temperatureLast = temperature;
				}

				char weatherPrintH[4];

				if (humidity != humidityLast && humidity >= 0 && humidity < 100) {

					sprintf(weatherPrintH, "%.1f", humidityLast);
					if (humidityLast >= 10)
						LCD_Font(105, 40, weatherPrintH, &DejaVu_Sans_36, 1, BLACK);
					else LCD_Font(131, 40, weatherPrintH, &DejaVu_Sans_36, 1, BLACK);


					sprintf(weatherPrintH, "%.1f", humidity);
					if (humidity >= 10)
						LCD_Font(105, 40, weatherPrintH, &DejaVu_Sans_36, 1, CYAN);
					else LCD_Font(131, 40, weatherPrintH, &DejaVu_Sans_36, 1, CYAN);

//					for (double i = 0.0; i < 400; i++) {
//						if (humidity > i / 4) LCD_Line(647, 470 - i, 663, 470 - i, 1, GRAY);
//						else LCD_Line(647, 470 - i, 663, 470 - i, 1, 0x101010);
//					}

					humidityLast = humidity;
				}
	
	

	pressure = (uint16_t)BME280_getPressure();
	if (pressure > 300 && pressure < 1100) {

		for (uint16_t i = 0; i < 367; i++) {
//			barographHourly[i] = byteS(AT24XX_Read(i * 2 + 1000), AT24XX_Read(i * 2 + 1 + 1000));
//			barographDaily[i] = byteS(AT24XX_Read(i * 2 + 2000), AT24XX_Read(i * 2 + 1 + 2000));
		}

		if (barographHourly[0] != rtcHrs && pressure > 300 && pressure < 1100) {

			barographHourly[0] = rtcHrs;

			for (uint16_t i = 1; i < 366; i++) barographHourly[i] = barographHourly[i + 1];
			barographHourly[366] = (uint16_t)pressure;

			for (uint16_t i = 0; i < 367; i++) {
//				AT24XX_Update(i * 2 + 1000, byteL(barographHourly[i]));
//				AT24XX_Update(i * 2 + 1 + 1000, byteH(barographHourly[i]));
			}

			if (barographDaily[0] != rtcDate) {
				barographDaily[0] = rtcDate;

				uint32_t barographAverageLast24Hours = 0;
				uint16_t historyHours = 0;
				for (uint8_t i = 0; i < 24; i++) {
					if (barographHourly[366 - i] != 0) {
						barographAverageLast24Hours = barographAverageLast24Hours + barographHourly[366 - i];
						historyHours++;
					}
				}
				if (historyHours != 0) barographAverageLast24Hours = barographAverageLast24Hours / historyHours;

				for (uint16_t i = 1; i < 366; i++) {
					barographDaily[i] = barographDaily[i + 1];
				}
				barographDaily[366] = barographAverageLast24Hours;

				for (uint16_t i = 0; i < 367; i++) {
//					AT24XX_Update(i * 2 + 2000, byteL(barographDaily[i]));
//					AT24XX_Update(i * 2 + 1 + 2000, byteH(barographDaily[i]));
				}
			}
			barographViewed = 0;
		}

		if (!barographViewed) {

			uint32_t barographAverageLast366Days = 0;
			uint16_t historyDays = 0;
			for (uint16_t i = 0; i < 365; i++) {
				if (barographDaily[366 - i] != 0) {
					barographAverageLast366Days = barographAverageLast366Days + barographDaily[366 - i];
					historyDays++;
				}
			}
			if (historyDays != 0) barographAverage = barographAverageLast366Days / historyDays;

			barographMinimum = barographDaily[366];
			barographMaximum = barographDaily[366];

			for (uint16_t i = 1; i < 366; i++) {
				if (barographDaily[366 - i] != 0) {
					if (barographDaily[366 - i] < barographMinimum) barographMinimum = barographDaily[366 - i];
					if (barographDaily[366 - i] > barographMaximum) barographMaximum = barographDaily[366 - i];
				}
			}

			for (uint16_t i = 1; i < 366; i++) {
				if (barographHourly[366 - i] != 0) {
					if (barographHourly[366 - i] < barographMinimum) barographMinimum = barographHourly[366 - i];
					if (barographHourly[366 - i] > barographMaximum) barographMaximum = barographHourly[366 - i];
				}
			}

			barographDaily[0] = rtcDate;

//			LCD_Rect(1, 201, 368, 128, 1, BLUE);

//			for (uint16_t i = 0; i < 366; i++) {
//				int16_t val = 0;
//				val = barographHourly[i + 1];
//				if (val < barographMaximum - 127) val = barographMaximum - 127;
//				LCD_Line(2 + i, 328, 2 + i, 202, 1, BLACK);
//				LCD_Line(2 + i, 328, 2 + i, 202 + (barographMaximum - val), 1, RGB(255 - ((barographMaximum - val) * 2), 0, 255 - (255 - ((barographMaximum - val) * 2))));
//			}

//			LCD_Rect(1, 329, 368, 128, 1, BLUE);

			for (uint16_t i = 0; i < 366; i++) {
				int16_t val = 0;
				val = barographDaily[i + 1];
				if (val < barographMaximum - 127) val = barographMaximum - 127;

//				LCD_Line(2 + i, 456, 2 + i, 330, 1, BLACK);
//				LCD_Line(2 + i, 456, 2 + i, 330 + (barographMaximum - val), 1, RGB(255 - ((barographMaximum - val) * 2), 0, 255 - (255 - ((barographMaximum - val) * 2))));
			}

//			LCD_Rect_Fill(1, 460, 397, 18, BLACK);

			char s[10];

			if (barographAverage >= 1000) sprintf(s, "Mid:%02d|", barographAverage);
			else sprintf(s, "Mid:0%02d|", barographAverage);
			LCD_Font(1, 474, s, &DejaVu_Sans_18, 1, GRAY);

			if (barographMinimum >= 1000) sprintf(s, "|Min:%02d|", barographMinimum);
			else sprintf(s, "|Min:0%02d|", barographMinimum);
			LCD_Font(91, 474, s, &DejaVu_Sans_18, 1, GRAY);

			if (barographMaximum >= 1000) sprintf(s, "|Max:%02d|", barographMaximum);
			else sprintf(s, "|Max:0%02d|", barographMaximum);
			LCD_Font(188, 474, s, &DejaVu_Sans_18, 1, GRAY);

			if (pressure >= 1000) sprintf(s, "|Now:%02d", pressure);
			else sprintf(s, "|Now:0%02d", pressure);
			LCD_Font(290, 474, s, &DejaVu_Sans_18, 1, GRAY);

			if (barographHourly[365] != barographHourly[366])
			{
//				LCD_Triangle_Fill(370, 302, 398, 302, 384, 316, BLACK);
//				LCD_Triangle_Fill(370, 262, 398, 262, 384, 272, BLACK);
//				LCD_Triangle_Fill(370, 222, 398, 222, 384, 236, BLACK);
//				LCD_Triangle_Fill(370, 316, 398, 316, 384, 302, BLACK);
//				LCD_Triangle_Fill(370, 276, 398, 276, 384, 262, BLACK);
//				LCD_Triangle_Fill(370, 236, 398, 236, 384, 222, BLACK);
//				if (barographHourly[365] > barographHourly[366])     LCD_Triangle_Fill(370, 302, 398, 302, 384, 316, BLUE);
//				if (barographHourly[365] > barographHourly[366] + 1) LCD_Triangle_Fill(370, 262, 398, 262, 384, 272, BLUE);
//				if (barographHourly[365] > barographHourly[366] + 2) LCD_Triangle_Fill(370, 222, 398, 222, 384, 236, BLUE);
//				if (barographHourly[365] < barographHourly[366])     LCD_Triangle_Fill(370, 316, 398, 316, 384, 302, RED);
//				if (barographHourly[365] < barographHourly[366] - 1) LCD_Triangle_Fill(370, 276, 398, 276, 384, 262, RED);
//				if (barographHourly[365] < barographHourly[366] - 2) LCD_Triangle_Fill(370, 236, 398, 236, 384, 222, RED);
			}

			if (barographDaily[365] != barographDaily[366])
			{
//				LCD_Triangle_Fill(370, 430, 398, 430, 384, 444, BLACK);
//				LCD_Triangle_Fill(370, 390, 398, 390, 384, 404, BLACK);
//				LCD_Triangle_Fill(370, 350, 398, 350, 384, 364, BLACK);
//				LCD_Triangle_Fill(370, 444, 398, 444, 384, 430, BLACK);
//				LCD_Triangle_Fill(370, 404, 398, 404, 384, 390, BLACK);
//				LCD_Triangle_Fill(370, 364, 398, 364, 384, 350, BLACK);
//				if (barographDaily[365] > barographDaily[366] + 5)      LCD_Triangle_Fill(370, 430, 398, 430, 384, 444, BLUE);
//				if (barographDaily[365] > barographDaily[366] + 10) 	LCD_Triangle_Fill(370, 390, 398, 390, 384, 404, BLUE);
//				if (barographDaily[365] > barographDaily[366] + 15) 	LCD_Triangle_Fill(370, 350, 398, 350, 384, 364, BLUE);
//				if (barographDaily[365] < barographDaily[366] - 5)     	LCD_Triangle_Fill(370, 444, 398, 444, 384, 430, RED);
//				if (barographDaily[365] < barographDaily[366] - 10) 	LCD_Triangle_Fill(370, 404, 398, 404, 384, 390, RED);
//				if (barographDaily[365] < barographDaily[366] - 15) 	LCD_Triangle_Fill(370, 364, 398, 364, 384, 350, RED);
			}
			barographViewed = 1;
		}
	}
}

void uartDecode() {

//	if (memcmp(rx_buffer, "CE", 2) == 0) {
//		for (uint16_t i = 0; i < 4096; i++) AT24XX_Update(i, 0);

//		uint8_t uartTransmit[] = "EEPROM IS CLEANED\r\n";
//		HAL_UART_Transmit(&huart1, uartTransmit, sizeof(uartTransmit), 100);

////		barograph();
//	}

//	if (memcmp(rx_buffer, "RS", 2) == 0) {

//		char valT[4] = { 0 };

//		for (uint8_t i = 0; i < 4; i++) valT[i] = rx_buffer[2 + i];

//		temperatureRemote = atoi(valT);
//		temperatureRemote = temperatureRemote / 10;

//		char valH[3] = { 0 };

//		for (uint8_t i = 0; i < 3; i++) valH[i] = rx_buffer[6 + i];

//		humidityRemote = atoi(valH);
//		humidityRemote = humidityRemote / 10;
//	}

//	if (memcmp(rx_buffer, "RT", 2) == 0) {

//		char val[2];

//		val[0] = rx_buffer[2];
//		val[1] = rx_buffer[3];
//		DS3231_setHrs(atoi(val));

//		val[0] = rx_buffer[4];
//		val[1] = rx_buffer[5];
//		DS3231_setMin(atoi(val));

//		val[0] = rx_buffer[6];
//		val[1] = rx_buffer[7];
//		DS3231_setSec(atoi(val));

//		val[0] = rx_buffer[8];
//		val[1] = rx_buffer[9];
//		DS3231_setDate(atoi(val));

//		val[0] = rx_buffer[10];
//		val[1] = rx_buffer[11];
//		DS3231_setMonth(atoi(val));

//		val[0] = rx_buffer[12];
//		val[1] = rx_buffer[13];
//		DS3231_setYear(atoi(val));

//		val[1] = rx_buffer[14];
//		DS3231_setDay(atoi(val));
//	}

//	for (uint8_t i = 0; i < 255; i++) rx_buffer[i] = 0;
}

//void signal(void)
//{
//	for (uint32_t i = 0; i <= 65536; i++)
//	{
//		TIM1->CCR1 = i;
//	}
//}

void alarm(void)
{
//	alarm1 = AT24XX_Read(4000);
//	if (rtcHrsA1 < 24 && rtcMinA1 < 60)
//	{
//		if (!printAlarm)
//		{
//			LCD_Rect_Fill(71, 170, 105, 28, BLACK);
//			char alarm1Time[8];
//			sprintf(alarm1Time, "%02d:%02d", rtcHrsA1, rtcMinA1);
//			if (alarm1) LCD_Font(70, 197, alarm1Time, &DejaVu_Sans_36, 1, RED);
//			else LCD_Font(70, 197, alarm1Time, &DejaVu_Sans_36, 1, GRAY);

//			LCD_Rect_Round(2, 170, 30, 30, 2, 1, GRAY);
//			LCD_Font(8, 179, "_", &DejaVu_Sans_36, 1, GRAY);
//			LCD_Rect_Round(34, 170, 30, 30, 2, 1, GRAY);
//			LCD_Font(34, 197, "+", &DejaVu_Sans_36, 1, GRAY);

//			LCD_Rect_Round(182, 170, 30, 30, 2, 1, GRAY);
//			LCD_Font(188, 179, "_", &DejaVu_Sans_36, 1, GRAY);
//			LCD_Rect_Round(214, 170, 30, 30, 2, 1, GRAY);
//			LCD_Font(214, 197, "+", &DejaVu_Sans_36, 1, GRAY);

//			LCD_Rect_Round(260, 170, 130, 30, 2, 1, GRAY);
//			LCD_Rect_Round(300, 175, 50, 20, 2, 1, GRAY);
//			LCD_Font(262, 192, "OFF", &DejaVu_Sans_18, 1, GRAY);
//			LCD_Font(356, 192, "ON", &DejaVu_Sans_18, 1, GRAY);

//			if (alarm1)
//			{
//				LCD_Rect_Round_Fill(329, 176, 20, 18, 1, RED);
//				LCD_Rect_Round_Fill(301, 176, 20, 18, 1, BLACK);
//			}
//			else
//			{
//				LCD_Rect_Round_Fill(301, 176, 20, 18, 1, GRAY);
//				LCD_Rect_Round_Fill(329, 176, 20, 18, 1, BLACK);
//			}

//			printAlarm = 1;
//		}
//		if (alarm1 && rtcHrsA1 == rtcHrs && rtcMinA1 == rtcMin) signal();
//	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FSMC_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

	LCD_Init();
	XPT2046_Init();
	BME280_Init();

	if (clearEEPROM) {
		for (uint16_t i = 0; i < 4096; i++) {
			AT24XX_Update(i, 0);
		}
	}
	
			DS3231_Update();
//		SetRTC(10, 22, 0, 27, 01, 12, 21);

//	LCD_Rect_Fill(0, 0, 800, 480, BLACK);
//	LCD_Rect(0, 0, 798, 479, 1, BLUE);

//	HAL_UART_Receive_IT(&huart1, &rx_data, 1);

//	uint8_t uartTransmit[] = "UART OK\r\n";
//	HAL_UART_Transmit(&huart1, uartTransmit, sizeof(uartTransmit), 100);

//	uint8_t uartTransmitDMA[] = "UART DMA OK\r\n";
//	HAL_UART_Transmit_DMA(&huart1, uartTransmitDMA, sizeof(uartTransmitDMA));

//	LCD_Line(400, 2, 400, 478, 1, BLUE);
//	LCD_Rect(445, 70, 20, 402, 1, CYAN);
//	LCD_Rect(745, 70, 20, 402, 1, CYAN);

//	for (uint32_t i = 0; i < 41; i++) {
//		char numbers[3];
//		sprintf(numbers, "%02d", 40 - i);
//		if (i % 5 == 0) LCD_Font(710, 77 + i * 10, numbers, &DejaVu_Sans_18, 1, GREEN);
//		if (i % 5 == 0 && i > 30) LCD_Font(710, 77 + i * 10, "0", &DejaVu_Sans_18, 1, BLACK);

//		if (i % 5 == 0) LCD_Line(735, 71 + i * 10, 745, 71 + i * 10, 1, CYAN);
//		else LCD_Line(740, 71 + i * 10, 745, 71 + i * 10, 1, CYAN);
//	}

//	LCD_Rect(645, 70, 20, 402, 1, CYAN);
//	
//		for (uint8_t i = 0; i < 41; i++) {
//		uint16_t n = (200 - i * 5) >> 1;
//		if ((uint8_t)n % 10 == 0) {
//			char numbers[3];
//			sprintf(numbers, "%02d", (uint8_t)n);
//			if (n < 100) LCD_Font(610, 77 + i * 10, numbers, &DejaVu_Sans_18, 1, GREEN);
//			else LCD_Font(601, 77 + i * 10, numbers, &DejaVu_Sans_18, 1, GREEN);

//			LCD_Line(635, 71 + i * 10, 645, 71 + i * 10, 1, CYAN);

//		}
//		else if ((uint8_t)n % 5 == 0 && i < 40)
//			LCD_Line(640, 71 + i * 10, 645, 71 + i * 10, 1, CYAN);
//	}

//	LCD_Font(610, 477, "0", &DejaVu_Sans_18, 1, BLACK);

//	for (uint8_t i = 0; i < 41; i++) {
//		uint16_t n = (200 - i * 5) >> 1;
//		if ((uint8_t)n % 10 == 0) {
//			char numbers[3];
//			sprintf(numbers, "%02d", (uint8_t)n);
//			if (n < 100) LCD_Font(410, 77 + i * 10, numbers, &DejaVu_Sans_18, 1, GREEN);
//			else LCD_Font(401, 77 + i * 10, numbers, &DejaVu_Sans_18, 1, GREEN);
//			LCD_Line(435, 71 + i * 10, 445, 71 + i * 10, 1, CYAN);
//		}
//		else if ((uint8_t)n % 5 == 0 && i < 40)
//			LCD_Line(440, 71 + i * 10, 445, 71 + i * 10, 1, CYAN);
//	}

//	LCD_Font(410, 477, "0", &DejaVu_Sans_18, 1, BLACK);

//	LCD_Rect(545, 70, 20, 402, 1, CYAN);

//	for (int32_t i = 0; i < 81; i++) {
//		char numbers[3];
//		sprintf(numbers, "%02d", 40 - i);
//		if (i % 10 == 0) {
//			if (40 - i >= 0) LCD_Font(510, 77 + i * 5, numbers, &DejaVu_Sans_18, 1, GREEN);
//			else if (40 - i < 0) LCD_Font(501, 77 + i * 5, numbers, &DejaVu_Sans_18, 1, GREEN);
//			LCD_Line(535, 71 + i * 5, 545, 71 + i * 5, 1, CYAN);
//		}
//		else if (i % 5 == 0) LCD_Line(540, 71 + i * 5, 545, 71 + i * 5, 1, CYAN);
//		LCD_Font(510, 277, "0", &DejaVu_Sans_18, 1, BLACK);
//	}

//	LCD_Line(500, 2, 500, 478, 1, BLUE);
//	LCD_Line(600, 2, 600, 478, 1, BLUE);
//	LCD_Line(700, 2, 700, 478, 1, BLUE);

	LCD_Rect_Fill(0, 0, 320, 240, BLUE);
	LCD_Rect_Fill(1, 1, 318, 238, BLACK);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
	
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET) {

		touchX = getX();	
		touchY = getY();
		if (touchX && touchY && touchX != 0x0DB)
		{
		LCD_Rect_Fill(touchX, touchY, 1, 1, WHITE);
		}
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
		touchX = 0;
		touchY = 0;
		touchIRQ = 0;
		} else 
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

		if (sound)
		{
//			signal();
//			sound = 0;
		}

		if (rx_index != 0) {
//			HAL_Delay(200);
//			rx_index = 0;
//			uartDecode();
		}

//		if ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET)) {
//			uint16_t touchXr = getX();
//			uint16_t touchYr = getY();

//			if (touchXr && touchYr && touchXr != 0x2F5 && touchYr != 0x0DB) {
//				touchX = touchXr;
//				touchY = touchYr;

//				LCD_Rect_Fill(touchX, touchY, 2, 2, WHITE);

//				if (touchX > 0 && touchX < 32 && touchY > 170 && touchY < 200)
//				{
//					LCD_Rect_Round(2, 170, 30, 30, 2, 1, RED);
//					LCD_Font(8, 179, "_", &DejaVu_Sans_36, 1, RED);
//					if (rtcHrsA1 > 0) rtcHrsA1 = rtcHrsA1 - 1; else rtcHrsA1 = 23;
//					DS3231_setAlarm1Hour(rtcHrsA1);
//					printAlarm = 0;
//					sound = 1;
//				}
//				else if (touchX > 32 && touchX < 64 && touchY > 170 && touchY < 200)
//				{
//					LCD_Rect_Round(34, 170, 30, 30, 2, 1, RED);
//					LCD_Font(34, 197, "+", &DejaVu_Sans_36, 1, RED);
//					if (rtcHrsA1 < 23) rtcHrsA1 = rtcHrsA1 + 1; else rtcHrsA1 = 0;
//					DS3231_setAlarm1Hour(rtcHrsA1);
//					printAlarm = 0;
//					sound = 1;
//				}
//				else if (touchX > 182 && touchX < 212 && touchY > 170 && touchY < 200)
//				{
//					if (rtcMinA1 > 0) rtcMinA1 = rtcMinA1 - 5; else rtcMinA1 = 55;
//					DS3231_setAlarm1Min(rtcMinA1);
//					LCD_Rect_Round(182, 170, 30, 30, 2, 1, RED);
//					LCD_Font(188, 179, "_", &DejaVu_Sans_36, 1, RED);
//					printAlarm = 0;
//					sound = 1;
//				}
//				else if (touchX > 214 && touchX < 244 && touchY > 170 && touchY < 200)
//				{
//					if (rtcMinA1 < 55) rtcMinA1 = rtcMinA1 + 5; else rtcMinA1 = 0;
//					DS3231_setAlarm1Min(rtcMinA1);
//					LCD_Rect_Round(214, 170, 30, 30, 2, 1, RED);
//					LCD_Font(214, 197, "+", &DejaVu_Sans_36, 1, RED);
//					printAlarm = 0;
//					sound = 1;
//				}
//				else if (touchX > 260 && touchX < 390 && touchY > 170 && touchY < 200)
//				{
//					if (AT24XX_Read(4000)) AT24XX_Update(4000, 0);
//					else AT24XX_Update(4000, 1);
//					printAlarm = 0;
//					sound = 1;
//				}

//			}
//		}


		DS3231_Update();
		rtcSec = DS3231_getSec();
		rtcMin = DS3231_getMin();
		rtcHrs = DS3231_getHrs();
		rtcDay = DS3231_getWeekDay();
		rtcDate = DS3231_getDate();
		rtcMonth = DS3231_getMonth();
		rtcYear = DS3231_getYear();
		//		rtcSecA1 = DS3231_getAlarm1Sec();
		rtcMinA1 = DS3231_getAlarm1Min();
		rtcHrsA1 = DS3231_getAlarm1Hour();
		//		rtcDayA1 = DS3231_getAlarm1Day();
		//		rtcDateA1 = DS3231_getAlarm1Date();
		//		rtcMinA2 = DS3231_getAlarm2Min();
		//		rtcHrsA2 = DS3231_getAlarm2Hour();
		//		rtcDayA2 = DS3231_getAlarm2Day();
		//		rtcDateA2 = DS3231_getAlarm2Date();

		char clockPrint[13];

		if (rtcSecLast != rtcSec) { 		
				
	//		bme280();

			static const uint32_t hues[24] = {
				HUE_01, HUE_02, HUE_03, HUE_04, HUE_05, HUE_06, HUE_07, HUE_08, HUE_09, HUE_10, HUE_11, HUE_12,
			HUE_13, HUE_14, HUE_15, HUE_16, HUE_17, HUE_18, HUE_19, HUE_20, HUE_21, HUE_22, HUE_23, HUE_24 };

			LCD_Circle(160, 25, 8, 0, 1, hues[rtcHrs]);
			LCD_Circle(160, 65, 8, 0, 1, hues[rtcHrs]);

			if (rtcSec % 2 != 0) {
				LCD_Circle(160, 25, 7, 1, 1, hues[rtcHrs]);
				LCD_Circle(160, 65, 7, 1, 1, hues[rtcHrs]);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);	
			}
			else {
				LCD_Circle(160, 25, 7, 1, 1, BLACK);
				LCD_Circle(160, 65, 7, 1, 1, BLACK);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
			}

			if (rtcMinLast != rtcMin) {

				sprintf(clockPrint, "%02d", rtcMinLast);
				LCD_Font(180, 85, clockPrint, &DejaVu_Sans_112, 1, BLACK);
				sprintf(clockPrint, "%02d", rtcMin);
				LCD_Font(180, 85, clockPrint, &DejaVu_Sans_112, 1, hues[rtcHrs]);

				if (rtcHrsLast != rtcHrs) {
					sprintf(clockPrint, "%02d", rtcHrsLast);
					LCD_Font(0, 85, clockPrint, &DejaVu_Sans_112, 1, BLACK);
					sprintf(clockPrint, "%02d", rtcHrs);
					LCD_Font(0, 85, clockPrint, &DejaVu_Sans_112, 1, hues[rtcHrs]);

/*					if (rtcDayLast != rtcDay) {

						static const char* days[7] = { "MONDAY", "TUESDAY", "WEDNESDAY",
							"THURSTDAY", "FRIDAY", "SATURDAY", "SUNDAY" };

						LCD_Font(2, 140, days[(7 + rtcDay - 2) % 7], &DejaVu_Sans_36, 1, BLACK);
						LCD_Font(2, 140, days[(7 + rtcDay - 1) % 7], &DejaVu_Sans_36, 1, GRAY);

						static const char* months[12] = { "JANUARY", "FEBRUARY", "MARCH", "APRIL", "MAY",
						"JUNE", "JULE", "AUGUST", "SEPTEMBER", "OCTOBER", "NOVEMBER", "DECEMBER" };

						LCD_Font(3, 168, months[(12 + rtcMonth - 2) % 12], &DejaVu_Sans_36, 1, BLACK);
						LCD_Font(3, 168, months[(12 + rtcMonth - 1) % 12], &DejaVu_Sans_36, 1, GRAY);

						sprintf(clockPrint, "%02d-%02d-%02d", rtcDateLast, rtcMonthLast, rtcYearLast);
						LCD_Font(230, 168, clockPrint, &DejaVu_Sans_36, 1, BLACK);

						sprintf(clockPrint, "%02d-%02d-%02d", rtcDate, rtcMonth, rtcYear);
						LCD_Font(230, 168, clockPrint, &DejaVu_Sans_36, 1, GRAY);

//						if (rtcMoonLast < 10) sprintf(clockPrint, "M:  %01d", (uint16_t)rtcMoonLast);
//						else sprintf(clockPrint, "M: %02d", (uint16_t)rtcMoonLast);
//						LCD_Font(297, 140, clockPrint, &DejaVu_Sans_36, 1, BLACK);
//						rtcMoon = DS3231_getMoonDay();
//						if (rtcMoon < 10) sprintf(clockPrint, "M:  %01d", (uint16_t)rtcMoon);
//						else sprintf(clockPrint, "M: %02d", (uint16_t)rtcMoon);
//						LCD_Font(297, 140, clockPrint, &DejaVu_Sans_36, 1, GRAY);
//						rtcMoonLast = rtcMoon;

						rtcDayLast = rtcDay;
					} */
					rtcDateLast = rtcDate;
					rtcMonthLast = rtcMonth;
					rtcYearLast = rtcYear;
					rtcHrsLast = rtcHrs;
					sound = 1;
				}


				rtcMinLast = rtcMin;
			}
//			alarm();
			
		rtcSecLast = rtcSec;
		}					
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_1_Pin|LED_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_1_Pin LED_2_Pin */
  GPIO_InitStruct.Pin = LED_1_Pin|LED_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : TOUCH_IRQ_Pin */
  GPIO_InitStruct.Pin = TOUCH_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TOUCH_IRQ_GPIO_Port, &GPIO_InitStruct);

}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{

  /* USER CODE BEGIN FSMC_Init 0 */

  /* USER CODE END FSMC_Init 0 */

  FSMC_NORSRAM_TimingTypeDef Timing = {0};
  FSMC_NORSRAM_TimingTypeDef ExtTiming = {0};

  /* USER CODE BEGIN FSMC_Init 1 */

  /* USER CODE END FSMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FSMC_NORSRAM_DEVICE;
  hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_ENABLE;
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  /* Timing */
  Timing.AddressSetupTime = 0;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 1;
  Timing.BusTurnAroundDuration = 0;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */
  ExtTiming.AddressSetupTime = 0;
  ExtTiming.AddressHoldTime = 15;
  ExtTiming.DataSetupTime = 2;
  ExtTiming.BusTurnAroundDuration = 0;
  ExtTiming.CLKDivision = 16;
  ExtTiming.DataLatency = 17;
  ExtTiming.AccessMode = FSMC_ACCESS_MODE_A;

  if (HAL_SRAM_Init(&hsram1, &Timing, &ExtTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /** Disconnect NADV
  */

  __HAL_AFIO_FSMCNADV_DISCONNECTED();

  /* USER CODE BEGIN FSMC_Init 2 */

  /* USER CODE END FSMC_Init 2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
		/* User can add his own implementation to report the HAL error return state */

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
		   tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

