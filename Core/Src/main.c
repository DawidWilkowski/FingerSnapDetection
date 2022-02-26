/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2021 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void errorKomenda();
void errorKodowanie();
void wyslijTrybPracyJeden();
void wyslijTrybPracyDwa();
void wyslijGranica();
void analyseStart(int);
void wyslijSetGranica();
int wystepujeKorelacja();
double* korelacjaWzajemna();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// =============== ZMIENNE ===============
#define RX_BUFF_SIZE 512
#define TX_BUFF_SIZE 512
#define DATA_BUFF_SIZE 2048
#define ilosc_probek 150

__IO char BUFF_RX[RX_BUFF_SIZE];
__IO char BUFF_TX[TX_BUFF_SIZE];
__IO uint16_t BUFF_DATA[DATA_BUFF_SIZE];

__IO uint16_t EmptyRx = 0;
__IO uint16_t BusyRx = 0;
__IO uint16_t EmptyTx = 0;
__IO uint16_t BusyTx = 0;

__IO uint16_t granica = 3300;
__IO uint16_t dataIndex = 0; // 0 - polowa danych, 1023 - druga polowa danych
__IO uint8_t trybPracy = 1;
__IO uint8_t dataReady = 0;
__IO uint16_t count = 0;
__IO char temp;

uint8_t buforRamki[RX_BUFF_SIZE];
uint8_t stan = 0;

char device_address[3] = "STM";
char source_address[3]="DAW";
char destination_address[4];
uint16_t dlugoscRamki = 0;
int cmdLength;
uint8_t znak;
char wiadomosc[RX_BUFF_SIZE];

//======= ZMIENNE - suma kontrolna ===========
char suma_kontrolna[5];
char odebranaGranica[5];
int sumaNadOdb = 464;
uint16_t suma_wyliczona = 0;
int podana_suma_kontrolna=0;

//======= ZMIENNE - korelacja ===========
uint8_t przekroczonoGranice = 0;//
double wspolczynnik = 0;
uint8_t iloscPstryk= 0; //liczy pstrykniecia 
int pstrykniecie[150];

int wzor_pstrykniecia[150] = { 2046,2061,2080,2124,2118,2131,2140,2147,2140,
2150,2162,2171,2188,2191,2201,2225,2250,2232,2254,2265,2278,2283,2312,
2302,2285,2294,2287,2320,2277,2286,2296,2258,2346,2268,2271,2249,2278,
2222,2319,2198,2236,2127,2185,2148,2183,2058,1965,1796,1661,891,4007,
1852,2591,4007,4007,0,0,4004,4004,0,4005,0,216,1368,4007,4005,4007,0,
2282,0,3995,4006,4007,4007,181,0,0,0,4009,4007,4007,3241,0,0,0,4009,
4007,3630,2856,0,0,1695,1119,3842,3209,2563,1396,296,448,1534,2423,3500,
2128,1762,1251,171,1234,1584,2800,2646,2401,1686,1188,1088,1453,2238,2150,
2677,2138,1239,1311,1153,1914,2395,2441,2224,1639,911,1659,1480,2113,2445,
2213,1749,1658,1530,1850,1738,2399,1937,1886,1531,1690,1757,2151,2111,2134,
1732,1752,1967};



// =============== ZMIENNE ===============
// ====================== USART SEND ======================

void send(char *format, ...) {

	char tmp_s[512];
	va_list arglist;
	va_start(arglist, format);
	vsprintf(tmp_s, format, arglist);
	va_end(arglist);

	uint16_t idx = EmptyTx;
	for (int i = 0; i < strlen(tmp_s); i++) {
		BUFF_TX[idx] = tmp_s[i];
		idx++;
		if (idx >= TX_BUFF_SIZE)
			idx = 0;
	}
	__disable_irq();
	if (BusyTx == EmptyTx 
	&& __HAL_UART_GET_FLAG(&huart2,UART_FLAG_TXE) == SET) {
		EmptyTx = idx;
		temp = BUFF_TX[BusyTx];
		BusyTx++;
		if (BusyTx >= TX_BUFF_SIZE)
			BusyTx = 0;
		HAL_UART_Transmit_IT(&huart2, &temp, 1);
	} else {
		EmptyTx = idx;
	}
	__enable_irq();
}

// ====================== USART SEND ======================
// ====================== CALLBACK ======================

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart2) {
		if (BusyTx != EmptyTx) {
			temp = BUFF_TX[BusyTx];
			BusyTx++;
			if (BusyTx >= TX_BUFF_SIZE)
				BusyTx = 0;
			HAL_UART_Transmit_IT(&huart2, &temp, 1);
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart2) {
		EmptyRx++;
		if (EmptyRx >= RX_BUFF_SIZE)
			EmptyRx = 0;
	}
	HAL_UART_Receive_IT(&huart2, &BUFF_RX[EmptyRx], 1);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc1) {
	dataReady = 1;
	dataIndex = 0;
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc1) {
	dataReady = 1;
	dataIndex = 1023;
}

// ====================== CALLBACK ======================

//========== ANALIZA KOMENDY ============
	void analizaKomendy(char cmd[]) {

		if (!strcmp(cmd, "Tryb1")) {
			trybPracy = 1;
		} else if (!strcmp(cmd, "Tryb2")) {
			trybPracy = 2;
		} else if (!strcmp(cmd, "PokazTryb")) {

			if (trybPracy == 1)
			wyslijTrybPracyJeden();
			else
			wyslijTrybPracyDwa();


		}
		else if(!strcmp(cmd, "GRANICA")){
			wyslijGranica();
		}
		else if(!strncmp(cmd, "SETGRANICA", 9)) {
					memcpy(odebranaGranica, &buforRamki[20], 4);
					odebranaGranica[4]= '\n';
					granica = atoi(odebranaGranica);
					if(granica >2000&&granica <4000){
					wyslijGranica();
					}
					else {
						granica = 3300;

					}

				}
		else {
			errorKomenda();

		}
	}
//========== ANALIZA KOMENDY ============
//========= DEKODOWANIE ============
	void dekoduj() {
		if (znak == '1') {
			buforRamki[dlugoscRamki] = '[';
			dlugoscRamki++;
			stan = 1;
		} else if (znak == '2') {
			buforRamki[dlugoscRamki] = ']';
			dlugoscRamki++;
			stan = 1;
		} else if (znak == '3') {
			buforRamki[dlugoscRamki] = '!';
			dlugoscRamki++;
			stan = 1;
		} else {
			stan = 0;
			errorKodowanie();
		}

	}
//========= DEKODOWANIE ============
//========= ERROR ========
	void errorSumaKontrolna() {
		sprintf(wiadomosc, "[%s%s%dNieprawidlowaSumaKontrolna]\r\n",device_address,source_address,sumaNadOdb+2724);
		send(wiadomosc);
	}
	void errorKomenda(){
		sprintf(wiadomosc,"[%s%s%dNiepoprawnaKomenda]\r\n",device_address,source_address,sumaNadOdb+1859);
		send(wiadomosc);
	}
	void errorKodowanie(){
		sprintf(wiadomosc,"[%s%s%dBladKodowania]\r\n",device_address,source_address,sumaNadOdb+1296);
		send(wiadomosc);
	}
//========== ERROR =======
//========== INFORMACJE ZWROTNE ========
	void start(){
		sprintf(wiadomosc,"[%s%s%dRozpoczynamPrace!]\r\n",device_address,source_address,sumaNadOdb+1720);
		send(wiadomosc);
	}
	void wyslijPstrykJeden(){
		sprintf(wiadomosc,"[%s%s%dPSTRYK1]\r\n",device_address,source_address,sumaNadOdb+542);
		send(wiadomosc);
	}
	void wyslijPstrykDwa(){
		sprintf(wiadomosc,"[%s%s%dPSTRYK2]\r\n",device_address,source_address,sumaNadOdb+543);
		send(wiadomosc);
	}
	void odebranoRamke(){
		sprintf(wiadomosc,"[%s%s%dOdebranoRamke]\r\n",device_address,source_address,sumaNadOdb+1306);
		send(wiadomosc);
	}
	void wyslijTrybPracyJeden(){
		sprintf(wiadomosc,"[%s%s%dTryb1PSTRYK]\r\n",device_address,source_address,sumaNadOdb+959);
		send(wiadomosc);
	}
	void wyslijTrybPracyDwa(){
		sprintf(wiadomosc,"[%s%s%dTryb2PSTRYK]\r\n",device_address,source_address,sumaNadOdb+960);
		send(wiadomosc);
	}
	void wyslijGranica(){
	int granicaTablica[4];
	int granicaKopia = granica;
	int sumaGranicaTablica = 0;
	for (int j = 3; j>=0; j--){//suma kontrolna z wpisanej granicy
	granicaTablica[j] = granicaKopia % 10;// reszta z dzielenia przez 10 => przepisanie int do tablicy
	sumaGranicaTablica += granicaTablica[j] + 48;//do ascii
	granicaKopia/=10;
	}
	sprintf(wiadomosc,"[%s%s%dGRANICA%d]\r\n",device_address,source_address,sumaNadOdb+sumaGranicaTablica+501,granica);
	send(wiadomosc);
	}
//========== INFORMACJE ZWROTNE ========
//========== BADANIE PSTRYKNIECIA - Korelacja======
	void analyseData() {
			if (dataIndex == 0) {
				for (int i = 0; i < 1023; i++) {
					analyseStart(i);
				}
			} else if (dataIndex == 1023) {
				for (int i = 1023; i < 2048; i++) {
					analyseStart(i);
				}
			}
		}
	void analyseStart(int i) {
		if(przekroczonoGranice==0){
		if(BUFF_DATA[i]>granica){
			przekroczonoGranice=1;
			for(int j =i-50;j<i+100;j++){
				if(j<0) j = 2048+j;
				if(j>2048) j =0;
				pstrykniecie[count]=BUFF_DATA[j];
				count++;
				}
			if (wystepujeKorelacja()) {
				przekroczonoGranice=0;
				if(trybPracy==1){wyslijPstrykJeden();}
				else if (trybPracy==2){
					iloscPstryk++;
					if (iloscPstryk == 2 )
					{iloscPstryk = 0;
					wyslijPstrykDwa();}
					}
				}

				else {
					przekroczonoGranice=0;
					iloscPstryk = 0;
				}
			count=0;
	}}}
	int wystepujeKorelacja() {
		double* wspolczynnik = korelacjaWzajemna();

			for (int k = 0; k <=150*2; k++) {
				if (fabs(wspolczynnik[k]) > 0.50) {
					return 1;
				}
			}

			return 0;
	}

	double* korelacjaWzajemna() {
		double suma_x = 0.0;
		double suma_y = 0.0;
		double srednia_x = 0.0;
		double srednia_y = 0.0;
		double wyr_1, wyr_2, licznik;
		double kw_szereg_x = 0.0;
		double kw_szereg_y = 0.0;
		static double korelacja[20];
		int opoznienie, i;
		int opoznienie_max = 10;
		//liczenie sumy
		int z;
		for (z = 0; z < ilosc_probek; z++) {
			suma_x += wzor_pstrykniecia[z];
			suma_y += pstrykniecie[z];
		}
		//srednia
		srednia_x = suma_x / ilosc_probek;
		srednia_y = suma_y / ilosc_probek;
		//korelacja
		for (opoznienie = -opoznienie_max; opoznienie <= opoznienie_max; opoznienie++) {
			//od -10 do 10
			licznik = 0;
			kw_szereg_x = 0;
			kw_szereg_y = 0;

			for (i = 0; i < ilosc_probek; i++) {

				int j = i - opoznienie;
				if (j < 0 || j >= ilosc_probek) {
					wyr_1 = (wzor_pstrykniecia[i] - srednia_x);
					wyr_2 = (srednia_y-srednia_y);//zakładamy że wartosci spoza zakresu przyjmuja wartosc srednia
					licznik += wyr_1 * wyr_2;
					kw_szereg_x += wyr_1 * wyr_1;
					kw_szereg_y += wyr_2 * wyr_2;
				}
				else {
					wyr_1 = (wzor_pstrykniecia[i] - srednia_x);
					wyr_2 = (pstrykniecie[j] - srednia_y);
					licznik += wyr_1 * wyr_2;
					kw_szereg_x += wyr_1 * wyr_1;
					kw_szereg_y += wyr_2 * wyr_2;
				}

			}

			korelacja[opoznienie + opoznienie_max] = licznik / sqrt(kw_szereg_x * kw_szereg_y);
		}

		return korelacja;
	}
//========== BADANIE PSTRYKNIECIA - Korelacja======



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
  MX_USART2_UART_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*) BUFF_DATA, 2048);
		HAL_UART_Receive_IT(&huart2, &BUFF_RX[EmptyRx], 1);
		start();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
		 while (1) {
			 if (dataReady) {
			 				dataReady = 0;
			 				analyseData();
			 			}

			if (BusyRx != EmptyRx) {
				znak = BUFF_RX[BusyRx];//Przepisanie z bufora
				BusyRx++;
				if (BusyRx >= RX_BUFF_SIZE) BusyRx = 0;
				if (znak == '[')
					{
					stan = 1;
					dlugoscRamki = 0;
					}
				else if (stan == 1) // stan 1 - odnaleziono początek ramki
					{

					if (znak == '!') stan = 2; //stan 2 - znak wymaga odkodowania
					 else if (znak == ']')
					{
						buforRamki[dlugoscRamki] = '\0';

						if (dlugoscRamki >= 10) // podano więcej znaków, niż minimalna ilość znaków w ramce
							{

							memcpy(source_address, &buforRamki[0], 3);
							memcpy(destination_address, &buforRamki[3], 3);
							memcpy(suma_kontrolna, &buforRamki[6], 4);

							source_address[3] = '\0';
							destination_address[3] = '\0';
							suma_kontrolna[4] = '\0';

							podana_suma_kontrolna = atoi(suma_kontrolna);// jak bedzie jakas bledna wartosc podana to nie przejdzie

							cmdLength = dlugoscRamki - 10;
							if (strncmp(device_address, destination_address, 3) == 0) //jesli poprawny odbiorca
									{
									char cmd[cmdLength + 1];
									memcpy(cmd, &buforRamki[10], cmdLength);
									cmd[cmdLength] = '\0';

									for (int i = 0; i < 3; i++) suma_wyliczona +=(int) source_address[i];
									for (int i = 0; i < 3; i++) suma_wyliczona +=(int) destination_address[i];
									sumaNadOdb = suma_wyliczona;// suma z nadawcy + odbiorcy
									for (int i = 0; i <= cmdLength; i++) suma_wyliczona += (int) cmd[i];

									if (podana_suma_kontrolna == suma_wyliczona) // jeżeli komenda została przysłana
											{
										odebranoRamke();
										analizaKomendy(cmd);
									} else {
										 errorSumaKontrolna();
									}

									suma_wyliczona = 0;

							}
						}

						stan = 0;
						dlugoscRamki = 0;
					}
					 else
					{
						buforRamki[dlugoscRamki] = znak; // zapisuje znak do bufora ramki
						dlugoscRamki++;
						if (dlugoscRamki > (62)) {
							dlugoscRamki = 0;
							stan = 0; //stan - 0 - stan poszukiwania nowego znaku rozpoczecia ramki '['
							break;
						}
					}
				} else if (stan == 2)
				{
					dekoduj();
				}
			}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		}
}
  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
		__disable_irq();
		while (1) {
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

