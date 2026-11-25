/* Standard includes. */
#include <stdio.h>
#include <conio.h>
#include <string.h>
#include <stdlib.h>
/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "extint.h"

/* Hardware simulator utility functions */
#include "HW_access.h"

//---------------------------------SERIAL SIMULATOR CHANNEL-----------------------------------------
#define COM_CH0 (0)
#define COM_CH1 (1)
#define COM_CH2 (2)

//-------------------------------------TASK PRIORITIES----------------------------------------------
#define	TASK_SERIAL_SEND_PRI		( tskIDLE_PRIORITY + (UBaseType_t)3 )
#define TASK_SERIAL_REC_PRI			( tskIDLE_PRIORITY + (UBaseType_t)4 )
#define TASK_SENSOR_SERIAL_REC_PRI	( tskIDLE_PRIORITY + (UBaseType_t)5 )
#define OBRADA_TASK_PRI				( tskIDLE_PRIORITY + (UBaseType_t)1 )
#define	SERVICE_TASK_PRI			( tskIDLE_PRIORITY + (UBaseType_t)2 )

//--------------------------------TASKS: FORWARD DECLARATIONS---------------------------------------
void SerialSend_Task(void* pvParameters);
void SerialSend_Task0(void* pvParameters);
void SerialSend_Task1(void* pvParameters);
void SerialReceive_Task(void* pvParameters);
void Obrada_task(void* pvParameters);
void LED_bar_task(void* pvParameters);
void senzor0_task(void* pvParameters);
void senzor1_task(void* pvParameters);
void Seg7_task(void* pvParameters);
void Led_task(void* pvParameters);
void task_obrada_kalibracija(void* pvParameters);
void main_demo(void);
int32_t izracunaj_procenat(int32_t senzor_mm, int32_t udaljenost_min, int32_t udaljenost_max);

//-------------------------------------SEMAPHORE HANDLES--------------------------------------------
static SemaphoreHandle_t TBE_BinarySemaphore_0;
static SemaphoreHandle_t TBE_BinarySemaphore_1;
static SemaphoreHandle_t TBE_BinarySemaphore_2;
static SemaphoreHandle_t RXC_BinarySemaphore_0;
static SemaphoreHandle_t RXC_BinarySemaphore_1;
static SemaphoreHandle_t RXC_BinarySemaphore_2;
static SemaphoreHandle_t LED_INT_BinarySemaphore;
static SemaphoreHandle_t obrada;
static SemaphoreHandle_t seg7;
static SemaphoreHandle_t senzor_0;
static SemaphoreHandle_t senzor_1;

//------------------------------------ QUEUE HANDLES------------------------------------------------
static QueueHandle_t kanal_2_ispis_rijeci = NULL;
static QueueHandle_t kanal_2_ispis_duzine = NULL;
static QueueHandle_t komandna_rijec_niz = NULL;
static QueueHandle_t komandna_rijec_duzina = NULL;
static QueueHandle_t kalibracija = NULL;
static QueueHandle_t vrednost1_max = NULL;
static QueueHandle_t vrednost1_min = NULL;
static QueueHandle_t vrednost2_max = NULL;
static QueueHandle_t vrednost2_min = NULL;
static QueueHandle_t procenat = NULL;
static QueueHandle_t prekidac = NULL;
static QueueHandle_t stanje = NULL;
static QueueHandle_t senzor0_seg7 = NULL;
static QueueHandle_t senzor1_seg7 = NULL;
static QueueHandle_t senzor0_obrada = NULL;
static QueueHandle_t senzor1_obrada = NULL;

//--------------------------------------TIMER FUNKCIJA----------------------------------------------
static void seg7_tajmer_callback(TimerHandle_t Tmh); //svakih 80ms ispisuje stanje sistema			
static void tajmer0_callback(TimerHandle_t Tmh); //svakih 200ms ispisuje stanje sistema		        
static void tajmer1_callback(TimerHandle_t Tmh); //svakih 200ms ispisuje stanje sistema		        
static void ispis_tajmer_callback(TimerHandle_t Tmh); //svakih 5000ms ispisuje stanje sistema																							

//---------------------------------------TIMER HANDLES----------------------------------------------																
static TimerHandle_t seg7_tajmer;
static TimerHandle_t tajmer0;
static TimerHandle_t tajmer1;
static TimerHandle_t ispis_tajmer;

//----------------------------------PERIODIC TIMER CALLBACK-----------------------------------------
static void seg7_tajmer_callback(TimerHandle_t Tmh) {
	(void)Tmh;
	xSemaphoreGive(seg7); // predaj semafor svakih 80ms											
}
static void tajmer0_callback(TimerHandle_t Tmh) {
	(void)Tmh;
	xSemaphoreGive(senzor_0); // predaj semafor svakih 200ms										
}
static void tajmer1_callback(TimerHandle_t Tmh) {
	(void)Tmh;
	xSemaphoreGive(senzor_1); // predaj semafor svakih 200ms										
}
static void ispis_tajmer_callback(TimerHandle_t Tmh) {
	(void)Tmh;
	xSemaphoreGive(obrada); // predaj semafor svakih 5000ms											
}

//--------------------------------------------------------------------------------------------------
typedef unsigned int t_point_t;
typedef unsigned int t_point0_t;
typedef unsigned int t_point1_t;
typedef float procenat_t;
typedef int senzor_mm_t;
static volatile uint8_t ukljucen = 0;

//-------------------------------------TRASNMISSION DATA--------------------------------------------
static const char trigger0[] = "S0";
static const char trigger1[] = "S1";
static volatile t_point_t t_point;
static volatile t_point0_t t_point0;
static volatile t_point1_t t_point1;

//---------------------------------------RECEPTION DATA---------------------------------------------
#define R_BUF_SIZE (25)
#define R_BUF_SIZE_tasks (2)
static volatile uint32_t r_point;
static volatile uint32_t r_point0 = 0;
static volatile uint32_t r_point1 = 0;

//--------------------------7-SEG NUMBER DATABASE - ALL HEX DIGITS----------------------------------
static const uint8_t hexnum[] = { 0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07,
								0x7F, 0x6F, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71 };

//----------------------------------------INTERUPT--------------------------------------------------
/* TBE - TRANSMISSION BUFFER EMPTY - INTERRUPT HANDLER */
static uint32_t prvProcessTBEInterrupt(void)
{
	BaseType_t xHigherPTW = pdFALSE;

	if (get_TBE_status(0) != 0)
	{
		xSemaphoreGiveFromISR((SemaphoreHandle_t)TBE_BinarySemaphore_0, &xHigherPTW);
	}
	if (get_TBE_status(1) != 0)
	{
		xSemaphoreGiveFromISR((SemaphoreHandle_t)TBE_BinarySemaphore_1, &xHigherPTW);
	}
	if (get_TBE_status(2) != 0)
	{
		xSemaphoreGiveFromISR((SemaphoreHandle_t)TBE_BinarySemaphore_2, &xHigherPTW);
	}
	portYIELD_FROM_ISR(xHigherPTW);
}

/* RXC - RECEPTION COMPLETE - INTERRUPT HANDLER */
static uint32_t prvProcessRXCInterrupt(void)
{
	BaseType_t xHigherPTW = pdFALSE;

	if (get_RXC_status(0) != 0)
	{
		xSemaphoreGiveFromISR((SemaphoreHandle_t)RXC_BinarySemaphore_0, &xHigherPTW);
	}
	if (get_RXC_status(1) != 0)
	{
		xSemaphoreGiveFromISR((SemaphoreHandle_t)RXC_BinarySemaphore_1, &xHigherPTW);
	}
	if (get_RXC_status(2) != 0)
	{
		xSemaphoreGiveFromISR((SemaphoreHandle_t)RXC_BinarySemaphore_2, &xHigherPTW);
	}
	portYIELD_FROM_ISR(xHigherPTW);
}

/* OPC - ON INPUT CHANGE - INTERRUPT HANDLER */
static uint32_t OnLED_ChangeInterrupt(void)
{
	BaseType_t xHigherPTW = pdFALSE;
	BaseType_t xResult;

	xResult = xSemaphoreGiveFromISR((SemaphoreHandle_t)LED_INT_BinarySemaphore, &xHigherPTW);
	(void)xResult;

	portYIELD_FROM_ISR(xHigherPTW);
}

//-------------------------------------MAIN - SYSTEM STARTUP POINT--------------------------------------
void main_demo(void)
{
	//--------------------------------------INICIJALIZACIJA---------------------------------------------
	(void)init_serial_uplink(COM_CH0);  // inicijalizacija serijske TX na kanalu 0
	(void)init_serial_downlink(COM_CH0);// inicijalizacija serijske TX na kanalu 0
	(void)init_serial_uplink(COM_CH1);  // inicijalizacija serijske TX na kanalu 1
	(void)init_serial_downlink(COM_CH1);// inicijalizacija serijske TX na kanalu 1
	(void)init_serial_uplink(COM_CH2);  // inicijalizacija serijske TX na kanalu 2
	(void)init_serial_downlink(COM_CH2);// inicijalizacija serijske TX na kanalu 2
	(void)init_LED_comm();
	(void)init_7seg_comm();
	(void)select_7seg_digit(4);
	(void)set_7seg_digit(0x40);

	//------------------------------------CREATE A TIMER TASK-------------------------------------------
	seg7_tajmer = (TimerHandle_t)xTimerCreate("Timer", pdMS_TO_TICKS(250), pdTRUE, NULL, seg7_tajmer_callback); //callback 250ms
	(void)xTimerStart(seg7_tajmer, 0);

	tajmer0 = (TimerHandle_t)xTimerCreate("Timer1", pdMS_TO_TICKS(200), pdTRUE, NULL, tajmer0_callback); //callback 200ms
	(void)xTimerStart(tajmer0, 0);

	tajmer1 = (TimerHandle_t)xTimerCreate("Timer2", pdMS_TO_TICKS(200), pdTRUE, NULL, tajmer1_callback); //callback 200ms
	(void)xTimerStart(tajmer1, 0);

	ispis_tajmer = (TimerHandle_t)xTimerCreate("Timer3", pdMS_TO_TICKS(5000), pdTRUE, NULL, ispis_tajmer_callback); //callback 5000ms
	(void)xTimerStart(ispis_tajmer, 0);

	//------------------------------------CREATE SEMAPHORE----------------------------------------------
	/* Create TBE semaphore - serial transmit comm */
	TBE_BinarySemaphore_0 = (SemaphoreHandle_t)xSemaphoreCreateBinary();
	TBE_BinarySemaphore_1 = (SemaphoreHandle_t)xSemaphoreCreateBinary();
	TBE_BinarySemaphore_2 = (SemaphoreHandle_t)xSemaphoreCreateBinary();

	/* Create TBE semaphore - serial transmit comm */
	RXC_BinarySemaphore_0 = (SemaphoreHandle_t)xSemaphoreCreateBinary();
	RXC_BinarySemaphore_1 = (SemaphoreHandle_t)xSemaphoreCreateBinary();
	RXC_BinarySemaphore_2 = (SemaphoreHandle_t)xSemaphoreCreateBinary();

	LED_INT_BinarySemaphore = (SemaphoreHandle_t)xSemaphoreCreateBinary();
	obrada = (SemaphoreHandle_t)xSemaphoreCreateBinary();
	seg7 = (SemaphoreHandle_t)xSemaphoreCreateBinary();
	senzor_0 = (SemaphoreHandle_t)xSemaphoreCreateBinary();
	senzor_1 = (SemaphoreHandle_t)xSemaphoreCreateBinary();

	//----------------------------------------INTERUPT--------------------------------------------------
	/* SERIAL TRANSMISSION INTERRUPT HANDLER */
	vPortSetInterruptHandler(portINTERRUPT_SRL_TBE, prvProcessTBEInterrupt);

	/* SERIAL RECEPTION INTERRUPT HANDLER */
	vPortSetInterruptHandler(portINTERRUPT_SRL_RXC, prvProcessRXCInterrupt);

	/* ON INPUT CHANGE INTERRUPT HANDLER */
	vPortSetInterruptHandler(portINTERRUPT_SRL_OIC, OnLED_ChangeInterrupt);

	//---------------------KREIRANJE REDOVA ZA KOMUNIKACIJU IZMEDJU TASKOVA-----------------------------
	kanal_2_ispis_rijeci = xQueueCreate(3, sizeof(uint8_t[120])); //red za skladistenje poruke za ispis
	kanal_2_ispis_duzine = xQueueCreate(3, sizeof(uint8_t)); //red za skladistenje duzine rijeci

	komandna_rijec_niz = xQueueCreate(3, sizeof(uint8_t[25])); //red za skladistenje primljene komande
	komandna_rijec_duzina = xQueueCreate(3, sizeof(uint8_t)); //red za skladistenje duzine primljene komande

	prekidac = xQueueCreate(1, sizeof(uint8_t)); //red za primanje stanja prekidaca
	stanje = xQueueCreate(1, sizeof(uint8_t)); //red za primanje stanja sistema, tj. ukljucen/iskljucen

	kalibracija = xQueueCreate(3, sizeof(uint8_t[25]));
	vrednost1_max = xQueueCreate(1, sizeof(uint8_t[4]));
	vrednost1_min = xQueueCreate(1, sizeof(uint8_t[3]));
	vrednost2_max = xQueueCreate(1, sizeof(uint8_t[4]));
	vrednost2_min = xQueueCreate(1, sizeof(uint8_t[3]));
	procenat = xQueueCreate(5, sizeof(procenat_t));

	senzor0_seg7 = xQueueCreate(5, sizeof(uint8_t[2]));
	senzor1_seg7 = xQueueCreate(5, sizeof(uint8_t[2]));

	senzor0_obrada = xQueueCreate(1, sizeof(uint8_t[2]));
	senzor1_obrada = xQueueCreate(1, sizeof(uint8_t[2]));

	//----------------------------------------CREATE TASK-----------------------------------------------
	// PC RECEIVE TASK
	(void)xTaskCreate(SerialReceive_Task, "SRx", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAL_REC_PRI, NULL);

	// PC SEND TASK
	(void)xTaskCreate(SerialSend_Task, "STx", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAL_SEND_PRI, NULL);

	(void)xTaskCreate(SerialSend_Task0, "STx0", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAL_SEND_PRI, NULL);

	(void)xTaskCreate(SerialSend_Task1, "STx1", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAL_SEND_PRI, NULL);

	// SENZOR_0 TASK
	(void)xTaskCreate(senzor0_task, "Senzor_0x", configMINIMAL_STACK_SIZE, NULL, TASK_SENSOR_SERIAL_REC_PRI, NULL);

	// SENZOR_1 TASK
	(void)xTaskCreate(senzor1_task, "Senzor_1x", configMINIMAL_STACK_SIZE, NULL, TASK_SENSOR_SERIAL_REC_PRI, NULL);

	// OBRADA PODATAKA TASK
	(void)xTaskCreate(Obrada_task, "Obrada_task", configMINIMAL_STACK_SIZE, NULL, OBRADA_TASK_PRI, NULL);
	(void)xTaskCreate(task_obrada_kalibracija, "task za obradu kalibracije", configMINIMAL_STACK_SIZE, NULL, OBRADA_TASK_PRI, NULL);

	// LED BAR TASK
	(void)xTaskCreate(LED_bar_task, "LED_bar_task", configMINIMAL_STACK_SIZE, NULL, SERVICE_TASK_PRI, NULL);

	// SEG7 TASK
	(void)xTaskCreate(Seg7_task, "Seg7_task", configMINIMAL_STACK_SIZE, NULL, SERVICE_TASK_PRI, NULL);

	// LED TASK
	(void)xTaskCreate(Led_task, "LED_task", configMINIMAL_STACK_SIZE, NULL, SERVICE_TASK_PRI, NULL);

	vTaskStartScheduler();

	for (;;)
	{

	}
}

void SerialSend_Task(void* pvParameters)
{
	uint8_t ispis_niza[60] = { 0 };
	uint8_t duzina_ispis_niza = 0;
	BaseType_t xResult = pdFALSE;
	(void)pvParameters;
	for (;;)
	{
		xResult = xSemaphoreTake((SemaphoreHandle_t)TBE_BinarySemaphore_2, portMAX_DELAY);
		(void)xResult;

		xResult = xQueueReceive(kanal_2_ispis_rijeci, ispis_niza, pdMS_TO_TICKS(10));
		(void)xResult;

		xResult = xQueueReceive(kanal_2_ispis_duzine, &duzina_ispis_niza, pdMS_TO_TICKS(10));
		(void)xResult;

		if (t_point < duzina_ispis_niza)
		{
			(void)send_serial_character(COM_CH2, ispis_niza[t_point++]);
		}
		else
		{
			t_point = 0;
			duzina_ispis_niza = 0;
		}
	}
}

void SerialSend_Task0(void* pvParameters) {
	(void)pvParameters;
	t_point0 = 0;
	for (;;)
	{
		xSemaphoreTake(senzor_0, portMAX_DELAY);

		if (ukljucen != 0U)
		{
			if ((uint8_t)t_point0 > (sizeof(trigger0) - 1))
			{
				t_point0 = 0;
			}

			(void)send_serial_character(COM_CH0, (uint8_t)trigger0[t_point0++]);

			BaseType_t xResult = pdFALSE;
			xResult = xSemaphoreTake((SemaphoreHandle_t)TBE_BinarySemaphore_0, portMAX_DELAY); // MISRA 11.2 i 17.7
			(void)xResult;
		}
		else
		{
			vTaskDelay(pdMS_TO_TICKS(100));
		}
	}
}

void SerialSend_Task1(void* pvParameters) {
	(void)pvParameters;
	t_point1 = 0;
	for (;;)
	{
		xSemaphoreTake(senzor_1, portMAX_DELAY);
		if (ukljucen != 0U)
		{
			if ((uint8_t)t_point1 > (sizeof(trigger1) - 1))
			{
				t_point1 = 0;
			}

			(void)send_serial_character(COM_CH1, (uint8_t)trigger1[t_point1++]);

			BaseType_t xResult = pdFALSE;
			xResult = xSemaphoreTake((SemaphoreHandle_t)TBE_BinarySemaphore_1, portMAX_DELAY); // MISRA 11.2 i 17.7
			(void)xResult;
		}
		else
		{
			vTaskDelay(pdMS_TO_TICKS(100));
		}
	}
}

void SerialReceive_Task(void* pvParameters)
{
	(void)pvParameters;
	uint8_t cc = 0;
	uint8_t r_buffer[R_BUF_SIZE] = { 0 };
	uint8_t duzina_primljene_rijeci = 0;
	r_point = 0;
	BaseType_t xResult = pdFALSE;

	for (;;)
	{
		xResult = xSemaphoreTake((SemaphoreHandle_t)RXC_BinarySemaphore_2, portMAX_DELAY);
		(void)xResult;

		xResult = get_serial_character(COM_CH2, &cc);
		(void)xResult;

		if ((uint8_t)cc == 0x0D)
		{
			r_buffer[r_point] = (uint8_t)0;
			duzina_primljene_rijeci = r_point;

			if (r_buffer[0] == (uint8_t)75)
			{
				xResult = xQueueSend(kalibracija, r_buffer, portMAX_DELAY);
				(void)xResult;

				xResult = xQueueSend(komandna_rijec_duzina, &duzina_primljene_rijeci, pdMS_TO_TICKS(10));
				(void)xResult;
			}
			else
			{
				xResult = xQueueSend(komandna_rijec_niz, r_buffer, 0U);
				(void)xResult;

				xResult = xQueueSend(komandna_rijec_duzina, &duzina_primljene_rijeci, 0U);
				(void)xResult;
			}
			r_point = 0;

		}

		else if (r_point < (R_BUF_SIZE - 1))
		{
			r_buffer[r_point++] = cc;
		}
	}
}

int32_t izracunaj_procenat(int32_t senzor_mm, int32_t udaljenost_min, int32_t udaljenost_max)
{
	procenat_t rezultat = 0.0f;

	if (udaljenost_max <= udaljenost_min)
	{
		rezultat = 0.0f;
	}
	else if (senzor_mm <= udaljenost_min)
	{
		rezultat = 0.0f;
	}
	else if (senzor_mm >= udaljenost_max)
	{
		rezultat = 100.0f;
	}
	else
	{
		rezultat = ((procenat_t)(senzor_mm - udaljenost_min) * 100.0f)
			/ (procenat_t)(udaljenost_max - udaljenost_min);
	}

	return rezultat;
}

void Obrada_task(void* pvParameters)
{
	uint8_t udaljenost1_min[3];
	uint8_t udaljenost1_max[4];
	uint8_t udaljenost2_min[3];
	uint8_t udaljenost2_max[4];
	int32_t udaljenost1mm_min = 0;
	int32_t udaljenost2mm_min = 0;
	int32_t udaljenost1mm_max = 0;
	int32_t udaljenost2mm_max = 0;
	int32_t senzor = 0;
	uint8_t LED = 0x00;
	uint8_t r_buffer[R_BUF_SIZE];
	uint8_t duzina_komandne_rijeci = 0;
	uint8_t duzina_ispis_niza = 0;
	uint8_t r_buffer0[3];  // senzor 0
	uint8_t r_buffer1[3];  // senzor 1

	char buffer0[20];
	char buffer1[20];
	char ispis_niza[120];

	senzor_mm_t senzor0_mm = 0;
	senzor_mm_t senzor1_mm = 0;
	procenat_t vrednost = 0.0f;

	for (;;)
	{
		xSemaphoreTake(obrada, portMAX_DELAY);

		BaseType_t result;
		result = xQueueReceive(senzor0_obrada, r_buffer0, pdMS_TO_TICKS(10));
		if (result != pdTRUE)
		{

		}

		result = xQueueReceive(senzor1_obrada, r_buffer1, pdMS_TO_TICKS(10));
		if (result != pdTRUE)
		{

		}

		result = xQueueReceive(komandna_rijec_niz, r_buffer, pdMS_TO_TICKS(10));
		if (result != pdTRUE)
		{

		}

		result = xQueueReceive(komandna_rijec_duzina, &duzina_komandne_rijeci, pdMS_TO_TICKS(10));
		if (result != pdTRUE)
		{

		}

		result = xQueueReceive(prekidac, &LED, pdMS_TO_TICKS(10));
		if (result != pdTRUE)
		{

		}

		result = xQueueReceive(vrednost1_min, udaljenost1_min, pdMS_TO_TICKS(10));
		if (result != pdTRUE)
		{

		}

		result = xQueueReceive(vrednost1_max, udaljenost1_max, pdMS_TO_TICKS(10));
		if (result != pdTRUE)
		{

		}

		result = xQueueReceive(vrednost2_min, udaljenost2_min, pdMS_TO_TICKS(10));
		if (result != pdTRUE)
		{

		}

		result = xQueueReceive(vrednost2_max, udaljenost2_max, pdMS_TO_TICKS(10));
		if (result != pdTRUE)
		{

		}

		memset(ispis_niza, 0, 120);
		duzina_ispis_niza = 0;

		udaljenost1mm_max = (int32_t)udaljenost1_max[0] * 1000 + (int32_t)udaljenost1_max[1] * 100 + (int32_t)udaljenost1_max[2] * 10 + (int32_t)udaljenost1_max[3];
		udaljenost2mm_max = (int32_t)udaljenost2_max[0] * 1000 + (int32_t)udaljenost2_max[1] * 100 + (int32_t)udaljenost2_max[2] * 10 + (int32_t)udaljenost2_max[3];
		udaljenost2mm_min = (int32_t)udaljenost2_min[0] * 100 + (int32_t)udaljenost2_min[1] * 10 + (int32_t)udaljenost2_min[2];
		udaljenost1mm_min = (int32_t)udaljenost1_min[0] * 100 + (int32_t)udaljenost1_min[1] * 10 + (int32_t)udaljenost1_min[2];

		senzor0_mm = (int32_t)r_buffer0[0] * 100 + (int32_t)r_buffer0[1];
		senzor1_mm = (int32_t)r_buffer1[0] * 100 + (int32_t)r_buffer1[1];

		if (senzor0_mm > senzor1_mm)
		{
			senzor = senzor1_mm;
			vrednost = izracunaj_procenat(senzor, udaljenost2mm_min, udaljenost2mm_max);
			printf("VREDNOST senzora 1  = %.2f \n", vrednost);
		}
		else if (senzor0_mm < senzor1_mm)
		{
			senzor = senzor0_mm;
			vrednost = izracunaj_procenat(senzor, udaljenost1mm_min, udaljenost1mm_max);
			printf("VREDNOST senzora 0  = %.2f \n", vrednost);
		}
		if (senzor0_mm == senzor1_mm)
		{
			senzor = senzor1_mm;
			vrednost = izracunaj_procenat(senzor, udaljenost2mm_min, udaljenost2mm_max);
			printf("VREDNOST JE  = %.2f \n", vrednost);
		}
		// ===== REVERSE =====
		if (((duzina_komandne_rijeci == (sizeof("REVERSE") - 1)) &&
			(strncmp((char*)r_buffer, "REVERSE", duzina_komandne_rijeci) == 0))
			|| (LED == 0x01))
		{
			ukljucen = 1;
			set_LED_BAR(1, 0x01);

			strcat((char*)ispis_niza, "Sistem: UKLJUCEN\n");

			if (vrednost >= 100) strcat((char*)ispis_niza, "NEMA DETEKCIJE");
			else if (vrednost < 100 && vrednost > 20) strcat((char*)ispis_niza, "DETEKCIJA");
			else    strcat((char*)ispis_niza, "KONTAKT DETEKCIJA");

			strcat((char*)ispis_niza, "\n");
			strcat((char*)ispis_niza, "Senzor 0: ");
			sprintf(buffer0, "%d mm\n", senzor0_mm);
			strcat(ispis_niza, buffer0);

			strcat((char*)ispis_niza, "Senzor 1: ");
			sprintf(buffer1, "%d mm\n", senzor1_mm);
			strcat(ispis_niza, buffer1);
		}
		// ===== PARK NEUTRAL DRIVE =====
		else if (((duzina_komandne_rijeci == (sizeof("PARK") - 1)) &&
			(strncmp((char*)r_buffer, "PARK", duzina_komandne_rijeci) == 0))
			&& (LED == 0x00))
		{
			ukljucen = 0;
			set_LED_BAR(1, 0x00);
			strcat((char*)ispis_niza, "Sistem: ISKLJUCEN\n");
		}

		else if (((duzina_komandne_rijeci == (sizeof("DRIVE") - 1)) &&
			(strncmp((char*)r_buffer, "DRIVE", duzina_komandne_rijeci) == 0))
			&& (LED == 0x00))
		{
			ukljucen = 0;
			set_LED_BAR(1, 0x00);
			strcat((char*)ispis_niza, "Sistem: ISKLJUCEN\n");
		}

		else if (((duzina_komandne_rijeci == (sizeof("NEUTRAL") - 1)) &&
			(strncmp((char*)r_buffer, "NEUTRAL", duzina_komandne_rijeci) == 0))
			&& (LED == 0x00))
		{
			ukljucen = 0;
			set_LED_BAR(1, 0x00);
			strcat((char*)ispis_niza, "Sistem: ISKLJUCEN\n");
		}

		else
		{
			strcat((char*)ispis_niza, "NEPOZNATA KOMANDA\n");
		}

		duzina_ispis_niza = strlen((char*)ispis_niza);
		send_serial_character(COM_CH2, 13);

		result = xQueueSend(kanal_2_ispis_rijeci, ispis_niza, 0U);
		if (result != pdTRUE)
		{

		}

		result = xQueueSend(procenat, &vrednost, 0U);
		if (result != pdTRUE)
		{

		}

		result = xQueueSend(kanal_2_ispis_duzine, &duzina_ispis_niza, 0U);
		if (result != pdTRUE)
		{

		}

		result = xQueueSend(stanje, &ukljucen, 0U);
		if (result != pdTRUE)
		{

		}
	}
}

void task_obrada_kalibracija(void* pvParameters) {

	uint8_t duzina1 = 0;
	uint8_t kal[26];
	uint8_t niz[] = { 48, 49, 50, 51, 52, 53, 54, 55, 56, 57 };
	uint8_t udaljenost1_min[3] = { 0 };
	uint8_t udaljenost1_max[4] = { 0 };
	uint8_t udaljenost2_min[3] = { 0 };
	uint8_t udaljenost2_max[4] = { 0 };
	uint8_t procenat1_max[3] = { 0 };
	uint8_t procenat1_min[1] = { 0 };
	uint8_t procenat2_max[3] = { 0 };
	uint8_t procenat2_min[1] = { 0 };

	for (;;)
	{
		xQueueReceive(kalibracija, kal, portMAX_DELAY);
		xQueueReceive(komandna_rijec_duzina, &duzina1, portMAX_DELAY);

		for (uint8_t i = 0; i < duzina1; i++)
		{
			if ((kal[i] == (uint8_t)95) && (i == (uint8_t)12) && duzina1 == 24)
			{
				if (kal[11] == (uint8_t)49)
				{
					printf("usao u 1 max\n");
					udaljenost1_max[0] = kal[i + 1];
					if (kal[i + 2] != 109) { udaljenost1_max[1] = kal[i + 2]; }
					if (kal[i + 3] != 109) { udaljenost1_max[2] = kal[i + 3]; }
					if (kal[i + 4] != 109) { udaljenost1_max[3] = kal[i + 4]; }
				}
				else if (kal[11] == (uint8_t)50)
				{
					printf("usao u 2max\n");
					udaljenost2_max[0] = kal[i + 1];
					if (kal[i + 2] != 109) { udaljenost2_max[1] = kal[i + 2]; }
					if (kal[i + 3] != 109) { udaljenost2_max[2] = kal[i + 3]; }
					if (kal[i + 4] != 109) { udaljenost2_max[3] = kal[i + 4]; }
				}
				else
				{

				}
			}
			else if ((kal[i] == (uint8_t)95) && (i == (uint8_t)12) && duzina1 == 21)
			{
				if (kal[11] == (uint8_t)49)
				{
					printf("usao u 1 min\n");
					udaljenost1_min[0] = kal[i + 1];
					if (kal[i + 2] != 109) { udaljenost1_min[1] = kal[i + 2]; }
					if (kal[i + 3] != 109) { udaljenost1_min[2] = kal[i + 3]; }
				}
				else if (kal[11] == (uint8_t)50)
				{
					printf("usao u 2 min\n");
					udaljenost2_min[0] = kal[i + 1];
					if (kal[i + 2] != 109) { udaljenost2_min[1] = kal[i + 2]; }
					if (kal[i + 3] != 109) { udaljenost2_min[2] = kal[i + 3]; }
				}
				else
				{

				}
			}
			if ((kal[i] == 95) && (i > 12) && duzina1 == 24)
			{
				if (kal[11] == (uint8_t)49)
				{
					printf("Usao u procenat1max!\n");
					procenat1_max[0] = kal[i + 1];
					if (kal[i + 2] != 37) { procenat1_max[1] = kal[i + 2]; }
					if (kal[i + 3] != 37) { procenat1_max[2] = kal[i + 3]; }
				}
				else if (kal[11] == (uint8_t)50)
				{
					printf("Usao u procenat2max!\n");
					procenat2_max[0] = kal[i + 1];
					if (kal[i + 2] != 37) { procenat2_max[1] = kal[i + 2]; }
					if (kal[i + 3] != 37) { procenat2_max[2] = kal[i + 3]; }
				}
				else
				{

				}
			}
			else if ((kal[i] == 95) && (i > 12) && duzina1 == 21)
			{
				if (kal[11] == (uint8_t)49)
				{
					printf("Usao u procenat1min!\n");
					procenat1_min[0] = kal[i + 1];
				}
				else if (kal[11] == (uint8_t)50)
				{
					printf("Usao u procenat2min!\n");
					procenat2_min[0] = kal[i + 1];
				}
				else
				{

				}
			}

		}
		for (uint8_t i = 0; i < 10; i++)
		{
			if (udaljenost1_max[0] == niz[i])
			{
				udaljenost1_max[0] = i;
			}
			if (udaljenost1_max[1] == niz[i])
			{
				udaljenost1_max[1] = i;
			}
			if (udaljenost1_max[2] == niz[i])
			{
				udaljenost1_max[2] = i;
			}
			if (udaljenost1_max[3] == niz[i])
			{
				udaljenost1_max[3] = i;
			}

			if (udaljenost1_min[0] == niz[i])
			{
				udaljenost1_min[0] = i;
			}
			if (udaljenost1_min[1] == niz[i])
			{
				udaljenost1_min[1] = i;
			}
			if (udaljenost1_min[2] == niz[i])
			{
				udaljenost1_min[2] = i;
			}

			if (udaljenost2_max[0] == niz[i])
			{
				udaljenost2_max[0] = i;
			}
			if (udaljenost2_max[1] == niz[i])
			{
				udaljenost2_max[1] = i;
			}
			if (udaljenost2_max[2] == niz[i])
			{
				udaljenost2_max[2] = i;
			}
			if (udaljenost2_max[3] == niz[i])
			{
				udaljenost2_max[3] = i;
			}

			if (udaljenost2_min[0] == niz[i])
			{
				udaljenost2_min[0] = i;
			}
			if (udaljenost2_min[1] == niz[i])
			{
				udaljenost2_min[1] = i;
			}
			if (udaljenost2_min[2] == niz[i])
			{
				udaljenost2_min[2] = i;
			}

			if (procenat1_max[0] == niz[i])
			{
				procenat1_max[0] = i;
			}
			if (procenat1_max[1] == niz[i])
			{
				procenat1_max[1] = i;
			}
			if (procenat1_max[2] == niz[i])
			{
				procenat1_max[2] = i;
			}

			if (procenat2_max[0] == niz[i])
			{
				procenat2_max[0] = i;
			}
			if (procenat2_max[1] == niz[i])
			{
				procenat2_max[1] = i;
			}
			if (procenat2_max[2] == niz[i])
			{
				procenat2_max[2] = i;
			}

			if (procenat1_min[0] == niz[i])
			{
				procenat1_min[0] = i;
			}
			if (procenat2_min[0] == niz[i])
			{
				procenat2_min[0] = i;
			}


		}

		xQueueSend(vrednost1_max, udaljenost1_max, portMAX_DELAY);
		xQueueSend(vrednost1_min, udaljenost1_min, portMAX_DELAY);
		xQueueSend(vrednost2_max, udaljenost2_max, portMAX_DELAY);
		xQueueSend(vrednost2_min, udaljenost2_min, portMAX_DELAY);
	}

}

void LED_bar_task(void* pvParameters)
{
	uint8_t d;

	for (;;)
	{
		xSemaphoreTake(LED_INT_BinarySemaphore, portMAX_DELAY);

		get_LED_BAR(0, &d);

		xQueueSend(prekidac, &d, 0U);
	}
}

void senzor0_task(void* pvParameters)
{
	uint8_t ll = 0;
	uint8_t r_buffer0[R_BUF_SIZE_tasks];

	for (;;)
	{
		xSemaphoreTake(RXC_BinarySemaphore_0, portMAX_DELAY);
		get_serial_character(COM_CH0, &ll);

		if (ll == 0x0d)
		{
			if (r_point0 == R_BUF_SIZE_tasks) {

				xQueueSend(senzor0_obrada, &r_buffer0, 0U);
				xQueueSend(senzor0_seg7, &r_buffer0, 0U);
			}
			r_point0 = 0;
		}
		else if (r_point0 < R_BUF_SIZE_tasks)
		{
			r_buffer0[r_point0++] = ll;
		}
	}
}

void senzor1_task(void* pvParameters)
{
	uint8_t dd = 0;
	uint8_t r_buffer1[R_BUF_SIZE_tasks];

	for (;;)
	{

		xSemaphoreTake(RXC_BinarySemaphore_1, portMAX_DELAY);
		get_serial_character(COM_CH1, &dd);
		if (dd == 0x0d)
		{
			if (r_point1 == R_BUF_SIZE_tasks)
			{
				xQueueSend(senzor1_obrada, &r_buffer1, 0U);
				xQueueSend(senzor1_seg7, &r_buffer1, 0U);
			}
			r_point1 = 0;
		}
		else if (r_point1 < R_BUF_SIZE_tasks)
		{
			r_buffer1[r_point1++] = dd;
		}
	}
}

void Seg7_task(void* pvParameters)
{
	uint8_t r_buffer0[R_BUF_SIZE_tasks];
	uint8_t r_buffer1[R_BUF_SIZE_tasks];
	uint8_t duzina_ispis_niza;
	ukljucen = 0U;

	for (;;)
	{
		xSemaphoreTake(seg7, portMAX_DELAY);

		xQueueReceive(stanje, &ukljucen, pdMS_TO_TICKS(10));
		xQueueReceive(senzor0_seg7, &r_buffer0, pdMS_TO_TICKS(10));
		xQueueReceive(senzor1_seg7, &r_buffer1, pdMS_TO_TICKS(10));

		if (ukljucen == 1U)
		{
			if ((r_buffer0[0] > (uint8_t)99) || (r_buffer0[1] > (uint8_t)99))
			{
				select_7seg_digit(0);
				set_7seg_digit(hexnum[(uint8_t)1]);
				select_7seg_digit(1);
				set_7seg_digit(hexnum[(uint8_t)0]);
				select_7seg_digit(2);
				set_7seg_digit(hexnum[(uint8_t)0]);
				select_7seg_digit(3);
				set_7seg_digit(hexnum[(uint8_t)0]);
			}

			else
			{
				select_7seg_digit(0);
				set_7seg_digit(hexnum[(uint8_t)r_buffer0[0] / 10]);
				select_7seg_digit(1);
				set_7seg_digit(hexnum[(uint8_t)r_buffer0[0] % 10]);
				select_7seg_digit(2);
				set_7seg_digit(hexnum[(uint8_t)r_buffer0[1] / 10]);
				select_7seg_digit(3);
				set_7seg_digit(hexnum[(uint8_t)r_buffer0[1] % 10]);
			}

			if ((r_buffer1[0] > (uint8_t)99) || (r_buffer1[1] > (uint8_t)99))
			{
				select_7seg_digit(5);
				set_7seg_digit(hexnum[(uint8_t)1]);
				select_7seg_digit(6);
				set_7seg_digit(hexnum[(uint8_t)0]);
				select_7seg_digit(7);
				set_7seg_digit(hexnum[(uint8_t)0]);
				select_7seg_digit(8);
				set_7seg_digit(hexnum[(uint8_t)0]);
			}

			else
			{
				select_7seg_digit(5);
				set_7seg_digit(hexnum[(uint8_t)r_buffer1[0] / 10]);
				select_7seg_digit(6);
				set_7seg_digit(hexnum[(uint8_t)r_buffer1[0] % 10]);
				select_7seg_digit(7);
				set_7seg_digit(hexnum[(uint8_t)r_buffer1[1] / 10]);
				select_7seg_digit(8);
				set_7seg_digit(hexnum[(uint8_t)r_buffer1[1] % 10]);
			}
		}
	}
}

void Led_task(void* pvParameters)
{
	ukljucen = 0U;
	procenat_t procenat_led;

	for (;;)
	{
		xQueueReceive(stanje, &ukljucen, pdMS_TO_TICKS(10));
		if (xQueueReceive(procenat, &procenat_led, portMAX_DELAY) == pdPASS)
		{
			if (ukljucen == 1U)
			{
				if (procenat_led <= (procenat_t)20.0f)
				{
					set_LED_BAR(2, 0xFF);
				}
				else if ((procenat_led > (procenat_t)20.0f) && (procenat_led <= (procenat_t)30.0f))
				{
					set_LED_BAR(2, 0x7F);
				}
				else if ((procenat_led > (procenat_t)30.0f) && (procenat_led <= (procenat_t)40.0f))
				{
					set_LED_BAR(2, 0x3F);
				}
				else if ((procenat_led > (procenat_t)40.0f) && (procenat_led <= (procenat_t)50.0f))
				{
					set_LED_BAR(2, 0x1F);
				}
				else if ((procenat_led > (procenat_t)50.0f) && (procenat_led <= (procenat_t)60.0f))
				{
					set_LED_BAR(2, 0x0F);
				}
				else if ((procenat_led > (procenat_t)60.0f) && (procenat_led <= (procenat_t)70.0f))
				{
					set_LED_BAR(2, 0x07);
				}
				else if ((procenat_led > (procenat_t)70.0f) && (procenat_led <= (procenat_t)90.0f))
				{
					set_LED_BAR(2, 0x03);
				}
				else if ((procenat_led > (procenat_t)90.0f) && (procenat_led < (procenat_t)100.0f))
				{
					set_LED_BAR(2, 0x01);
				}
				else if ((procenat_led == (procenat_t)100.0f))
				{
					set_LED_BAR(2, 0x00);
				}
			}
			else
			{
				set_LED_BAR(2, 0x00);
			}
		}
	}
}

