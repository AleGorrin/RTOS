/* Copyright 2017-2018, Eric Pernia
 * All rights reserved.
 *
 * This file is part of sAPI Library.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*==================[inlcusiones]============================================*/

// Includes de FreeRTOS
#include "../../Entrega_1/inc/FreeRTOSConfig.h"
#include "../../Entrega_1/inc/tipos.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
// sAPI header
#include "sapi.h"

/*==================[definiciones y macros]==================================*/

/*==================[definiciones de datos internos]=========================*/
gpioMap_t teclas[] = {TEC1,TEC2};
gpioMap_t leds  [] = {LED2, LED3};

#define N_TECLAS sizeof(teclas)/sizeof(gpioMap_t) // 4*

tecla_led_t tecla_led_config[N_TECLAS];

/*==================[definiciones de datos externos]=========================*/

DEBUG_PRINT_ENABLE;

/*==================[declaraciones de funciones internas]====================*/
bool_t tecla_led_tarea_init(void);
bool_t tecla_led_tareas_crear(void);
/*==================[declaraciones de funciones externas]====================*/

// Prototipo de funcion de la tarea
void tarea_tecla( void* taskParmPtr );
void tarea_led( void* taskParmPtr );
/*==================[funcion principal]======================================*/

bool_t tecla_led_tarea_init(void)
{
	uint8_t i;
	// Creacion
	for(i=0; i < N_TECLAS; i++)
	{
		tecla_led_config[i].tecla = teclas[i];
		tecla_led_config[i].led = leds[i];
		tecla_led_config[i].sem_tec_pulsada = xSemaphoreCreateBinary();
		tecla_led_config[i].mutex		    = xSemaphoreCreateMutex();
	}
	return TRUE;
}

bool_t tecla_led_tareas_crear(void)
{
	uint8_t i;
	BaseType_t res_task_tecla[N_TECLAS];
	BaseType_t res_task_led[N_TECLAS];
	// Creacion
	for( i=0; i < N_TECLAS; i++ )
	{
		res_task_tecla[i] =
				xTaskCreate(
						tarea_tecla,                     	// Funcion de la tarea a ejecutar
						(const char *)"tarea_tecla_4",      // Nombre de la tarea como String amigable para el usuario
						configMINIMAL_STACK_SIZE*2, 		// Cantidad de stack de la tarea
						&tecla_led_config[i],               // Parametros de tarea
						tskIDLE_PRIORITY+1,         		// Prioridad de la tarea
						0                           		// Puntero a la tarea creada en el sistema
				);
		res_task_led[i] =
				xTaskCreate(
						tarea_led,                          // Funcion de la tarea a ejecutar
						(const char *)"tarea_led_B",        // Nombre de la tarea como String amigable para el usuario
						configMINIMAL_STACK_SIZE*2,         // Cantidad de stack de la tarea
						&tecla_led_config[i],               // Parametros de tarea
						tskIDLE_PRIORITY+1,                 // Prioridad de la tarea
						0                                   // Puntero a la tarea creada en el sistema
				);
	}
	return TRUE;
}

// FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE ENCENDIDO O RESET.
int main(void)
{
   // ---------- CONFIGURACIONES ------------------------------
   boardConfig();								// Inicializar y configurar la plataforma

   debugPrintConfigUart( UART_USB, 115200 );	// UART for debug messages
   debugPrintlnString( "EJ_2_1_multi_mutex" );

   if (tecla_led_tarea_init() == FALSE )		// Inicializa arreglo tecla_led, crea su semaforo y mutex
	   return ERROR;

   if ( tecla_led_tareas_crear() == FALSE )		// creamos todas las tareas en freeRTOS
	   return ERROR;

   vTaskStartScheduler();						// Iniciar scheduler

   // ---------- REPETIR POR SIEMPRE --------------------------
   while( TRUE ) {
      // Si cae en este while 1 significa que no pudo iniciar el scheduler
   }

   // NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa se ejecuta
   // directamenteno sobre un microcontroladore y no es llamado por ningun
   // Sistema Operativo, como en el caso de un programa para PC.
   return 0;
}

/*==================[definiciones de funciones internas]=====================*/

/*==================[definiciones de funciones externas]=====================*/

// Implementacion de funcion de la tarea
void tarea_led( void* taskParmPtr )
{
	tecla_led_t* config = (tecla_led_t*) taskParmPtr;
	TickType_t diferencia;
   // ---------- REPETIR POR SIEMPRE --------------------------
   while(TRUE) {

	   xSemaphoreTake(config->sem_tec_pulsada, portMAX_DELAY );

	   // Abrimos seccion critica
	   xSemaphoreTake(config->mutex, portMAX_DELAY);

	   diferencia = config->tiempo_medido;
	   config->tiempo_medido = 0;
	   // Cerramos seccion critica

	   xSemaphoreGive(config->mutex);

	   gpioWrite(config->led, ON);
	   vTaskDelay( diferencia );
	   gpioWrite(config->led, OFF);
   }
   vTaskDelete(NULL);
}
void tarea_tecla( void* taskParmPtr )
{
   // ---------- CONFIGURACIONES ------------------------------

	tecla_led_t* config = (tecla_led_t*) taskParmPtr;

	fsmButtonInit(config);

   // ---------- REPETIR POR SIEMPRE --------------------------
   while(TRUE) {
	   fsmButtonUpdate( config );
	   vTaskDelay( 1 / portTICK_RATE_MS );
   }
   vTaskDelete(NULL);
}

/*==================[fin del archivo]========================================*/
