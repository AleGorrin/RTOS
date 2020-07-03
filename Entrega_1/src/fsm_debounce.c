/* Copyright 2018, Eric Pernia.
 * All rights reserved.
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
 *
 */
 
#include "../../Entrega_1/inc/FreeRTOSConfig.h"
#include "../../Entrega_1/inc/tipos.h"
#include "sapi.h"
#include "FreeRTOS.h"
#include "task.h"

void fsmButtonError( tecla_led_t* config );
void fsmButtonInit( tecla_led_t* config );
void fsmButtonUpdate( tecla_led_t* config );
void buttonPressed( tecla_led_t* config );
void buttonReleased( tecla_led_t* config );

void tarea_led( void* taskParmPtr );

void buttonPressed( tecla_led_t* config )
{
	config->tiempo_down = xTaskGetTickCount();
}

void buttonReleased( tecla_led_t* config )
{
	config->tiempo_up = xTaskGetTickCount();
	// Abrimos seccion critica

	xSemaphoreTake(config->mutex, portMAX_DELAY);

	config->tiempo_medido = config->tiempo_up - config->tiempo_down;

	// Cerramos seccion critica
	xSemaphoreGive(config->mutex);

	if ( config->tiempo_medido > 0 )
		xSemaphoreGive( config->sem_tec_pulsada);
}

void fsmButtonError( tecla_led_t* config )
{
   config->fsmButtonState = STATE_BUTTON_UP;
}

void fsmButtonInit( tecla_led_t* config )
{
   config->fsmButtonState = STATE_BUTTON_UP;  // Set initial state
   config->contRising = 0;
   config->contFalling = 0;
}

// FSM Update Sate Function
void fsmButtonUpdate( tecla_led_t* config )
{
   switch( config->fsmButtonState )
   {
      case STATE_BUTTON_UP: 
         /* CHECK TRANSITION CONDITIONS */
         if( !gpioRead(config->tecla) ){
            config->fsmButtonState = STATE_BUTTON_FALLING;
         }
      break;

      case STATE_BUTTON_DOWN:
         /* CHECK TRANSITION CONDITIONS */
         if( gpioRead(config->tecla) ){
        	 config->fsmButtonState = STATE_BUTTON_RISING;
         }
      break;

      case STATE_BUTTON_FALLING:      
         /* ENTRY */

         /* CHECK TRANSITION CONDITIONS */
         if( config->contFalling >= DEBOUNCE_TIME ){
            if( !gpioRead(config->tecla) ){
            	config->fsmButtonState = STATE_BUTTON_DOWN;
               buttonPressed(config);
            } else{
            	config->fsmButtonState = STATE_BUTTON_UP;
            }
            config->contFalling = 0;
         }
         config->contFalling++;

         /* LEAVE */
         break;

      case STATE_BUTTON_RISING:      
         /* ENTRY */

         /* CHECK TRANSITION CONDITIONS */

         if( config->contRising >= DEBOUNCE_TIME ){
            if( gpioRead(config->tecla) )
            {
            	config->fsmButtonState = STATE_BUTTON_UP;
                buttonReleased(config);
            }
            else
            {
            	config->fsmButtonState = STATE_BUTTON_DOWN;
            }
            config->contRising = 0;
         }
         config->contRising++;
         /* LEAVE */
         break;

      default:
         fsmButtonError(config);
         break;
   }
}
