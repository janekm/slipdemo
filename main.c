/**************************************************************************//**
 * @file
 * @brief Simple LCD blink demo for EFM32_Gxxx_STK
 * @author Energy Micro AS
 * @version 2.1.2
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2009 Energy Micro AS, http://www.energymicro.com</b>
 ******************************************************************************
 *
 * This source code is the property of Energy Micro AS. The source and compiled
 * code may only be used on Energy Micro "EFM32" microcontrollers.
 *
 * This copyright notice may not be removed from the source code nor changed.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Energy Micro AS has no
 * obligation to support this Software. Energy Micro AS is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Energy Micro AS will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 *****************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "efm32.h"
#include "efm32_chip.h"
#include "efm32_emu.h"
#include "efm32_gpio.h"
#include "efm32_i2c.h"
#include "efm32_usart.h"
#include "efm32_rtc.h"
#include "efm32_cmu.h"
#include "efm32_adc.h"
#include "efm32_timer.h"

#include "config.h"
#include "nrf24.h"
#include "nrf24l01.h"
#include "MMA845XQ.h"
#include "nrf24_config.h"

#include <math.h>


static volatile int NRF_Interrupt = 0;
static volatile int MMA_Capture = 0;

uint32_t RTCVal = 0;

uint8_t buf[192*2];

uint8_t radio_buf[32];
volatile uint8_t nrf_status = 0;

int REDval = 0;
int GREENval = 1025;

void GPIO_EVEN_IRQHandler(void) 
{
  /* Acknowledge interrupt */
  if (GPIO->IF & (1 << NRF_INT_PIN)) 
  {
    NRF_Interrupt++;
    if((GPIO->P[2].DIN & (1<<NRF_INT_PIN))==0) {
      nrf_status = NRF_ReadRegister(NRF_STATUS);
    }
    GPIO->IFC = (1 << NRF_INT_PIN);
  }
}

void GPIO_ODD_IRQHandler(void)
{
  if (GPIO->IF & (1 << MMA_INT_PIN)) 
  {
    GPIO->P[0].DOUT ^= (1 << 3);
    MMA_Capture++;
    RTCVal = RTC_CounterGet();
    GPIO->IFC = (1 << MMA_INT_PIN);
  }
}

void LETIMER0_IRQHandler(void)
{
}
void TIMER0_IRQHandler(void)
{ 
  TIMER_CompareBufSet(TIMER0, 0, REDval);
  TIMER_CompareBufSet(TIMER0, 1, GREENval);
  REDval = (REDval + 1) % 1025;
  GREENval = (GREENval - 1) % 1025;
//  if(GPIO->P[NRF_INT_PORT].DIN & (1 << NRF_INT_PIN)) {
//    GPIO->P[LED_GREEN_PORT].DOUT &= ~(1 << LED_GREEN_PIN);
//    GPIO->P[LED_RED_PORT].DOUT |= (1 << LED_RED_PIN);
//  } else {
//    GPIO->P[LED_RED_PORT].DOUT &= ~(1 << LED_RED_PIN);
//    GPIO->P[LED_GREEN_PORT].DOUT |= (1 << LED_GREEN_PIN);
//  }
  TIMER_IntClear(TIMER3, TIMER_IF_OF);
}

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  
  
  /* Chip errata */
  CHIP_Init();
  /* Ensure core frequency has been updated */
  SystemCoreClockUpdate();

  IO_Init();
  UART1->ROUTE = UART_ROUTE_LOCATION_LOC3
          | UART_ROUTE_TXPEN | UART_ROUTE_RXPEN;
  
  //InitRGBLEDPWM();
  uart_init(UART1); // for printf
  GPIO->P[0].DOUT &= ~(1 << 0);
  printf("Hello World\n");

  MMAInit(); // set up accelerometer
  MMARegReadN(OUT_X_MSB_REG, 6, buf);


  nrf_status = NRF_ReadRegister(NRF_FIFO_STATUS);
  nrf_status = NRF_Status();
  nrf_status = NRF_ReadRegister(NRF_STATUS);
  NRF_SetupTX(); // set up radio as transmitter


  NRF_PowerDown(); // power down radio until we need it to save power

  while(1)
  {
    //nrf_status = NRF_Status();
    if (NRF_Interrupt>0) 
    {
      
      NRF_CE_lo;
      GPIO->P[RXEN_PORT].DOUT &= ~(1 << RXEN_PIN); 
      if (nrf_status & 0x10) 
      {
      }
      else if (nrf_status & 0x20)
      {
      }
      NRF_WriteRegister(NRF_STATUS, 0x7E);
      NRF_Interrupt--;
    }
    if (MMA_Capture>0) {

  
      MMARegReadN(OUT_X_MSB_REG, 6, buf);
      printf("X: %d, Y: %d, Z: %d\n", (buf[0]<<8) | buf[1], (buf[2]<<8) | buf[3], (buf[4]<<8) | buf[5]);
      NRF_PowerUp();
      NRF_WriteRegister(NRF_CONFIG, 0x0E); // Power Up, Transmitter
      NRF_SendCommand(0xE1, 0xFF);
      NRF_CE_hi;
      GPIO->P[RXEN_PORT].DOUT |= (1 << RXEN_PIN); 

    }    
    if((MMA_Capture<=0) && (NRF_Interrupt<=0)) 
    {
      EMU_EnterEM2(true); // send processor to sleep
    }
  }
}

