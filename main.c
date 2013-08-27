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
#include "efm32_int.h"

#include "config.h"
#include "nrf24.h"
#include "nrf24l01.h"
#include "MMA845XQ.h"
#include <string.h>

#include "nrf24_config.h"

#include <math.h>

#include "data.h"

static volatile int NRF_Interrupt = 0;
static volatile int MMA_Capture = 0;

uint32_t RTCVal = 0;

uint8_t buf[192*2];

uint8_t radio_buf[32];
volatile uint8_t nrf_status = 0;

int REDval = 0;
int GREENval = 1025;

Accel_Vector_Type accelReading;

#define PACKET_TYPE_ACCELDATA 1

typedef struct {
  uint8_t type;
  uint8_t nodeID;
  int16_t accelX;
  int16_t accelY;
  int16_t accelZ;
  uint8_t padding[24];
} __attribute__ ((packed)) Packet_Type ;

sample_t packet;
int packetIndex = 0;

void GPIO_EVEN_IRQHandler(void) 
{
  /* Acknowledge interrupt */
  if (GPIO->IF & (1 << NRF_INT_PIN)) 
  {
    NRF_Interrupt++;
    GPIO->IFC = (1 << NRF_INT_PIN);
  }
}

void GPIO_ODD_IRQHandler(void)
{
  if (GPIO->IF & (1 << MMA_INT_PIN)) 
  {
    //GPIO->P[0].DOUT ^= (1 << 3);
    MMA_Capture++;
    //RTCVal = RTC_CounterGet();
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
  //REDval = (REDval + 1) % 1025;
  //GREENval = (GREENval - 1) % 1025;
//  if(GPIO->P[NRF_INT_PORT].DIN & (1 << NRF_INT_PIN)) {
//    GPIO->P[LED_GREEN_PORT].DOUT &= ~(1 << LED_GREEN_PIN);
//    GPIO->P[LED_RED_PORT].DOUT |= (1 << LED_RED_PIN);
//  } else {
//    GPIO->P[LED_RED_PORT].DOUT &= ~(1 << LED_RED_PIN);
//    GPIO->P[LED_GREEN_PORT].DOUT |= (1 << LED_GREEN_PIN);
//  }
  TIMER_IntClear(TIMER0, TIMER_IF_OF);
}

sample_t buffer[5000];
bufferindex = 0;
bufferfill = 0;

int node2prev = 0;
int node48prev = 0;
int node45prev = 0;



void printPacket(sample_t packet) {
  int secondtime;
  secondtime = packet.hour * 60 * 60 + packet.minute * 60 + packet.second;
  
  if(packet.nodeId == 2) {
    if (secondtime < (node2prev + 20*60)) {
      //node2prev = secondtime;
      return;
    }
    node2prev = secondtime;
  }
  if(packet.nodeId == 45) {
    if (secondtime < (node45prev + 20*60)) {
      //node45prev = secondtime;
      return;
    }
    node2prev = secondtime;
  }
  if(packet.nodeId == 48) {
    if (secondtime < (node48prev + 20*60)) {
      //node2prev = secondtime;
      return;
    }
    node48prev = secondtime;
  }
  printf("%d, %d, %d, %d, %d, ", packet.year, packet.nodeId, packet.month, packet.day, packet.hour);
  printf("%d, %d, %ld, %ld, %d, ", packet.minute, packet.second, packet.latitude, packet.longitude, packet.nsats);
  printf("%d, %d, %d, %d, %d, %d, %d\n", packet.accelX, packet.accelY, packet.magX, packet.magY, packet.magZ, packet.light_mant, packet.light_exp);
}

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  char inputc;
  uint8_t msgbuf[32];
  
  /* Chip errata */
  CHIP_Init();
  /* Ensure core frequency has been updated */
  SystemCoreClockUpdate();

  IO_Init();
  UART1->ROUTE = UART_ROUTE_LOCATION_LOC3
          | UART_ROUTE_TXPEN | UART_ROUTE_RXPEN;
  
  TIMER_IntClear(TIMER0, TIMER_IF_OF);
  //InitRGBLEDPWM();
  TIMER_IntClear(TIMER0, TIMER_IF_OF);
  uart_init(UART1); // for printf

  #ifndef BASESTATION
  MMAInit(); // set up accelerometer
  MMARegReadN(OUT_X_MSB_REG, 6, buf);
  #endif // !BASESTATION
  //while(1) {
  //  EMU_EnterEM2(true);
  //}

  NRF_SetupTX(); // set up radio

  #ifdef BASESTATION
  //NRF_EnableRX();
  //RXEN_hi;
  #endif // BASESTATION
  
  NRF_WriteRegister(NRF_STATUS, 0x70); // Clear all radio interrupts

  printf("Send s to begin download\n");
  while(!(UART1->STATUS & UART_STATUS_RXDATAV));
  //inputc = UART1->RXDATA;
  //printf("Begin Download\n");
  //NRF_EnableRX();
  //RXEN_hi;
  
  while(1) {
  inputc = UART1->RXDATA;
  if (inputc == 's') {
    printf("Begin Download\n");
    NRF_EnableRX();
    RXEN_hi;
  }
  if (inputc == 'o') {
    printf("Stop Download\n");
    NRF_CE_lo;
    RXEN_lo;
    NRF_WriteRegister(NRF_CONFIG, 0x0C);
  }
  if (inputc == 'd') {
    for(int i = 0; i < bufferfill; i++) {
      printPacket(buffer[i]);
    }
  }
  while(!(UART1->STATUS & UART_STATUS_RXDATAV))
  {
    //nrf_status = NRF_Status();
    if (NRF_Interrupt>0) 
    {
      
      //RXEN_lo; 
      //NRF_CE_lo;
      //NRF_WriteRegister(NRF_CONFIG, 0x0C);
      nrf_status = NRF_ReadRegister(NRF_STATUS);
      
      if (nrf_status & 0x10) 
      {
        NRF_WriteRegister(NRF_STATUS, 0x10);
        //printf("nrf_status MAX_RT\n");
      } else if (nrf_status & 0x20)
      {
        NRF_WriteRegister(NRF_STATUS, 0x7E);
        
        INT_Disable();
        RXEN_hi;
        NRF_TransmitPacket(32, (uint8_t *)&packet);
        INT_Enable();
          GPIO->P[0].DOUT ^= (1 << 3);
      } else if (nrf_status & 0x40)
      {
        NRF_WriteRegister(NRF_STATUS, 0x70);
        //printf("nrf_status DATA_READY\n");
        NRF_ReceivePayload(NRF_R_RX_PAYLOAD, 32, msgbuf);

        if (false) { // msgbuf[0] == 0xEA) {
          printf("GSV for %d: ", msgbuf[1]);
          for (int i = 2; i < 13; i++) {
            printf("%d, ", msgbuf[i]);
          }
          printf("%d\n", msgbuf[13]);

        } else {
          memcpy(&packet, msgbuf, 32);
          memcpy(&buffer[bufferfill], msgbuf, 32);
          printPacket(packet);
        }
        //NRF_SendCommand(NRF_FLUSH_RX, 0xFF);
        GPIO->P[0].DOUT ^= (1 << 3);
        RXEN_hi; 
        NRF_CE_hi;
      }
      
      INT_Disable();
      NRF_Interrupt--;
      INT_Enable();
      
    }

    if((MMA_Capture<=0) && (NRF_Interrupt<=0)) 
    {
      EMU_EnterEM1(); // send processor to sleep
    }
  }
}
}

