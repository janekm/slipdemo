#include <efm32.h>
#include <stdio.h>

const static uint8_t sirf_command_NMEA[] =  {0xA0, 0xA2, 0x00, 0x18, 0x81, 0x02, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x12, 0xC0, 0x01, 0x60, 0xB0, 0xB3};
const static uint8_t sirf_command_MPM[] = {0xA0, 0xA2, 0x00, 0x06, 0xDA, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDC, 0xB0, 0xB3};
const static uint8_t sirf_command_PTF[] = {0xA0, 0xA2, 0x00, 0x0E, 0xDA, 0x04, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 0x0F, 0xA0, 0x00, 0x02, 0xBF ,0x20, 0x02, 0xE6, 0xB0, 0xB3};
const static uint8_t sirf_command_fullPowerMode = {0xA0, 0xA2, 0x00, 0x02, 0xDA, 0x00, 0x00, 0xDA, 0xB0, 0xB3};
// this returns a single binary byte that is the checksum 

uint8_t nmea_buffer[256];
uint32_t nmea_len = 0;
uint32_t nmea_msg_rcvd = 0;
uint32_t valid_fix_count = 0;

// you must convert it to hex if you are going to print it or send it 
uint8_t nmea_generateChecksum(char *strPtr) 
{ 
   int p; 
   char c; 
   uint8_t chksum; 

   c = strPtr[0]; // get first chr 
   chksum = c; 
   p = 1; 
   while ( c != 0x00 ) 
     { 
     c = strPtr[p]; // get next chr 
     if ( c != 0x00 ) { chksum = chksum ^ c; } 
     p++; 
     } 

   return chksum; 
} 

void uart_send_str(USART_TypeDef *uart, char *msg) 
{
    uint8_t c;
    while ((c = *(msg++))!=0) {
        while (!(uart->STATUS & UART_STATUS_TXBL));
        uart->TXDATA = c;
    }
}

void leuart_send_str(LEUART_TypeDef *uart, char *msg) 
{
    uint8_t c;
    while ((c = *(msg++))!=0) {
        while (!(uart->STATUS & LEUART_STATUS_TXBL));
        uart->TXDATA = c;
    }
}

void uart_send_array(USART_TypeDef *uart, uint8_t *msg, uint32_t len) 
{
    for (int i = 0; i < len; i++) {
        while (!(uart->STATUS & UART_STATUS_TXBL));
        uart->TXDATA = msg[i];
    }
}

void leuart_send_array(LEUART_TypeDef *uart, uint8_t *msg, uint32_t len) 
{
    for (int i = 0; i < len; i++) {
        while (!(uart->STATUS & LEUART_STATUS_TXBL));
        uart->TXDATA = msg[i];
    }
}


// send nmea message with checksum. 0 terminated string not including $,*,CHK,CRLF
void nmea_sendmessage(LEUART_TypeDef *uart, char *msg)
{
    uint8_t chksum;
    uint8_t c;
    chksum = nmea_generateChecksum(msg);
    while (!(uart->STATUS & LEUART_STATUS_TXBL));
    uart->TXDATA = '$';
    leuart_send_str(uart, msg);
    while (!(uart->STATUS & LEUART_STATUS_TXBL));
    uart->TXDATA = '*';
    while (!(uart->STATUS & LEUART_STATUS_TXBL));
    if (((chksum&0xF0)>>4)>9) {
        uart->TXDATA = ((chksum&0xF0)>>4) - 10 + 'A';
    } else {
        uart->TXDATA = ((chksum&0xF0)>>4) + '0';
    }
    while (!(uart->STATUS & LEUART_STATUS_TXBL));
    if (((chksum&0x0F))>9) {
        uart->TXDATA = ((chksum&0x0F)) - 10 + 'A';
    } else {
        uart->TXDATA = ((chksum&0x0F)) + '0';
    }
    while (!(uart->STATUS & LEUART_STATUS_TXBL));
    uart->TXDATA = '\r';
    while (!(uart->STATUS & LEUART_STATUS_TXBL));
    uart->TXDATA = '\n';
}

void uart_setbaud(USART_TypeDef *uart, uint32_t baud)
{
    static const uint32_t ovs = 16;
    static const uint32_t refclk = 14060000;
    uart->CMD = 0;
    uart->CLKDIV = (256 * refclk) / ((ovs * baud) - 1);
    uart->CMD = UART_CMD_TXEN | UART_CMD_RXEN;
}

static void uart_init(USART_TypeDef *uart)
{
    static const uint32_t baud = 4800;
    static const uint32_t ovs = 16;
    static const uint32_t refclk = 14060000;

    uart->CLKDIV = (256 * refclk) / ((ovs * baud) - 1);
    uart->CMD = UART_CMD_TXEN | UART_CMD_RXEN;
    uart->IEN = UART_IEN_RXDATAV;
}

static void leuart_init(LEUART_TypeDef *uart)
{
    static const uint32_t baud = 4800;
    static const uint32_t ovs = 16;
    static const uint32_t refclk = 37000;

    uart->CLKDIV = ((256 * refclk) / (baud)) - 256;
    //uart->CLKDIV = 1488;
    uart->CMD = LEUART_CMD_TXEN | LEUART_CMD_RXEN;
    uart->IEN = LEUART_IEN_RXDATAV;
}

static void leuart_setbaud(LEUART_TypeDef *uart, uint32_t baud)
{
    //static const uint32_t baud = 4800;
    static const uint32_t ovs = 16;
    static const uint32_t refclk = 37000;

    uart->CMD = 0;
    uart->CLKDIV = ((256 * refclk) / (baud)) - 256;
    //uart->CLKDIV = 1488;
    uart->CMD = LEUART_CMD_TXEN | LEUART_CMD_RXEN;
    //uart->IEN = LEUART_IEN_RXDATAV;
}

void UART0_RX_IRQHandler(void)
{
    while (!(UART1->STATUS & UART_STATUS_TXBL));

    UART1->TXDATA = UART0->RXDATA;
}

void UART1_RX_IRQHandler(void)
{
    while (!(LEUART1->STATUS & LEUART_STATUS_TXBL));
    LEUART1->TXDATA = UART1->RXDATA;
}

void LEUART1_IRQHandler(void)
{
    uint8_t b;
    //LEUART1->IFC = ~_LEUART_IFC_RESETVALUE;
    while (!(UART1->STATUS & UART_STATUS_TXBL));
    b = LEUART1->RXDATA;
    if (b == '$') {
        nmea_len = 0;
    }
    if (nmea_len >= 254) {
        nmea_len = 0;
    }
    nmea_buffer[nmea_len++] = b;
    UART1->TXDATA = b;
    if (nmea_len == 10) {
        nmea_msg_rcvd = 1;
    }
}

int main(int argc, char **argv)
{
    /* Init clock tree */
    int nextRTC;
    CMU->HFCORECLKEN0 = CMU_HFCORECLKEN0_LE;
    CMU->HFPERCLKEN0 = CMU_HFPERCLKEN0_UART0 | CMU_HFPERCLKEN0_UART1
            | CMU_HFPERCLKEN0_GPIO;

    CMU->OSCENCMD = CMU_OSCENCMD_LFRCOEN;

    while (!(CMU->STATUS & CMU_STATUS_LFRCORDY));

    CMU->OSCENCMD |= CMU_OSCENCMD_LFXOEN;
    /* Wait until LFXO ready */
    /* Note that this could be done more energy friendly with an interrupt in EM1 */
    while (!(CMU->STATUS & CMU_STATUS_LFXORDY)) ;

    CMU->LFCLKSEL = CMU_LFCLKSEL_LFA_LFRCO | CMU_LFCLKSEL_LFB_LFRCO;
    CMU->LFACLKEN0 = CMU_LFACLKEN0_RTC;
    CMU->LFBCLKEN0 = CMU_LFBCLKEN0_LEUART1;

    /* Configure external pins */

#if 1
    GPIO->P[0].DOUT |= (1 << 3);
    GPIO->P[0].DOUT |= (1 << 1);
    GPIO->P[0].DOUT |= (1 << 0);
    GPIO->P[0].MODEL = 0; // LEDB, CTS_N
#endif

    GPIO->P[0].MODEL |= 
              GPIO_P_MODEL_MODE0_WIREDAND
            | GPIO_P_MODEL_MODE1_WIREDAND
            | GPIO_P_MODEL_MODE3_WIREDAND
            | GPIO_P_MODEL_MODE5_PUSHPULL
            | GPIO_P_MODEL_MODE6_INPUT;
    GPIO->P[0].MODEH =
              GPIO_P_MODEH_MODE14_WIREDAND;
    GPIO->P[0].DOUT &= ~(1 << 14);

    GPIO->P[4].MODEL =
              GPIO_P_MODEL_MODE2_PUSHPULL
            | GPIO_P_MODEL_MODE3_INPUT;

    GPIO->P[4].MODEH =
              GPIO_P_MODEH_MODE13_PUSHPULL;

    GPIO->P[5].MODEL =
              GPIO_P_MODEL_MODE6_PUSHPULL
            | GPIO_P_MODEL_MODE7_INPUT;

    /* Init UARTs */

    UART0->ROUTE = UART_ROUTE_LOCATION_LOC0 
            | UART_ROUTE_TXPEN | UART_ROUTE_RXPEN;

    UART1->ROUTE = UART_ROUTE_LOCATION_LOC3
            | UART_ROUTE_TXPEN | UART_ROUTE_RXPEN;
    
    LEUART1->ROUTE = LEUART_ROUTE_LOCATION_LOC1
            | LEUART_ROUTE_TXPEN | LEUART_ROUTE_RXPEN;

    uart_init(UART0);
    uart_init(UART1);
    leuart_init(LEUART1);

    NVIC_EnableIRQ(UART0_RX_IRQn);
    NVIC_EnableIRQ(UART1_RX_IRQn);
    NVIC_EnableIRQ(LEUART1_IRQn);

    /* Start RTC, wait for 2 sec */
    UART1->TXDATA = '1';
    printf("RTC->CNT: %d\n", RTC->CNT);

    RTC->CTRL = RTC_CTRL_EN;
    while (RTC->CNT < 65536);
    leuart_send_array(LEUART1, sirf_command_NMEA, sizeof(sirf_command_NMEA));
    leuart_send_array(LEUART1, sirf_command_NMEA, sizeof(sirf_command_NMEA));
    leuart_send_array(LEUART1, sirf_command_NMEA, sizeof(sirf_command_NMEA));
    leuart_send_array(LEUART1, sirf_command_NMEA, sizeof(sirf_command_NMEA));
    //uart_setbaud(LEUART1,4800);
    while (RTC->CNT < (65536 + 9*(16000)));
    nmea_sendmessage(LEUART1, "PSRF117,16");
    

    while (RTC->CNT < (65536 + 10*(16000)));

   /* Pulse GPS on/off signal for 500 ms */
    while (!(UART1->STATUS & UART_STATUS_TXBL));
    UART1->TXDATA = '2';
    //printf("RTC->CNT: %d\n", RTC->CNT);
    //nextRTC = RTC->CNT + 8000;
    //printf("RTC->CNT: %d\n", RTC->CNT);
    //printf("RTC->CNT: %d, nextRTC: %d\n", RTC->CNT, nextRTC);
    //while (RTC->CNT < nextRTC);
    //printf("done\n");
    nextRTC = RTC->CNT + 8000;
    GPIO->P[4].DOUT = 1 << 13;
    while (RTC->CNT < nextRTC);
    GPIO->P[4].DOUT = 0;
    while (!(UART1->STATUS & UART_STATUS_TXBL));
    UART1->TXDATA = '3';
    //GPIO->P[0].DOUT = 0;
    //while (RTC->CNT < (65536 + (16000)*15));
//Wakes up in NMEA
//Nmea: Go to SIRF
    //nmea_sendmessage(LEUART1, "PSRF100,0,4800,8,1,0");
    //if the following delay is +1  -> it skips over the next command and remains in sirf
    //while (RTC->CNT < (65536 + (16000)*20));



    //uart_send_array(LEUART1, sirf_command_MPM, sizeof(sirf_command_MPM));
    //leuart_send_array(LEUART1, sirf_command_PTF, sizeof(sirf_command_PTF));
    //while (RTC->CNT < (65536 + (16000)*15));
    
 //Dies when it reaches this comand   
//Sirf: Go to NMEA
    //leuart_send_array(LEUART1, sirf_command_fullPowerMode, sizeof(sirf_command_fullPowerMode));
    //leuart_send_array(LEUART1, sirf_command_NMEA, sizeof(sirf_command_NMEA));
    //leuart_send_array(LEUART1, sirf_command_NMEA, sizeof(sirf_command_NMEA));
    //uart_setbaud(LEUART1,4800);
    // while (RTC->CNT < (65536 + (16000)*25));
    // while (!(UART1->STATUS & UART_STATUS_TXBL));
    // UART1->TXDATA = '4';
    // nmea_sendmessage(LEUART1, "PSRF101,0,0,0,0,0,0,12,4");
    // //nmea_sendmessage(LEUART1, "PSRF100,0,9600,8,1,0");
    // //leuart_setbaud(LEUART1, 9600);

    // while (RTC->CNT < (65536 + (16000)*50));
    // while (!(UART1->STATUS & UART_STATUS_TXBL));
    // UART1->TXDATA = '5';
    // nmea_sendmessage(LEUART1, "PSRF101,0,0,0,0,0,0,12,4");
    
    
    //leuart_send_array(LEUART1, sirf_command_NMEA, sizeof(sirf_command_NMEA));
    //leuart_send_array(LEUART1, sirf_command_NMEA, sizeof(sirf_command_NMEA));
    //leuart_setbaud(LEUART1, 4800);
    //leuart_send_array(LEUART1, sirf_command_NMEA, sizeof(sirf_command_NMEA));

//
//RTC wraps around - 24 bits = 256 * 65536
//
//    



//this doesn't work !!
  //  while (RTC->CNT < (65536 + (16000)*20));
    //leuart_send_array(LEUART1, sirf_command_NMEA, sizeof(sirf_command_NMEA));

  //  while (RTC->CNT < (65536 + (16000)*25));
    //nmea_sendmessage(LEUART1, "PSRF100,0,4800,8,1,0");

   // while (RTC->CNT < (65536 + (16000)*30));
    //leuart_send_array(LEUART1, sirf_command_NMEA, sizeof(sirf_command_NMEA));


    /* Shallow-sleep forever */

    for (;;) {
        if (nmea_msg_rcvd) {
            GPIO->P[0].DOUT ^= (1 << 3);
            if((nmea_buffer[4] == 'S') && (nmea_buffer[5] == 'A')) {
                if (nmea_buffer[9] == '1') {
                    GPIO->P[0].DOUT ^= (1 << 0);
                    GPIO->P[0].DOUT |= (1 << 1);
                } else {
                    GPIO->P[0].DOUT |= (1 << 0);
                    GPIO->P[0].DOUT ^= (1 << 1);
                    valid_fix_count++;
                }
                
            }
            nmea_msg_rcvd = 0;
            //if(valid_fix_count > 10) {
            //    valid_fix_count = 0;
            //    nmea_sendmessage(LEUART1, "PSRF101,0,0,0,0,0,0,12,4");
            //}
        }

        //while (!(UART1->STATUS & UART_STATUS_TXBL));
        //UART1->TXDATA = ((GPIO->P[0].DIN & (1 << 1)) ? '1': '0');
        __WFI();
    }
}

