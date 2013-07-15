#include <efm32.h>
#include <stdio.h>

#include <em_rtc.h>

#define FALSE 0
#define TRUE 1
typedef int bool;

const static uint8_t sirf_command_NMEA2[] = {0xA0, 0xA2, 0x00, 0x09, 0x97, 0x00, 0x01, 0x00, 0xC8, 0x00, 0x00, 0x00, 0xC8, 0x02, 0x28, 0xB0, 0xB3 };
const static uint8_t sirf_command_NMEA[] =  {0xA0, 0xA2, 0x00, 0x18, 0x81, 0x02, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x12, 0xC0, 0x01, 0x60, 0xB0, 0xB3};
const static uint8_t sirf_command_MPM[] = {0xA0, 0xA2, 0x00, 0x06, 0xDA, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDC, 0xB0, 0xB3};
const static uint8_t sirf_command_PTF[] = {0xA0, 0xA2, 0x00, 0x0E, 0xDA, 0x04, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 0x0F, 0xA0, 0x00, 0x02, 0xBF ,0x20, 0x02, 0xE6, 0xB0, 0xB3};
const static uint8_t sirf_command_fullPowerMode = {0xA0, 0xA2, 0x00, 0x02, 0xDA, 0x00, 0x00, 0xDA, 0xB0, 0xB3};
// this returns a single binary byte that is the checksum 

uint8_t nmea_buffer[256];
uint32_t nmea_len = 0;
uint32_t nmea_msg_rcvd = 0;
uint32_t valid_fix_count = 0;


typedef struct {
    uint8_t tm_hour;
    uint8_t tm_min;
    uint8_t tm_sec;
    uint8_t tm_mday;
    uint8_t tm_mon;
    uint8_t tm_year;
    uint8_t tm_isdst;
} timem_t;

void NmeaEvent_position(int16_t latitude, int16_t longitude, uint8_t n_sats) {

}

void NmeaEvent_time(timem_t* time) {
    printf("Hour: %d, Minute: %d, Second: %d\n", time->tm_hour, time->tm_min, time->tm_sec);
}

char *toke(char **pos_param) {
    char *pos;
    char *token;

    pos = *pos_param;
    token = pos;

    while (*pos != '\0' && *pos != ',' && *pos != '*') {
        pos++;
    }

    if (*pos != '\0') {
        *pos = '\0';
        pos++;
    }

    *pos_param = pos;

    return token;
}

bool fixedPoint(char *pos, int *result) {
    int val;

    val = 0;

    if (*pos == '\0') {
        return FALSE;
    }

    while (*pos) {
        if (*pos >= '0' && *pos <= '9') {
            val = (10 * val) + (*pos - '0');
        } else if (*pos != '.') {
            return FALSE;
        }

        pos++;
    }

    *result = val;

    return TRUE;
}

bool fixedWidthInt(char **pos_param, int ndigits, int *result) {
    int i;
    char *pos;
    int val;

    pos = *pos_param;
    val = 0;

    for (i = 0 ; i < ndigits ; i++) {
        if (*pos >= '0' && *pos <= '9') {
            val = (val * 10) + (*pos - '0');
        } else {
            return FALSE;
        }

        pos++;
    }

    *pos_param = pos;
    *result = val;

    return TRUE;
}

void parsePosition(char *str) {
    int latitude;
    int longitude;
    int nsats;
    char *pos;

    latitude = 0;
    longitude = 0;

    // UTC time (don't care)
    //printf("Parse Position\n");

    toke(&str);

    // Latitude. Lack of fix will give empty lat/long field values, and
    // any such position packets will be discarded.

    pos = toke(&str);

    if (!fixedPoint(pos, &latitude)) {
        printf("No valid latitude\n");
        return;
    }

    // North/South. I wonder if this code will ever run anywhere other than
    // the NW quadrant...

    pos = toke(&str);

    if (*pos == 'S') {
        latitude = -latitude;
    } else if (*pos != 'N') {
        return;
    }

    // Longitude

    pos = toke(&str);

    if (!fixedPoint(pos, &longitude)) {
        return;
    }

    // East/West
    
    pos = toke(&str);

    if (*pos == 'W') {
        longitude = -longitude;
    } else if (*pos != 'E') {
        return;
    }

    // Fix indicator. Only accept value of 1

    pos = toke(&str);

    if (*pos != '1') {
        printf("No valid fix\n");
        return;
    }

    // Satellite count

    pos = toke(&str);

    if (!fixedWidthInt(&pos, 2, &nsats)) {
        return;
    }

    // Don't care about subsequent fields

    NmeaEvent_position(latitude, longitude, nsats);
}

void parseTime(char *str) {
    char *pos;
    timem_t t;

    // hhmmss
    printf("Parsetime\n");

    pos = toke(&str);

    if (!fixedWidthInt(&pos, 2, &t.tm_hour)
        || !fixedWidthInt(&pos, 2, &t.tm_min)
        || !fixedWidthInt(&pos, 2, &t.tm_sec)) {
            return;
    }

    // Day

    pos = toke(&str);

    if (!fixedWidthInt(&pos, 2, &t.tm_mday)) {
        return;
    }

    // Month

    pos = toke(&str);

    if (!fixedWidthInt(&pos, 2, &t.tm_mon)) {
        return;
    }

    // Year

    pos = toke(&str);

    if (!fixedWidthInt(&pos, 4, &t.tm_year)) {
        return;
    }

    t.tm_isdst = 0;

    NmeaEvent_time(&t);
}

void NmeaReceive(char *str, size_t len) {
    char *type;

    type = toke(&str);
    //printf("type: %s\n", type);

    if (strcmp(type, "$GPGGA") == 0) {
        parsePosition(str);
    } else if (strcmp(type, "$GPZDA") == 0) {
        parseTime(str);
    }
}


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

int nmea_len2 = 0;

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
    //UART1->TXDATA = b;
    if ((b == 0x0A) && (nmea_len >= 10)) {
    //if (nmea_len == 10) {
        nmea_msg_rcvd = 1;
        nmea_len2 = nmea_len;
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
    //printf("RTC->CNT: %d\n\n\n", RTC->CNT);

    RTC->CNT = 0;
    RTC->CTRL = RTC_CTRL_EN;
    while (RTC->CNT < 65536);
    //uart_setbaud(LEUART1,9600);
    leuart_send_array(LEUART1, sirf_command_NMEA, sizeof(sirf_command_NMEA));
    leuart_send_array(LEUART1, sirf_command_NMEA, sizeof(sirf_command_NMEA));
    leuart_send_array(LEUART1, sirf_command_NMEA, sizeof(sirf_command_NMEA));
    leuart_send_array(LEUART1, sirf_command_NMEA, sizeof(sirf_command_NMEA));
    //uart_setbaud(LEUART1,4800);
    while (RTC->CNT < (65536 + 9*(16000)));
    nmea_sendmessage(LEUART1, "PSRF117,16");
    //printf("RTC->CNT: %d\n\n\n", RTC->CNT);
    

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
    while (RTC->CNT < nextRTC);
    GPIO->P[4].DOUT = 1 << 13;
    nextRTC = RTC->CNT + 8000;
    while (RTC->CNT < nextRTC);
    GPIO->P[4].DOUT = 0;
    while (!(UART1->STATUS & UART_STATUS_TXBL));
    UART1->TXDATA = '3';
    leuart_send_array(LEUART1, sirf_command_NMEA, sizeof(sirf_command_NMEA));
    nextRTC = RTC->CNT + 8000;
    while (RTC->CNT < nextRTC);
    printf("RTC->CNT: %d\n\n\n", RTC->CNT);
    //while(1);
    //GPIO->P[0].DOUT = 0;
    //while (RTC->CNT < (65536 + (16000)*15));
//Wakes up in NMEA
//Nmea: Go to SIRF
    nmea_sendmessage(LEUART1, "PSRF100,0,4800,8,1,0");
    nextRTC = RTC->CNT + 16000;
    while (RTC->CNT < nextRTC);
    leuart_send_array(LEUART1, sirf_command_MPM, sizeof(sirf_command_MPM));
    //leuart_send_array(LEUART1, sirf_command_NMEA, sizeof(sirf_command_NMEA));
    nextRTC = RTC->CNT + 20000;
    while (RTC->CNT < nextRTC);
    GPIO->P[4].DOUT = 1 << 13;
    nextRTC = RTC->CNT + 8000;
    while (RTC->CNT < nextRTC);
    GPIO->P[4].DOUT = 0;
    nextRTC = RTC->CNT + 20000;
    while (RTC->CNT < nextRTC);
    leuart_send_array(LEUART1, sirf_command_NMEA, sizeof(sirf_command_NMEA));
    
    nextRTC = RTC->CNT + 16000;
    while (RTC->CNT < nextRTC);
    nmea_sendmessage(LEUART1, "PSRF103,3,0,5,1");
    nmea_sendmessage(LEUART1, "PSRF103,0,0,1,1");
    nmea_sendmessage(LEUART1, "PSRF103,2,0,1,1");
    nmea_sendmessage(LEUART1, "PSRF103,8,0,1,1");
    for (;;) {
        if (nmea_msg_rcvd) {
            //printf("Test\n");
            NmeaReceive(nmea_buffer, nmea_len2);
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

