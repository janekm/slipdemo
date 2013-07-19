

#define RADIO_CONNECTED 1
#define RADIO_DISCONNECTED 0

#define RADIO_BUFFER_LEN 1024

static uint8_t Radio_Buffer[RADIO_BUFFER_LEN];
static uint32_t Radio_Buffer_Start = 0;
static uint32_t Radio_Buffer_Fill = 0;

static int Radio_Transmissions_in_Period = 0;
static int Radio_Period = 0;
#define Radio_Transmissions_in_Period_Max 100

volatile uint32_t msTicks; /* counts 1ms timeTicks */
static volatile int NRF_Interrupt = 0;
static volatile int Radio_State = RADIO_CONNECTED;
static volatile int MMA_Capture = 0;
static volatile int NRF_Transmit = 0;
static volatile int Packets_ToTransmit = 0;
static volatile int SeqNum = 0;
static volatile int Radio_Channel = 0;
static volatile int battery_voltage = 0;
static volatile int temperature = 0;
static volatile int temperatureFloat = 0;

static volatile int radio_force_irq = 0;

#define FLASH_BUFFER_SIZE 8388608

static uint32_t Flash_Buffer_Start = 0;
static uint32_t Flash_Buffer_Fill = 0;
#define FLASH_SECTORS (FLASH_BUFFER_SIZE >> 12)
static uint32_t Flash_Sector_Erased = 0;
static volatile int Flash_On = 0;

uint16_t manid = 0;

void Flash_Add(uint8_t *data) {
  if ((Flash_Buffer_Fill + 256) <= FLASH_BUFFER_SIZE) {
    if (Flash_On == 0) {
      SST25_LDO_ON();
      for(volatile int c = 0; c < 2000; c++) {}
      Flash_On = 1;
    }
    MX25_RDP();
    MX25_BusyLoop();
    if (((((Flash_Buffer_Start + Flash_Buffer_Fill) + 256)% FLASH_BUFFER_SIZE)>> 12)%FLASH_SECTORS  == (Flash_Sector_Erased + 1)%FLASH_SECTORS ) {
      Flash_Sector_Erased++;
      MX25_BusyLoop();
      MX25_WREN();
      MX25_SectorErase(Flash_Sector_Erased << 12);
      MX25_BusyLoop();
    }
    
    MX25_WREN();
    MX25_ProgramPage((Flash_Buffer_Start + Flash_Buffer_Fill) % FLASH_BUFFER_SIZE, data);
    MX25_BusyLoop();
    Flash_Buffer_Fill += 256;
    #ifdef USE_FLASH
    if (Flash_On == 1) {
      if (((((Flash_Buffer_Start + Flash_Buffer_Fill) + 256)% FLASH_BUFFER_SIZE)>> 12)%FLASH_SECTORS  == (Flash_Sector_Erased + 1)%FLASH_SECTORS ) {
        Flash_Sector_Erased++;
        MX25_BusyLoop();
        MX25_WREN();
        MX25_SectorErase(Flash_Sector_Erased << 12);
      }
      MX25_BusyLoop();
    }
    #endif // USE_FLASH    

  }
}

void Flash_Dequeue(uint8_t *data) {
  if (Flash_On == 0) {
    SST25_LDO_ON();
    for(volatile int c = 0; c < 2000; c++) {}
    Flash_On = 1;
  }
  MX25_RDP();
  MX25_BusyLoop();
  MX25_Read(Flash_Buffer_Start, 256, data);
  Flash_Buffer_Start = (Flash_Buffer_Start + 256) % FLASH_BUFFER_SIZE;
  Flash_Buffer_Fill -= 256;
}

#define OUTPUT_BUFFER_SIZE 512

static uint8_t Output_Buffer[OUTPUT_BUFFER_SIZE];
static uint32_t Output_Buffer_Start = 0;
static uint32_t Output_Buffer_Fill = 0;

void OutputBuffer_Refill(void)
{
  if(Output_Buffer_Fill == 0) {
#ifdef USE_FLASH
    if(Flash_Buffer_Fill >=256) {
#else
    if (false) {
#endif
      Flash_Dequeue(&Output_Buffer[0]);
      Output_Buffer_Start = 0;
      Output_Buffer_Fill = 256;
    } else if (Radio_Buffer_Fill >= 256) {
      Output_Buffer_Start = 0;
      Output_Buffer_Fill = 0;
      for (int i = 0; i < 256; i++) {
        Output_Buffer[(Output_Buffer_Start + Output_Buffer_Fill + i)%OUTPUT_BUFFER_SIZE] = Radio_Buffer[(Radio_Buffer_Start + i) % RADIO_BUFFER_LEN];
      }
      Output_Buffer_Fill = 256;
      Radio_Buffer_Start = (Radio_Buffer_Start + 256) % RADIO_BUFFER_LEN;
      Radio_Buffer_Fill -= 256;
    }  
  }
}

void OutputBuffer_Dequeue(void)
{
  if (Output_Buffer_Fill >=32) {
    Output_Buffer_Start = ((Output_Buffer_Start + 32)%OUTPUT_BUFFER_SIZE);
    Output_Buffer_Fill -= 32;
  }
}


int RadioBufferAdd(int len, uint8_t* data) {
  int i;
  if ((Radio_Buffer_Fill + len) > RADIO_BUFFER_LEN) {
    Radio_Buffer_Start = 0;
    Radio_Buffer_Fill = 0;
    return -1;
  } else {
    for (i = 0; i < len; i++) {
      Radio_Buffer[(Radio_Buffer_Start + Radio_Buffer_Fill + i)%RADIO_BUFFER_LEN] = data[i];
    }
    //Radio_Buffer_Start = ((Radio_Buffer_Start + len)%RADIO_BUFFER_LEN);
    Radio_Buffer_Fill += len;
  }
  //OutputBuffer_Refill();
#ifdef USE_FLASH
  if (Radio_Buffer_Fill >= (RADIO_BUFFER_LEN - 64)) {
    while (Radio_Buffer_Fill >= 256) {
      Flash_Add(&Radio_Buffer[(Radio_Buffer_Start) % RADIO_BUFFER_LEN]);
      Radio_Buffer_Start = ((Radio_Buffer_Start + 256)%RADIO_BUFFER_LEN);
      Radio_Buffer_Fill -= 256;
    }
  }
#endif
  return len;
}

void RadioBufferTransmit(void) {
//   if (Radio_Buffer_Fill > 0) {
//     NRF_TransmitPacket(32, &Radio_Buffer[Radio_Buffer_Start]);
//   }
  OutputBuffer_Refill();
  if (Output_Buffer_Fill >= 32) {
    //NRF_PowerUp();
      //NRF_WriteRegister(NRF_STATUS, 0x7E); // Clear Interrupts
    //NRF_WriteRegister(NRF_CONFIG, 0x0E); // Power Up, Transmitter
   // NRF_SendCommand(0xE1, 0xFF);
    NRF_TransmitPacket(32, &Output_Buffer[Output_Buffer_Start]);
    NRF_CE_hi;
  }   
}

void RadioBufferDequeue(void) {
//   if (Radio_Buffer_Fill >= 32) {
//     Radio_Buffer_Start = ((Radio_Buffer_Start + 32)%RADIO_BUFFER_LEN);
//     Radio_Buffer_Fill -= 32;
//   }
  if (Output_Buffer_Fill >= 32) {
    Output_Buffer_Start = (Output_Buffer_Start + 32) % OUTPUT_BUFFER_SIZE;
    Output_Buffer_Fill -= 32;
  }
}
