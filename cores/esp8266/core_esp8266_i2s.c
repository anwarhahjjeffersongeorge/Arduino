/*
  i2s.c - Software I2S library for esp8266

  Code taken and reworked from espessif's I2S example

  Copyright (c) 2015 Hristo Gochkov. All rights reserved.
  This file is part of the esp8266 core for Arduino environment.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#include "Arduino.h"
#include "osapi.h"
#include "ets_sys.h"

#include "i2s_reg.h"
#include "i2s.h"

extern void ets_wdt_enable(void);
extern void ets_wdt_disable(void);

#define SLC_BUF_CNT (8) //Number of buffers in the I2S circular buffer
#define SLC_BUF_LEN (64) //Length of one buffer, in 32-bit words.
#define RX_NUM (128) //unit word

//We use a queue to keep track of the DMA buffers that are empty. The ISR will push buffers to the back of the queue,
//the mp3 decode will pull them from the front and fill them. For ease, the queue will contain *pointers* to the DMA
//buffers, not the data itself. The queue depth is one smaller than the amount of buffers we have, because there's
//always a buffer that is being used by the DMA subsystem *right now* and we don't want to be able to write to that
//simultaneously.

struct slc_queue_item {
  uint32  blocksize:12;
  uint32  datalen:12;
  uint32  unused:5;
  uint32  sub_sof:1;
  uint32  eof:1;
  uint32  owner:1;
  uint32  buf_ptr;
  uint32  next_link_ptr;
};

static uint32_t i2s_slc_rx_queue[SLC_BUF_CNT-1];
static uint8_t i2s_slc_rx_queue_len;
static uint32_t *i2s_slc_rx_buf_pntr[SLC_BUF_CNT]; //Pointer to the I2S DMA (rx) buffer data
static struct slc_queue_item i2s_slc_rx_items[SLC_BUF_CNT]; //I2S DMA (rx) buffer descriptors
static uint32_t *i2s_curr_slc_rx_buf=NULL;//current (rx) buffer for writing
static int i2s_curr_slc_rx_buf_pos=0; //position in the current (rx) buffer

static uint32_t i2s_slc_tx_queue[SLC_BUF_CNT-1];
static uint8_t i2s_slc_tx_queue_len;
static uint32_t *i2s_slc_tx_buf_pntr[SLC_BUF_CNT]; //Pointer to the I2S DMA (tx) buffer data
static struct slc_queue_item i2s_slc_tx_items[SLC_BUF_CNT]; //I2S DMA (tx) buffer descriptors


static uint32_t *i2s_curr_slc_tx_buf=NULL;//current (tx) buffer for writing
static int i2s_curr_slc_tx_buf_pos=0; //position in the current buffer (tx)

bool ICACHE_FLASH_ATTR i2s_is_full(){
  return (i2s_curr_slc_rx_buf_pos==SLC_BUF_LEN || i2s_curr_slc_rx_buf==NULL) && (i2s_slc_rx_queue_len == 0);
}

bool ICACHE_FLASH_ATTR i2s_is_empty(){
  return (i2s_slc_rx_queue_len >= SLC_BUF_CNT-1);
}

uint32_t ICACHE_FLASH_ATTR i2s_slc_rx_queue_next_item(){ //pop the top off the queue
  uint8_t i;
  uint32_t item = i2s_slc_rx_queue[0];
  i2s_slc_rx_queue_len--;
  for(i=0;i<i2s_slc_rx_queue_len;i++)
    i2s_slc_rx_queue[i] = i2s_slc_rx_queue[i+1];
  return item;
}

uint32_t ICACHE_FLASH_ATTR i2s_slc_tx_queue_next_item(){ //pop the top off the queue
  uint8_t i;
  uint32_t item = i2s_slc_tx_queue[0];
  i2s_slc_tx_queue_len--;
  for(i=0;i<i2s_slc_tx_queue_len;i++)
    i2s_slc_tx_queue[i] = i2s_slc_tx_queue[i+1];
  return item;
}

//This routine is called as soon as the DMA routine has something to tell us. All we
//handle here is the RX_EOF_INT status, which indicate the DMA has sent a buffer whose
//descriptor has the 'EOF' field set to 1.
void ICACHE_FLASH_ATTR i2s_slc_isr(void) {
  uint32_t slc_intr_status = SLCIS;
  SLCIC = 0xFFFFFFFF;

  struct slc_queue_item *finished_item;
  //transmitter side
  if (slc_intr_status & SLCIRXEOF) {
    //clear SLC interrupt
    ETS_SLC_INTR_DISABLE();
    finished_item = (struct slc_queue_item*)SLCRXEDA;
    memset((void *)finished_item->buf_ptr, 0x00, SLC_BUF_LEN * 4);//zero the buffer so it is mute in case of underflow
    if (i2s_slc_rx_queue_len >= SLC_BUF_CNT-1) { //All buffers are empty. This means we have an underflow
      i2s_slc_rx_queue_next_item(); //free space for finished_item
    }
    i2s_slc_rx_queue[i2s_slc_rx_queue_len++] = finished_item->buf_ptr;
    //reenable slc interrupt
    ETS_SLC_INTR_ENABLE();
  }

  //receiver side
  if (slc_intr_status & SLCITXEOF) {
    //clear slc interrupt
    ETS_SLC_INTR_DISABLE();
    finished_item = (struct slc_queue_item*)SLCTXEDA; // tx eof des addr
    memset((void *)finished_item->buf_ptr, 0x00, SLC_BUF_LEN * 4);//zero the buffer so it is mute in case of underflow
    if (i2s_slc_tx_queue_len >= SLC_BUF_CNT-1) { //All buffers are empty. This means we have an underflow
      i2s_slc_tx_queue_next_item(); //free space for finished_item
    }
    i2s_slc_tx_queue[i2s_slc_tx_queue_len++] = finished_item->buf_ptr;
    //reenable slc Interrupt
    ETS_SLC_INTR_ENABLE();
  }
}

void ICACHE_FLASH_ATTR i2s_slc_begin(){
  i2s_slc_rx_queue_len = 0;
  i2s_slc_tx_queue_len = 0;

  int x, y;

  //set up slc rx queue
 if(i2sOut==true)
  {
    for (x=0; x<SLC_BUF_CNT; x++) {
      i2s_slc_rx_buf_pntr[x] = malloc(SLC_BUF_LEN*4);
      for (y=0; y<SLC_BUF_LEN; y++) i2s_slc_rx_buf_pntr[x][y] = 0;

      i2s_slc_rx_items[x].unused = 0;
      i2s_slc_rx_items[x].owner = 1;
      i2s_slc_rx_items[x].eof = 1;
      i2s_slc_rx_items[x].sub_sof = 0;
      i2s_slc_rx_items[x].datalen = SLC_BUF_LEN*4;
      i2s_slc_rx_items[x].blocksize = SLC_BUF_LEN*4;
      i2s_slc_rx_items[x].buf_ptr = (uint32_t)&i2s_slc_rx_buf_pntr[x][0];
      i2s_slc_rx_items[x].next_link_ptr = (int)((x<(SLC_BUF_CNT-1))?(&i2s_slc_rx_items[x+1]):(&i2s_slc_rx_items[0]));
    }
  }

  //set up slc tx queue

  if(i2sIn == true){
   for (x=0; x<SLC_BUF_CNT; x++) {
      i2s_slc_tx_buf_pntr[x] = malloc(SLC_BUF_LEN*4);
      for (y=0; y<SLC_BUF_LEN; y++) i2s_slc_tx_buf_pntr[x][y] = 0;

      i2s_slc_tx_items[x].unused = 0;
      i2s_slc_tx_items[x].owner = 1;
      i2s_slc_tx_items[x].eof = 1;
      i2s_slc_tx_items[x].sub_sof = 0;
      i2s_slc_tx_items[x].datalen = SLC_BUF_LEN*4;
      i2s_slc_tx_items[x].blocksize = SLC_BUF_LEN*4;
      i2s_slc_tx_items[x].buf_ptr = (uint32_t)&i2s_slc_tx_buf_pntr[x][0];
      i2s_slc_tx_items[x].next_link_ptr = (int)((x<(SLC_BUF_CNT-1))?(&i2s_slc_tx_items[x+1]):(&i2s_slc_tx_items[0]));
    }
  }

  ETS_SLC_INTR_DISABLE();
  SLCC0 |= SLCRXLR | SLCTXLR;
  SLCC0 &= ~(SLCRXLR | SLCTXLR);
  SLCIC = 0xFFFFFFFF;

  //Configure DMA
  SLCC0 &= ~(SLCMM << SLCM); //clear DMA MODE
  SLCC0 |= (1 << SLCM); //set DMA MODE to 1
  SLCRXDC |= SLCBINR | SLCBTNR; //enable INFOR_NO_REPLACE and TOKEN_NO_REPLACE
  SLCRXDC &= ~(SLCBRXFE | SLCBRXEM | SLCBRXFM); //disable RX_FILL, RX_EOF_MODE and RX_FILL_MODE

  //Feed DMA the 1st buffer desc addr
  //To send data to the I2S subsystem, counter-intuitively we use the RXLINK part, not the TXLINK as you might
  //expect. The TXLINK part still needs a valid DMA descriptor, even if it's unused: the DMA engine will throw
  //an error at us otherwise. Just feed it any random descriptor.
  SLCTXL &= ~(SLCTXLAM << SLCTXLA); // clear TX descriptor address
  SLCTXL |= (uint32)&i2s_slc_tx_items[0] << SLCTXLA; //set TX descriptor address. (any random desc is OK when we don't use TX but it needs to be valid)
  SLCRXL &= ~(SLCRXLAM << SLCRXLA); // clear RX descriptor address
  SLCRXL |= (uint32)&i2s_slc_rx_items[0] << SLCRXLA; //set RX descriptor address

  ETS_SLC_INTR_ATTACH(i2s_slc_isr, NULL);
  SLCIE = SLCIRXEOF; //Enable only for RX EOF interrupt

  ETS_SLC_INTR_ENABLE();

  //Start transmission
  SLCTXL |= SLCTXLS;
  SLCRXL |= SLCRXLS;
}

void ICACHE_FLASH_ATTR i2s_slc_end(){
  ETS_SLC_INTR_DISABLE();
  SLCIC = 0xFFFFFFFF;
  SLCIE = 0;
  SLCTXL &= ~(SLCTXLAM << SLCTXLA); // clear TX descriptor address
  SLCRXL &= ~(SLCRXLAM << SLCRXLA); // clear RX descriptor address
}

// //this function reads a 32-bit sample from the I2S buffer
bool ICACHE_FLASH_ATTR i2s_read_sample(uint32_t* sample){
  if (i2s_curr_slc_tx_buf_pos==SLC_BUF_LEN || i2s_curr_slc_tx_buf==NULL) {
    if(i2s_slc_tx_queue_len == 0){
      while(1){
        if(i2s_slc_tx_queue_len > 0){
          break;
        } else {
          ets_wdt_disable();
          ets_wdt_enable();
        }
      }
    }
    ETS_SLC_INTR_DISABLE();
    i2s_curr_slc_tx_buf = (uint32_t *)i2s_slc_tx_queue_next_item();
    ETS_SLC_INTR_ENABLE();
    i2s_curr_slc_tx_buf_pos=0;
  }
  *sample = i2s_curr_slc_tx_buf[i2s_curr_slc_tx_buf_pos++];
  return true;
}

bool ICACHE_FLASH_ATTR i2s_read_sample_nb(uint32_t* sample) {
  if (i2s_curr_slc_tx_buf_pos==SLC_BUF_LEN || i2s_curr_slc_tx_buf==NULL) {
    if(i2s_slc_tx_queue_len == 0){
      return false;
    }
    ETS_SLC_INTR_DISABLE();
    i2s_curr_slc_tx_buf = (uint32_t *)i2s_slc_tx_queue_next_item();
    ETS_SLC_INTR_ENABLE();
    i2s_curr_slc_tx_buf_pos=0;
  }
  *sample = i2s_curr_slc_tx_buf[i2s_curr_slc_tx_buf_pos++];
  return true;
}

bool ICACHE_FLASH_ATTR i2s_read_lr(int16_t* left, int16_t* right){
  bool success = false;
  int sample;
  success = i2s_read_sample(&sample);
  if(success){
    *right = (uint16_t) sample & 0xFFFF;
    *left = (uint16_t) (sample >> 16) & 0xFFFF;

  }

  return success;
}

//This routine pushes a single, 32-bit sample to the I2S buffers. Call this at (on average)
//at least the current sample rate. You can also call it quicker: it will suspend the calling
//thread if the buffer is full and resume when there's room again.

bool ICACHE_FLASH_ATTR i2s_write_sample(uint32_t sample) {
  if (i2s_curr_slc_rx_buf_pos==SLC_BUF_LEN || i2s_curr_slc_rx_buf==NULL) {
    if(i2s_slc_rx_queue_len == 0){
      while(1){
        if(i2s_slc_rx_queue_len > 0){
          break;
        } else {
          ets_wdt_disable();
          ets_wdt_enable();
        }
      }
    }
    ETS_SLC_INTR_DISABLE();
    i2s_curr_slc_rx_buf = (uint32_t *)i2s_slc_rx_queue_next_item();
    ETS_SLC_INTR_ENABLE();
    i2s_curr_slc_rx_buf_pos=0;
  }
  i2s_curr_slc_rx_buf[i2s_curr_slc_rx_buf_pos++]=sample;
  return true;
}

bool ICACHE_FLASH_ATTR i2s_write_sample_nb(uint32_t sample) {
  if (i2s_curr_slc_rx_buf_pos==SLC_BUF_LEN || i2s_curr_slc_rx_buf==NULL) {
    if(i2s_slc_rx_queue_len == 0){
      return false;
    }
    ETS_SLC_INTR_DISABLE();
    i2s_curr_slc_rx_buf = (uint32_t *)i2s_slc_rx_queue_next_item();
    ETS_SLC_INTR_ENABLE();
    i2s_curr_slc_rx_buf_pos=0;
  }
  i2s_curr_slc_rx_buf[i2s_curr_slc_rx_buf_pos++]=sample;
  return true;
}

bool ICACHE_FLASH_ATTR i2s_write_lr(int16_t left, int16_t right){
  int sample = right & 0xFFFF;
  sample = sample << 16;
  sample |= left & 0xFFFF;
  return i2s_write_sample(sample);
}

//  END DMA
// =========
// START I2S


static uint32_t _i2s_sample_rate;

void ICACHE_FLASH_ATTR i2s_set_rate(uint32_t rate, bool txSlave, bool rxSlave){ //Rate in HZ
  if(rate == _i2s_sample_rate) return;
  _i2s_sample_rate = rate;
  uint32_t i2s_clock_div = (I2SBASEFREQ/(_i2s_sample_rate*32)) & I2SCDM;
  uint8_t i2s_bck_div = (I2SBASEFREQ/(_i2s_sample_rate*i2s_clock_div*2)) & I2SBDM;
  //os_printf("Rate %u Div %u Bck %u Frq %u\n", _i2s_sample_rate, i2s_clock_div, i2s_bck_div, I2SBASEFREQ/(i2s_clock_div*i2s_bck_div*2));

  //I2SCONF: clear I2S_BCK_DIV_NUM, clear I2S_CLKM_DIV_NUM, !I2S_BITS_MOD;
  I2SC &= ~((I2SBDM << I2SBD) | (I2SCDM << I2SCD) | (I2SBMM << I2SBM));

  i2sRxSlave = rxSlave;
  i2sTxSlave = txSlave;
  uint8_t sm;

  //tx
  if(i2sOut == true){
    //I2SCONF: clear I2S_TRANS_SLAVE_MOD
    I2SC &= ~(I2STSM);
    //I2SCONF: set I2S_TRANS_SLAVE_MOD ???
    sm = (i2sTxSlave) ? I2STSM : 0;
    I2SC |= sm;
  }


  //rx
  if(i2sIn == true){
    //I2SCONF: clear I2S_RECE_SLAVE_MOD,
    I2SC &= ~(I2SRSM);
    //I2SCONF: set I2S_RECE_SLAVE_MODE ????
    sm =  (rxSlave) ? I2SRSM : 0;
    I2SC |= sm;
  }


  //I2SCONF: I2S_RIGHT_FIRST, I2S_MSB_RIGHT, I2S_RECE_MSB_SHIFT, I2S_TRANS_MSB_SHIFT
  I2SC |=  I2SRF | I2SMR | I2SRMS | I2STMS;
  //Use I2S clock divider to produce appropriate sample rate
  I2SC |= ((i2s_bck_div-1) << I2SBD) | ((i2s_clock_div-1) << I2SCD);

  // //!trans master, !bits mod, rece slave mod, rece msb shift, right first, msb right
  // I2SC &= ~(I2STSM | (I2SBMM << I2SBM) | (I2SBDM << I2SBD) | (I2SCDM << I2SCD));
  // I2SC |= I2SRF | I2SMR | I2SRSM | I2SRMS | ((i2s_bck_div-1) << I2SBD) | ((i2s_clock_div-1) << I2SCD);
}


void ICACHE_FLASH_ATTR i2s_begin(uint32_t rate, bool in, bool out, bool txSlave, bool rxSlave, /*bool rxOutClock,*/ uint8_t txFifoMode, uint8_t rxFifoMode, uint8_t txChanMode, uint8_t rxChanMode){
  _i2s_sample_rate = 0;
  i2s_slc_begin();

  //if told neither to do I2S in or out, default to doing at least one (out)
  i2sOut = (in == false && out == false) ? true : out;
  i2sIn = in;

  //set i2s pins based on what we're doing
  //see: http://www.esp8266.com/wiki/doku.php?id=esp8266_gpio_pin_allocations
  if(i2sOut == true){
    pinMode(2, FUNCTION_1); //I2SO_WS (LRCK)
    pinMode(3, FUNCTION_1); //I2SO_DATA (SDIN)
    pinMode(15, FUNCTION_1); //I2SO_BCK (SCLK)
  }
  if(i2sIn == true){
    pinMode(14, FUNCTION_1); //I2SI_WS (LRCK)
    pinMode(12, FUNCTION_1); //I2SI_DATA (SDIN)
    pinMode(13, FUNCTION_1); //I2SI_BCK (SCLK)
  }

  //enable 160MHz Clock to I2S subsystem?
  I2S_CLK_ENABLE();
  I2SIC = 0x3F;
  I2SIE = 0;

  //Reset I2S
  I2SC &= ~(I2SRST);
  I2SC |= I2SRST;
  I2SC &= ~(I2SRST);

  //decide what tx and rx fifo modes to set or defaults
  i2sOutFifoMode = (txFifoMode > 5) ? I2STXFIFO_16FULL : txFifoMode; //(6-7) are invalid
  i2sInFifoMode = (rxFifoMode > 3) ? I2SRXFIFO_16FULL : rxFifoMode; //(4-7) are invalid
  i2sOutChanMode = (txChanMode > 4) ? I2STXCHAN_DUAL : txChanMode;
  i2sInChanMode = (rxChanMode > 2) ? I2SRXCHAN_DUAL : rxChanMode;

  //fifo, chan modes

  I2SFC &= ~(I2SDE | (I2STXFMM << I2STXFM) | (I2SRXFMM << I2SRXFM)); //Set RX/TX FIFO_MOD=0 and disable DMA (FIFO only)
  I2SFC |= (i2sOutFifoMode << I2STXFM) | (i2sInFifoMode << I2SRXFM); //set RX/TX FIFO_MOD to user value
  I2SFC |= I2SDE; //Enable DMA
  I2SCC &= ~((I2STXCMM << I2STXCM) | (I2SRXCMM << I2SRXCM)); //Set RX/TX CHAN_MOD=0
  I2SCC |= (i2sOutChanMode << I2STXCM) | (i2sInChanMode << I2SRXCM);

  //set RX eof num
  I2SRXEN &= 0;
  I2SRXEN != RX_NUM;

  i2s_set_rate(rate, txSlave, rxSlave);

  // i2sRXOutputClock = rxOutClock;

  if(i2sOut == true /*&& i2sTxSlave == false*/){
    I2SC |= I2STXS; //Start TX transmission
  }
  if(i2sIn == true){
    I2SC |= I2SRXS; //Start RX transmission
  }
}

void ICACHE_FLASH_ATTR i2s_end(){
  i2s_slc_end();
  if(i2sOut == true){
    pinMode(2, INPUT);
    pinMode(3, INPUT);
    pinMode(15, INPUT);
  }
  if(i2sIn == true){
    pinMode(14, INPUT);
    pinMode(12, INPUT);
    pinMode(13, INPUT);
  }
}
