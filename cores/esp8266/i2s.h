/*
  i2s.h - Software I2S library for esp8266

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
#ifndef I2S_h
#define I2S_h

/*
How does this work? Basically, to get sound, you need to:
- Connect an I2S codec to the I2S pins on the ESP.
- Start up a thread that's going to do the sound output
- Call i2s_begin()
- Call i2s_set_rate() with the sample rate you want.
- Generate sound and call i2s_write_sample() with 32-bit samples.
The 32bit samples basically are 2 16-bit signed values (the analog values for
the left and right channel) concatenated as (Rout<<16)+Lout

i2s_write_sample will block when you're sending data too quickly, so you can just
generate and push data as fast as you can and i2s_write_sample will regulate the
speed.
*/

#ifdef __cplusplus
extern "C" {
#endif

//TX FIFO Modes
#define I2STXFIFO_16FULL (0) //16 bits per channel full data (dual channel, FIFO data organisation, 16 bits data in the left channel,16 bits data in the right channel, and 16 bits data in the left channel)
#define I2STXFIFO_16HALF (1) //16 bits per channel half data (single channel, FIFO data organisation, 16 bits data, 16 bits invalid, 16 bits data)
#define I2STXFIFO_24FULL_DISCONTINUE (2) //24 bits per channel full data discontinue (dual channel, FIFO data organisation, 24 bits data in the left channel, 8 bits invalid, 24 bits data in the right channel, 8 bits empty)
#define I2STXFIFO_24HALF_DISCONTINUE (3) //24 bits per channel half data discontinue (single channel, FIFO data organisation, 24 bits data, 8 bits invalid, 24 bits data, 8 bits empty)
#define I2STXFIFO_24FULL_CONTINUE (4) // 24 bits per channel full data continue (left and right channels, FIFO data organisation, 24 bits data in the left channel, 24 bits data in the right channel)
#define I2STXFIFO_24HALF_CONTINUE (5) // 24 bits per channel half data continue (single channel, FIFO data organisation, 24 bits data, 24 bits data)

//RX FIFO Modes
#define I2SRXFIFO_16FULL (0) //16 bits per channel full data
#define I2SRXFIFO_16HALF (1) //16 bits per channel half data
#define I2SRXFIFO_24FULL_DISCONTINUE (2) //24 bits per channel full data discontinue
#define I2SRXFIFO_24HALF_DISCONTINUE (3) // 24 bits per channel half data discontinue

//TX Channel Modes
#define I2STXCHAN_DUAL (0) // dual-channel
#define I2STXCHAN_RIGHT (1) // right channel (left and right audio channels are used to put the data of the right channel)
#define I2STXCHAN_LEFT (2) // left channel (left and right audio channels are used to put the data of the left channel)
#define I2STXCHAN_RIGHT_CONSTLEFT (3) //right channel (put a constant from regfile in the left channel)
#define I2STXCHAN_LEFT_CONSTRIGHT (4) //left channel (put a constant from regfile in the right channel)

//RX Channel Modes
#define I2SRXCHAN_DUAL (0) // dual-channel
#define I2SRXCHAN_RIGHT (1) // right channel
#define I2SRXCHAN_LEFT (2) // left channel

//to let us know whether we are doing I2S in or I2s out
bool i2sIn;
bool i2sOut;
//...whether we are doing master/slave mode on i2s in/out
/*
  TX: Bit 8 in the I2SCONF register is used to start the Tx module.
  * In the master Tx mode:
    - When bit 8 is 1，the Tx mode will output the clock signal,
      the left and right channel signals and data.
      The first frame data is 0, and then the FIFO data will be shifted out.
      -- If no data is written into the the FIFO, the data line will remain 0.
      -- If the FIFO has transported all the written data
         and no new data is written in the FIFO,
         the data line will loop the last data in the FIFO.
  * In the slave passive Tx mode:
    - The Tx module will be started when it receives
      a clock signal from the Rx module.

  RX: Bit 9 in the I2SCONF register is used to start the Rx module.
  * In the master receive mode:
    - When bit 9 is 1，the Rx mode will output the clock signal,
      and sample the data line and the channel selection line.
    - When bit 9 is 0, it will stop the clock signal transport.
  * In the slave receive mode, it is prepared to receive any data from the master.
*/
bool i2sRxSlave;
// bool i2sRXOutputClock; // should the Rx module ouput clock signal in the master receive mode?
bool i2sTxSlave;
//to let us konw what fifo, channel mode we do i2s tx, rx in
uint8_t i2sInFifoMode;
uint8_t i2sOutFifoMode;
uint8_t i2sInChanMode;
uint8_t i2sOutChanMode;

void i2s_begin(uint32_t rate, bool in, bool out, bool txSlave, bool rxSlave, /*bool rxOutClock,*/ uint8_t txFifoMode, uint8_t rxFifoMode, uint8_t txChanMode, uint8_t rxChanMode);
void i2s_end();
void i2s_set_rate(uint32_t rate, bool txSlave, bool rxSlave);//Sample Rate in Hz (ex 44100, 48000)
bool i2s_write_sample(uint32_t sample);//32bit sample with channels being upper and lower 16 bits (blocking when DMA is full)
bool i2s_write_sample_nb(uint32_t sample);//same as above but does not block when DMA is full and returns false instead
bool i2s_write_lr(int16_t left, int16_t right);//combines both channels and calls i2s_write_sample with the result
bool i2s_is_full();//returns true if DMA is full and can not take more bytes (overflow)
bool i2s_is_empty();//returns true if DMA is empty (underflow)

#ifdef __cplusplus
}
#endif

#endif
