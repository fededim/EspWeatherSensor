#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"
#include "soc/sens_reg.h"
#include "soc/rtc_i2c_reg.h"

#include "globaldef.h"
#include "stack.s"


// unluckily bitbanging I2C trashes all registers except r3 which must contain the stack.
.macro i2crd regnum
    push  r2
    move r1,I2C_ADDR
    push r1
    move r1,\regnum
    push r1
    psr
    jump read8
    add r3,r3,2 // remove 2 arguments from stack
    pop r2
.endm

.macro i2cwr regnum,val
    push r2
    move r1,I2C_ADDR
    push r1
    move r1,\regnum
    push r1
    move r1,\val
    push r1
    psr
    jump write8
    add r3,r3,3 // remove 3 arguments from stack
    pop r2
.endm


/* Define variables, which go into .bss section (zero-initialized data) */
    .bss

.global samplecount
samplecount: .long 0      // long is 32bit in ulp

// rtc 48 bit diventano 96, 3 int da 32 (ognuno memorizza i 16 bit)
//pressione 20 bit diventano 64, 2 int da 32 (uno i primi sedici, l'altro i rimanenti 4)
//temperatura 20 bit diventano 64, 2 int da 32 (uno i primi sedici, l'altro i rimanenti 4)
//umidità 16 bit 2 byte diventa 32 bit, 1 int da 32.

.global sdata
sdata: .fill MAX_BME280_SAMPLES*(4*3+4*2+4*2+4*1)  // 3 int for RTC 2 int for pressure 2 int for temperature 1 int for humidity total 32 bytes

// stack
  .global stack
stack:
  .fill 100
  .global stackEnd
stackEnd:
  .long 0

// I2C.s
  
i2c_started:
  .long 0

i2c_didInit:
  .long 0


/* Code goes into .text section */
    .text

    .global entry
entry:   
    // Main code
    move r3,stackEnd

    move R2,sdata
    move R1,samplecount
    ld   R0,R1,0            // r0=m[samplecount]
    jumpr skip,MAX_BME280_SAMPLES,LT
    move  r0,0              // reset counter
    st    r0,r1,0           // m[samplecount]=0
skip:
    lsh  R0,R0,3            // r0=r0<<3, r0=ro*8(sdata offset, 32 bytes x samplecount, = 8 longs x samplecount)
    add  R2,R0,R2           // r2=r0+r2

    // READ RTC 48 bit counter
    WRITE_RTC_FIELD(RTC_CNTL_TIME_UPDATE_REG, RTC_CNTL_TIME_UPDATE, 1)
waitSampleRtc:
    READ_RTC_FIELD(RTC_CNTL_TIME_UPDATE_REG, RTC_CNTL_TIME_VALID)
    jumpr waitSampleRtc, 0, EQ
    READ_RTC_REG(RTC_CNTL_TIME1_REG, 0, 16)      //[47:32]
    st  R0,R2,0             // sdata[0]=r0
    READ_RTC_REG(RTC_CNTL_TIME0_REG, 16, 16)     //[31:16]
    st  R0,R2,4*1           // sdata[1]=r0
    READ_RTC_REG(RTC_CNTL_TIME0_REG, 0, 16)      //[15:0]
    st  R0,R2,4*2           // sdata[2]=r0


    // Read BME280
    i2cwr 0xF2,1           // oversampling x1 in 0xF2 register for humidity
    i2cwr 0xF4,0x25        // oversampling x1 for both pressure and temperature and mode = forced in 0xF4 register 
    i2cwr 0xF5,0           // disables SPI, no IIR filter, 0.5 standby in config register 0xF5

waitSampleBme:
    i2crd  0xF3           // read status register 0xF3
    and    r0,r0,8        // mask bit 3
    jumpr  waitSampleBme,0,GT

    // Read pressure
    i2crd   0xf7          // read reg 0xF7 pressure[19:12] 
    lsh     r1,r0,8       // r1=r0<<8
    and     r1,r1,0xFF00  // mask upper 8 bits, shouldn't be necessary
    push    r1
    
    i2crd   0xf8          // read reg 0xF8 pressure [11:4] 
    pop     r1
    and     r0,r0,0xFF    // mask first 8bit
    or      r1,r0,r1      // r1= r0|r1
    st      r1,r2,4*3     // sdata[3]=pressure [19:12]|pressure[11:4]
    
    i2crd   0xf9          // read reg 0xF9 pressure[3:0] from bit 4 
    rsh     r0,r0,4       // r0=r0>>4
    st      r0, r2, 4*4    // sdata[4]=pressure [3:0]
    
    // Read temp
    i2crd   0xfa          // read reg 0xFA temp [19:12] 
    lsh     r1,r0,8       // r1=r0<<8
    and     r1,r1,0xFF00  // mask upper 8 bits, shouldn't be necessary
    push    r1
    
    i2crd   0xfb          // read reg 0xFB temp [11:4] 
    pop     r1
    and     r0,r0,0xFF    // mask first 8bit
    or      r1,r0,r1      // r1= r0|r1
    st      r1,r2,4*5     // sdata[5]=temp [19:12]|temp[11:4]
    
    i2crd   0xfc          // read reg 0xFc temp[3:0] from bit 4
    rsh     r0,r0,4       // r0=r0>>4
    st      r0, r2, 4*6   // sdata[6]=temp [3:0]


    // Read humidity
    i2crd   0xfd          // read reg 0xFd humidity [15:8] 
    lsh     r1,r0,8       // r1=r0<<8
    and     r1,r1,0xFF00  // mask upper 8 bits
    push    r1
    
    i2crd   0xfe          // read reg 0xFe humidity [7:0] 
    pop     r1
    and     r0,r0,0xFF    // mask first 8bit
    or      r1,r0,r1      // r1= r0|r1
    st      r1,r2,4*7     // sdata[7]=humidity [15:8]|temp[7:0]

    // samplecount++
    move r1,samplecount
    ld   r0,r1,0            // r0=m[samplecount]
    add  r0,r0,1            // r0=r0+1
    st   r0,r1,0            // m[samplecount]=r0
    jumpr exit,MAX_BME280_SAMPLES,lt
    
    // Wake up ESP32
is_rdy_for_wakeup:
    READ_RTC_FIELD(RTC_CNTL_LOW_POWER_ST_REG, RTC_CNTL_RDY_FOR_WAKEUP) // Read RTC_CNTL_RDY_FOR_WAKEUP bit
    AND r0, r0, 1
    JUMP is_rdy_for_wakeup, eq    // Retry until the bit is set
    WAKE                    // Trigger wake up    

.global exit
exit:   /* end the program */
    halt                   // on halt the ulp will start sleeping and it will automatically restart after ULP_SENSOR_PERIOD has passed
    jump    entry          // shouldn't be necessary 


// I2C.s

/*
 * =============================== I2C code ==========================================
 * Implementation of pseudo code from
 * https://en.wikipedia.org/wiki/I%C2%B2C#Example_of_bit-banging_the_I.C2.B2C_master_protocol
 */


.global i2c_start_cond
.global i2c_stop_cond
.global i2c_write_bit
.global i2c_read_bit
.global i2c_write_byte
.global i2c_read_byte

.macro I2C_delay
  wait 10 // 38 // minimal 4.7us
.endm


// TOUCH0 = GPIO4 = SCL = RTC_GPIO10
// TOUCH1 = GPIO0 = SDA = RTC_GPIO11

.macro read_SCL // Return current level of SCL line, 0 or 1
  READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S + 10, 1) // RTC_GPIO_10 == GPIO_04
.endm

.macro read_SDA // Return current level of SDA line, 0 or 1
  READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S + 11, 1) // RTC_GPIO_11 == GPIO_00
.endm

.macro set_SCL // Do not drive SCL (set pin high-impedance)
  WRITE_RTC_REG(RTC_GPIO_ENABLE_W1TC_REG, RTC_GPIO_ENABLE_W1TC_S + 10, 1, 1)
.endm

.macro clear_SCL // Actively drive SCL signal low
  // Output mode
  WRITE_RTC_REG(RTC_GPIO_ENABLE_W1TS_REG, RTC_GPIO_ENABLE_W1TS_S + 10, 1, 1)
.endm

.macro set_SDA // Do not drive SDA (set pin high-impedance)
  WRITE_RTC_REG(RTC_GPIO_ENABLE_W1TC_REG, RTC_GPIO_ENABLE_W1TC_S + 11, 1, 1)
.endm

.macro clear_SDA // Actively drive SDA signal low
  // Output mode
  WRITE_RTC_REG(RTC_GPIO_ENABLE_W1TS_REG, RTC_GPIO_ENABLE_W1TS_S + 11, 1, 1)
.endm


i2c_start_cond:
  move r1,i2c_didInit
  ld r0,r1,0
  jumpr didInit,1,ge
  move r0,1
  st r0,r1,0
// set GPIO to pull low when activated
  WRITE_RTC_REG(RTC_GPIO_OUT_REG, RTC_GPIO_OUT_DATA_S + 10, 1, 0)
  WRITE_RTC_REG(RTC_GPIO_OUT_REG, RTC_GPIO_OUT_DATA_S + 11, 1, 0)
didInit:
  move r2,i2c_started
  ld r0,r2,0
  jumpr not_started,1,lt
// if started, do a restart condition
// set SDA to 1
  set_SDA
  I2C_delay
  set_SCL
clock_stretch: // TODO: Add timeout?
  read_SCL
  jumpr clock_stretch,1,lt

// Repeated start setup time, minimum 4.7us
  I2C_delay

not_started:
  // if (read_SDA() == 0) {
  //    arbitration_lost();
  // }

// SCL is high, set SDA from 1 to 0.
  clear_SDA
  I2C_delay
  clear_SCL
  move r0,1
  st r0,r2,0

  ret


i2c_stop_cond:
// set SDA to 0
  clear_SDA
  I2C_delay

  set_SCL
clock_stretch_stop:
  read_SCL
  jumpr clock_stretch_stop,1,lt

// Stop bit setup time, minimum 4us
  I2C_delay

// SCL is high, set SDA from 0 to 1
  set_SDA
  I2C_delay
  // if (read_SDA() == 0) {
  //    arbitration_lost();
  // }

  move r2,i2c_started
  move r0,0
  st r0,r2,0

  ret


// Write a bit to I2C bus
i2c_write_bit:
  jumpr bit0,1,lt
  set_SDA
  jump bit1
bit0:
  clear_SDA
bit1:

// SDA change propagation delay
  I2C_delay
// Set SCL high to indicate a new valid SDA value is available
  set_SCL
// Wait for SDA value to be read by slave, minimum of 4us for standard mode
  I2C_delay

clock_stretch_write:
  read_SCL
  jumpr clock_stretch_write,1,lt

  // SCL is high, now data is valid
  // If SDA is high, check that nobody else is driving SDA
  // if (bit && (read_SDA() == 0)) {
  //    arbitration_lost();
  // }

  // Clear the SCL to low in preparation for next change
  clear_SCL

  ret


// Read a bit from I2C bus
i2c_read_bit:
// Let the slave drive data
  set_SDA
// Wait for SDA value to be written by slave, minimum of 4us for standard mode
  I2C_delay
// Set SCL high to indicate a new valid SDA value is available
  set_SCL

clock_stretch_read:
  read_SCL
  jumpr clock_stretch_read,1,lt

// Wait for SDA value to be written by slave, minimum of 4us for standard mode
  I2C_delay
// SCL is high, read out bit
  read_SDA
// Set SCL low in preparation for next operation
  clear_SCL

  ret // bit in r0

// Write a byte to I2C bus. Return 0 if ack by the slave.
i2c_write_byte:
  stage_rst
next_bit:
  and r0,r2,0x80
  psr
  jump i2c_write_bit
  lsh r2,r2,1
  stage_inc 1
  jumps next_bit,8,lt

  psr
  jump i2c_read_bit
  ret // nack


// Read a byte from I2C bus
i2c_read_byte:
  push r2
  move r2,0
  stage_rst
next_bit_read:
  psr
  jump i2c_read_bit
  lsh r2,r2,1
  or r2,r2,r0
  stage_inc 1
  jumps next_bit_read,8,lt

  pop r0
  psr
  jump i2c_write_bit

  move r0,r2

  ret


/*
 * I2C ULP utility routines
 */

write_intro:
  psr
  jump i2c_start_cond

  ld r2,r3,20 // Address
  lsh r2,r2,1
  psr
  jump i2c_write_byte
  jumpr popfail,1,ge

  ld r2,r3,16 // Register
  psr
  jump i2c_write_byte
  jumpr popfail,1,ge
  ret


.global write8
write8:
  psr
  jump write_intro

write_b:
  ld r2,r3,8 // data byte
  psr
  jump i2c_write_byte
  jumpr fail,1,ge

  psr
  jump i2c_stop_cond

  move r2,0 // Ok
  ret


read_intro:
  psr
  jump i2c_start_cond

  ld r2,r3,16 // Address
  lsh r2,r2,1
  psr
  jump i2c_write_byte
  jumpr popfail,1,ge

  ld r2,r3,12 // Register
  psr
  jump i2c_write_byte
  jumpr popfail,1,ge

  psr
  jump i2c_start_cond

  ld r2,r3,16
  lsh r2,r2,1
  or r2,r2,1 // Address Read
  psr
  jump i2c_write_byte
  jumpr popfail,1,ge

  ret
popfail:
  pop r1 // pop caller return address
  move r2,1
  ret

.global read8
read8:
  psr
  jump read_intro

  move r2,1 // last byte
  psr
  jump i2c_read_byte
  push r0

  psr
  jump i2c_stop_cond

  pop r0

  move r2,0 // OK
  ret
fail:
  move r2,1
  ret
  
