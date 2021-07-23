/**
    File Name          : RM3100.c
    version            : V1.1


    Description        : This file provides code for the RM3100 driver interface. 



    COPYRIGHT(c)  Shanghai Chuanggan sensor technology co., LTD

    Redistribution and use in source and binary forms, with or without modification,
    are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
    3. Neither the name of STMicroelectronics nor the names of its contributors
    may be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
    FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
    DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
    SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  
*/


#ifndef __RM3100_H
#define __RM3100_H

#include <stdint.h>
#include <stdbool.h>


/* I/O interface */
typedef struct rm3100_iface {
    /* read reg value */
    int (*RM3100_read)(void *self, uint8_t reg, uint8_t *pData, uint16_t Size);
    
    /* write reg value */
    int (*RM3100_write)(void *self, uint8_t reg, uint8_t *pData, uint16_t Size);
    
    /* config params */
    void *params;

    float  ratio_x;
    float  ratio_y;
    float  ratio_z;

}RM3100_Iface;


enum ReturnCode {
    RM3100_RET_EIO      = -22,
    RM3100_RET_ENODEV   = -19,
    RM3100_RET_EINVAL   = -5,
    RM3100_RET_OK       = 0,
};


enum Register {
    RM3100_REG_POLL =   0x00, // polls for a single measurement
    RM3100_REG_CMM =    0x01, // initiates continuous measurement mode
    RM3100_REG_CCX =    0x04, // cycle counts -- X axis
    RM3100_REG_CCY =    0x06, // cycle counts -- Y axis
    RM3100_REG_CCZ =    0x08, // cycle counts -- Z axis
    RM3100_REG_TMRC =   0x0B, // sets continuous mode data rate
    RM3100_REG_MX =     0x24, // measurement results -- X axis
    RM3100_REG_MY =     0x27, // measurement results -- Y axis
    RM3100_REG_MZ =     0x2A, // measurement results -- Z axis
    RM3100_REG_BIST =   0x33, // built-in self test
    RM3100_REG_STATUS = 0x34, // status of DRDY
    RM3100_REG_HSHAKE = 0x35, // handshake register
    RM3100_REG_REVID =  0x36, // MagI2C revision identification
};



#define RM3100_REVID                (0x22)

#define SMM_AXIS_X                  (1 << 4)
#define SMM_AXIS_Y                  (1 << 5)
#define SMM_AXIS_Z                  (1 << 6)

#define CMM_AXIS_X                  (1 << 4)
#define CMM_AXIS_Y                  (1 << 5)
#define CMM_AXIS_Z                  (1 << 6)




/* CMMode config */
#define CMM_DRDM_ALL_AXIS           (0 << 2)
#define CMM_DRDM_ANY_AXIS           (1 << 2)


#define CYCLE_COUNTS_DEFAULT        (200)


struct SingleMeasurementMode {
    uint8_t res0:4;
    uint8_t pmx:1;
    uint8_t pmy:1;
    uint8_t pmz:1;
    uint8_t res7:1;
};



struct ContinuousMeasurementMode {
    uint8_t start:1;   // continuous measurement mode enabled
    uint8_t res1:1;
    uint8_t drdm:2;    // data ready mode
    uint8_t cmx:1;     // X axis measurement enabled in CMM
    uint8_t cmy:1;     // Y axis measurement enabled in CMM
    uint8_t cmz:1;     // Z axis measurement enabled in CMM
    uint8_t res7:1;
};

struct CycleCounts {
    uint16_t x;
    uint16_t y;
    uint16_t z;
};

enum ContinuousMeasurementModeUpdateRate {
    RM3100_CMM_RATE_600_0_HZ = 0x02, //   ~600 Hz
    RM3100_CMM_RATE_300_0_HZ = 0x03, //   ~300 Hz
    RM3100_CMM_RATE_150_0_HZ = 0x04, //   ~150 Hz
    RM3100_CMM_RATE_75_0_HZ =  0x05, //    ~75 Hz
    RM3100_CMM_RATE_37_0_HZ =  0x06, //    ~37 Hz
    RM3100_CMM_RATE_18_0_HZ =  0x07, //    ~18 Hz
    RM3100_CMM_RATE_9_0_HZ =   0x08, //     ~9 Hz
    RM3100_CMM_RATE_4_5_HZ =   0x09, //   ~4.5 Hz
    RM3100_CMM_RATE_2_3_HZ =   0x0A, //   ~2.3 Hz
    RM3100_CMM_RATE_1_2_HZ =   0x0B, //   ~1.2 Hz
    RM3100_CMM_RATE_0_6_HZ =   0x0C, //   ~0.6 Hz
    RM3100_CMM_RATE_0_3_HZ =   0x0D, //   ~0.3 Hz
    RM3100_CMM_RATE_0_015_HZ = 0x0E, // ~0.015 Hz
    RM3100_CMM_RATE_0_075_HZ = 0x0F, // ~0.075 Hz
    RM3100_CMM_RATE_MASK = RM3100_CMM_RATE_0_075_HZ,
    RM3100_CMM_RATE_MSB =      0x90,
};

enum StatusFlag {
    RM3100_STATUS_FLAG_DRDY = (1 << 7),
};

struct Status {
    uint8_t res0:7;
    uint8_t drdy:1;
};

struct Measurement {
    int32_t x;
    int32_t y;
    int32_t z;
};

struct Data {
    float x;
    float y;
    float z;
};

struct MeasurementScale {
    float x;
    float y;
    float z;
};

enum SelfTestCount {
    RM3100_SELF_TEST_COUNT_1 = 0x01, // 1 LR periods
    RM3100_SELF_TEST_COUNT_2 = 0x02, // 2 LR periods
    RM3100_SELF_TEST_COUNT_4 = 0x03, // 4 LR periods
    RM3100_SELF_TEST_COUNT_MASK = RM3100_SELF_TEST_COUNT_4,
};

enum SelfTestTimeout {
    RM3100_SELF_TEST_TIMEOUT_30_US = 0x01,  // 1 cycle  --  30 ¦Ìs
    RM3100_SELF_TEST_TIMEOUT_60_US = 0x02,  // 2 cycles --  60 ¦Ìs
    RM3100_SELF_TEST_TIMEOUT_120_US = 0x03, // 4 cycles -- 120 ¦Ìs
    RM3100_SELF_TEST_TIMEOUT_MASK = RM3100_SELF_TEST_TIMEOUT_120_US,
};

struct SelfTestConfig {
    uint8_t bp:2;  // test count (LR periods)
    uint8_t bw:2;  // timeout (sleep oscillation cycles)
    uint8_t xok:1; // X result -- 0 = error, 1 = ok
    uint8_t yok:1; // Y result -- 0 = error, 1 = ok
    uint8_t zok:1; // Z result -- 0 = error, 1 = ok
    uint8_t ste:1; // self test enable -- 0 = disable, 1 = enable
};

struct HandShakeConfig {
    uint8_t drc0:1;  // clear drdy on any register write
    uint8_t drc1:1;  // clear drdy on measurement read
    uint8_t res2:1;  // 0
    uint8_t res3:1;  // 1
    uint8_t nack0:1; // 1 when undef register write
    uint8_t nack1:1; // 1 when write SMM when CMM or visa versa
    uint8_t nack2:1; // 1 when measurement read before data ready
};


int RM3100_SetSingleMeasurementMode(void *self, uint8_t mode);

int RM3100_StartContinuousMeasurementMode(void *self, uint8_t mode);

int RM3100_StopContinuousMeasurementMode(void *self, uint8_t mode);

int RM3100_SetCycleCounts(void *self, uint16_t x, uint16_t y, uint16_t z);

int RM3100_SetContinuousMeasurementModeUpdateRate(void *self, uint8_t tmrc);

int RM3100_SetCMM_SampleRate(void *self, float rate);

int RM3100_GetStatus(void *self);

int RM3100_GetXYZ_Raw(void *self, struct Measurement *m);

int RM3100_GetData(void *self, struct Data *s);

int RM3100_PerformSelfTest(void *self, uint8_t count, uint8_t timeout,
                    struct SelfTestConfig *result);

uint8_t RM3100_GetHardwareRevision(void *self);


int RM3100_Init(void *self);

#endif /* __RM3100_H */

