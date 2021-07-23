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

#include <stdint.h>
#include "RM3100.h"



//struct MeasurementScale _scale;



/**
 * Gets the current single measurement mode config, POLL(0x00)
 *
 * @param smm pointer to read config into
 * @return 0 on success
 */
int RM3100_GetSingleMeasurementMode(void *self, struct SingleMeasurementMode *smm)
{
    RM3100_Iface *iface = self;
    return iface->RM3100_read(iface, RM3100_REG_POLL, (uint8_t *)smm, sizeof(*smm));
}


/**
 * Sets the current single measurement mode config
 *
 * @return 0 on success
 */
int RM3100_SetSingleMeasurementMode(void *self, uint8_t mode)
{
    RM3100_Iface *iface = self;
    uint8_t smm = mode;
    return iface->RM3100_write(iface, RM3100_REG_POLL, (uint8_t *)&smm, sizeof(smm));
}


/**
 * Gets the current continuous measurement mode config, CMM (0x01)
 *
 * @param cmm pointer to read config into
 * @return 0 on success
 */
int RM3100_GetContinuousMeasurementMode(void *self, struct ContinuousMeasurementMode *cmm)
{
    RM3100_Iface *iface = self;
    return iface->RM3100_read(iface, RM3100_REG_CMM, (uint8_t *)cmm, sizeof(*cmm));
}


/**
 * Sets the current continuous measurement mode config
 *
 * @param enabled enable CMM -- true = enabled, false = disabled
 * @return 0 on success
 */
int RM3100_StartContinuousMeasurementMode(void *self, uint8_t mode)
{
    RM3100_Iface *iface = self;
    uint8_t         cmm = (mode | 0x01);

    return iface->RM3100_write(iface, RM3100_REG_CMM, (uint8_t *)&cmm, sizeof(cmm));
}

int RM3100_StopContinuousMeasurementMode(void *self, uint8_t mode)
{
    RM3100_Iface *iface = self;
    uint8_t         cmm = (mode | 0x00);

    return iface->RM3100_write(iface, RM3100_REG_CMM, (uint8_t *)&cmm, sizeof(cmm));
}


/**
 * Gets the current cycle counts, CCX, CCY, CCZ (0x04 -- 0x09)
 *
 * @param cc pointer to read into
 * @return 0 on success
 */
int RM3100_GetCycleCounts(void *self, struct CycleCounts *cc)
{
    RM3100_Iface *iface = self;
    uint8_t buffer[6];
    int ret = iface->RM3100_read(iface, RM3100_REG_CCX, buffer, sizeof(buffer));
    if (ret) {
        return ret;
    }
    cc->x = (buffer[0] << 8) | buffer[1];
    cc->y = (buffer[2] << 8) | buffer[3];
    cc->z = (buffer[4] << 8) | buffer[5];
    return RM3100_RET_OK;
}


/*
 * Sets the current cycle counts (x, y, z)
 *
 * @param x cycle counts for X axis
 * @param y cycle counts for Y axis
 * @param z cycle counts for Z axis
 * @return 0 on success
 */
int RM3100_SetCycleCounts(void *self, uint16_t x, uint16_t y, uint16_t z)
{
    RM3100_Iface *iface = self;
    struct CycleCounts cc = {
        .x = x,
        .y = y,
        .z = z,
    };
    
    // convert to BE
    uint8_t buffer[] = {
        (cc.x >> 8), (cc.x & 0xFF),
        (cc.y >> 8), (cc.y & 0xFF),
        (cc.z >> 8), (cc.z & 0xFF),
    };

    iface->ratio_x = 1.0f / ((x * 0.3627f) + 1.85f);
    iface->ratio_y = 1.0f / ((y * 0.3627f) + 1.85f);
    iface->ratio_z = 1.0f / ((z * 0.3627f) + 1.85f);
    
    return iface->RM3100_write(iface, RM3100_REG_CCX, buffer, sizeof(buffer));
}


/**
 * Gets the continuous mode update rate, TMRC (0x0B)
 *
 * @param tmrc pointer to read into
 * @return 0 on success
 */
int RM3100_GetContinuousMeasurementModeUpdateRate(void *self, uint8_t *tmrc)
{
    RM3100_Iface *iface = self;
    return iface->RM3100_read(iface, RM3100_REG_TMRC, tmrc, sizeof(*tmrc));
}


/**
 * Sets the continuous mode update rate (TMRC)
 *
 * @param tmrc rate to set according to TMRC table
 * @return 0 on success
 */
int RM3100_SetContinuousMeasurementModeUpdateRate(void *self, uint8_t tmrc)
{
    RM3100_Iface *iface = self;
    uint8_t value = (tmrc & RM3100_CMM_RATE_MASK) | RM3100_CMM_RATE_MSB;
    return iface->RM3100_write(iface, RM3100_REG_TMRC, &value, sizeof(value));
}


/**
 * Sets the contiunous mode update rate (Hz)
 *
 * @param rate rate to set in Hz
 * @return 0 on success
 */
int RM3100_SetCMM_SampleRate(void *self, float rate)
{
    RM3100_Iface *iface = self;
    float r = 600.0f;
    uint8_t tmrc = RM3100_CMM_RATE_600_0_HZ;

    while ((tmrc < RM3100_CMM_RATE_MASK) && ((r / 2.0f) >= rate)) {
        r /= 2.0f;
        tmrc++;
    }
    return RM3100_SetContinuousMeasurementModeUpdateRate(iface, tmrc);
}


/**
 * Gets the current status
 *
 * @param status pointer to read into
 * @return 0 on success
 */
int RM3100_GetStatus(void *self)
{
    uint8_t status      = 0;
    RM3100_Iface *iface = self;
    iface->RM3100_read(iface, RM3100_REG_STATUS, (uint8_t *)&status, sizeof(status));
    return (status >> 7);
}


/**
 * Gets the current measurement data (24-bit signed)
 *
 * @param m pointer to read into
 * @return 0 on success
 */
int RM3100_GetXYZ_Raw(void *self, struct Measurement *m)
{
    RM3100_Iface *iface = self;
    uint8_t buffer[9] = { 0 };
    int ret = iface->RM3100_read(iface, RM3100_REG_MX, buffer, sizeof(buffer));
    if (ret) {
        return ret;
    }
    m->x = (((int8_t)buffer[0]) << 16) | (buffer[1] << 8) | (buffer[2]);
    m->y = (((int8_t)buffer[3]) << 16) | (buffer[4] << 8) | (buffer[5]);
    m->z = (((int8_t)buffer[6]) << 16) | (buffer[7] << 8) | (buffer[8]);
    return RM3100_RET_OK;
}


/**
 * Gets the current measurement sample (scale to ¦ÌT)
 *
 * @param s pointer to read into
 * @return 0 on success
 */
int RM3100_GetData(void *self, struct Data *s)
{
    RM3100_Iface *iface = self;
    struct Measurement m = { 0 };
    int ret = RM3100_GetXYZ_Raw(iface, &m);
    if (ret) {
        return ret;
    }
    s->x = (float)m.x * iface->ratio_x;
    s->y = (float)m.y * iface->ratio_y;
    s->z = (float)m.z * iface->ratio_z;
    
    return RM3100_RET_OK;
}


/**
 * Gets the current self-test config/result
 *
 * @param cfg pointer to read into
 * @return 0 on success
 */
int RM3100_GetSelfTestConfig(void *self, struct SelfTestConfig *cfg)
{
    RM3100_Iface *iface = self;
    return iface->RM3100_read(iface, RM3100_REG_BIST, (uint8_t *)cfg, sizeof(*cfg));
}


/**
 * Sets the current self-test config
 *
 * @param count LR periods
 * @param timeout sleep oscillation cycles
 * @return 0 on success
 */
int RM3100_SetSelfTestConfig(void *self, uint8_t count, uint8_t timeout, bool enabled)
{
    RM3100_Iface *iface = self;
    struct SelfTestConfig cfg = {
        .bp = (count & RM3100_SELF_TEST_COUNT_MASK),
        .bw = (timeout & RM3100_SELF_TEST_TIMEOUT_MASK),
        .ste = enabled,
    };
    return iface->RM3100_write(iface, RM3100_REG_BIST, (uint8_t *)&cfg, sizeof(cfg));
}


/**
 * Performs a self-test, returning result
 *
 * @param count LR periods
 * @param timeout sleep oscillation cycles
 * @param result pointer to read result into
 * @return 0 on success
 */
int RM3100_PerformSelfTest(void *self, uint8_t count, uint8_t timeout,
                            struct SelfTestConfig *result)
{
    RM3100_Iface *iface = self;
    
    // configure test
    int ret = RM3100_SetSelfTestConfig(iface, count, timeout, true);
    if (ret) {
        return ret;
    }

    HAL_Delay(1);

    // initiate single measurement
    ret = RM3100_SetSingleMeasurementMode(iface, CMM_AXIS_X|CMM_AXIS_Y|CMM_AXIS_Z);
    if (ret) {
        return ret;
    }

    // wait 1 ms for measurement
    HAL_Delay(1);

    // get result
    ret = RM3100_GetSelfTestConfig(iface, result);
    if (ret) {
        return ret;
    }

    HAL_Delay(1);

    return RM3100_ClearSelfTest(iface);
}


/**
 * Clears the self-test config
 *
 * @return 0 on success
 */
int RM3100_ClearSelfTest(void *self)
{
    uint8_t value       = 0;
    RM3100_Iface *iface = self;
    return iface->RM3100_write(iface, RM3100_REG_BIST, &value, sizeof(value));
}


/**
 * Gets the current handshake config
 *
 * @param cfg pointer to read into
 * @return 0 on success
 */
int RM3100_GetHandShakeConfig(void *self, struct HandShakeConfig *cfg)
{
    RM3100_Iface *iface = self;
    return iface->RM3100_read(iface, RM3100_REG_HSHAKE, (uint8_t *)cfg, sizeof(*cfg));
}


/**
 * Sets the current handshake config
 *
 * @param cfg pointer to write from
 * @return 0 on success
 */
int RM3100_SetHandShakeConfig(void *self, struct HandShakeConfig *cfg)
{
    cfg->res2           = 0;
    cfg->res3           = 1;
    RM3100_Iface *iface = self;
    return iface->RM3100_write(iface, RM3100_REG_HSHAKE, (uint8_t *)cfg, sizeof(*cfg));
}


/**
 * Sets the current data ready config
 *
 * @param on_write 1 = drdy cleared on any register write
 * @param on_read_measurement 1 = drdy cleared on measurement register read
 * @return 0 on success
 */
int RM3100_SetDrdyClearConfig(void *self, bool on_write, bool on_read_measurement)
{
    RM3100_Iface *iface = self;
    struct HandShakeConfig cfg = {
        .drc0 = on_write,
        .drc1 = on_read_measurement,
    };
    return RM3100_SetHandShakeConfig(iface, &cfg);
}


/**
 * Gets the current hardware revision
 *
 * @param rev pointer to read into
 * @return 0 on success
 */
uint8_t RM3100_GetHardwareRevision(void *self)
{
    uint8_t revid       = 0x00;
    RM3100_Iface *iface = self;
    iface->RM3100_read(iface, RM3100_REG_REVID, &revid, sizeof(revid));
    return revid;
}


int RM3100_Init(void *self){
    int ret = RM3100_RET_OK;

    do{
        ret = RM3100_SetCycleCounts(self, CYCLE_COUNTS_DEFAULT, CYCLE_COUNTS_DEFAULT, CYCLE_COUNTS_DEFAULT);
        if(RM3100_RET_OK != ret){
            break;
        }

    }while(0);

    return ret;
}
