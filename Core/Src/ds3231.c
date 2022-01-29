#include "ds3231.h"

extern I2C_HandleTypeDef DS3231_I2C;

uint8_t rtcBuffer[19], rtcBufferSet[17];

uint8_t decToBcd(uint8_t val) {
    return ((val / 10 * 16) + (val % 10));
}

uint8_t bcdToDec(uint8_t val) {
    return ((val / 16 * 10) + (val % 16));
}

void DS3231_Update(void) {
    uint8_t cmd = 0;
    HAL_I2C_Master_Transmit(&DS3231_I2C, DS3231_ADDRESS, &cmd, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&DS3231_I2C, DS3231_ADDRESS, rtcBuffer, 19, HAL_MAX_DELAY);
    for (uint8_t i = 0; i < 16; i++) rtcBufferSet[i+1] = rtcBuffer[i];
}

uint8_t DS3231_getSec(void) {
    return bcdToDec(rtcBuffer[0]);
}

uint8_t DS3231_getMin(void) {
    return bcdToDec(rtcBuffer[1]);
}

uint8_t DS3231_getHrs(void) {
    return bcdToDec(rtcBuffer[2]);
}

uint8_t DS3231_getWeekDay(void) {
    return bcdToDec(rtcBuffer[3]);
}

uint8_t DS3231_getDate(void) {
    return bcdToDec(rtcBuffer[4]);
}

uint8_t DS3231_getMonth(void) {
    return bcdToDec(rtcBuffer[5]);
}

uint8_t DS3231_getYear(void) {
    return bcdToDec(rtcBuffer[6]);
}

uint8_t DS3231_getAlarm1Sec(void) {
    return bcdToDec(rtcBuffer[7]);
}

uint8_t DS3231_getAlarm1Min(void) {
    return bcdToDec(rtcBuffer[8]);
}

uint8_t DS3231_getAlarm1Hour(void) {
    return bcdToDec(rtcBuffer[9]);
}

uint8_t DS3231_getAlarm1WeekDay(void) {
    return bcdToDec(rtcBuffer[10]);
}

uint8_t DS3231_getAlarm1Date(void) {
    return bcdToDec(rtcBuffer[11]);
}

uint8_t DS3231_getAlarm2Min(void) {
    return bcdToDec(rtcBuffer[12]);
}

uint8_t DS3231_getAlarm2Hour(void) {
    return bcdToDec(rtcBuffer[13]);
}

uint8_t DS3231_getAlarm2WeekDay(void) {
    return bcdToDec(rtcBuffer[14]);
}

uint8_t DS3231_getAlarm2Date(void) {
    return bcdToDec(rtcBuffer[15]);
}

double DS3231_getTemp(void) {
    uint8_t tempMSB = rtcBuffer[17];
    uint8_t tempLSB = rtcBuffer[18];
    double t = 0.0;
    tempLSB >>= 6;
    tempLSB &= 0x03;
    t = ((double) tempLSB);
    t *= 0.25;
    t += tempMSB;
    return t;
}

void DS3231_setSec(uint8_t value) {
    rtcBufferSet[1] = decToBcd(value);
    HAL_I2C_Master_Transmit(&DS3231_I2C, DS3231_ADDRESS, rtcBufferSet, 17, HAL_MAX_DELAY);
}

void DS3231_setMin(uint8_t value) {
    rtcBufferSet[2] = decToBcd(value);
    HAL_I2C_Master_Transmit(&DS3231_I2C, DS3231_ADDRESS, rtcBufferSet, 17, HAL_MAX_DELAY);
}

void DS3231_setHrs(uint8_t value) {
    rtcBufferSet[3] = decToBcd(value);
    HAL_I2C_Master_Transmit(&DS3231_I2C, DS3231_ADDRESS, rtcBufferSet, 17, HAL_MAX_DELAY);
}

void DS3231_setWeekDay(uint8_t value) {
    rtcBufferSet[4] = decToBcd(value);
    HAL_I2C_Master_Transmit(&DS3231_I2C, DS3231_ADDRESS, rtcBufferSet, 17, HAL_MAX_DELAY);
}

void DS3231_setDate(uint8_t value) {
    rtcBufferSet[5] = decToBcd(value);
    HAL_I2C_Master_Transmit(&DS3231_I2C, DS3231_ADDRESS, rtcBufferSet, 17, HAL_MAX_DELAY);
}

void DS3231_setMonth(uint8_t value) {
    rtcBufferSet[6] = decToBcd(value);
    HAL_I2C_Master_Transmit(&DS3231_I2C, DS3231_ADDRESS, rtcBufferSet, 17, HAL_MAX_DELAY);
}

void DS3231_setYear(uint8_t value) {
    rtcBufferSet[7] = decToBcd(value);
    HAL_I2C_Master_Transmit(&DS3231_I2C, DS3231_ADDRESS, rtcBufferSet, 17, HAL_MAX_DELAY);
}

void DS3231_setAlarm1Sec(uint8_t value) {
    rtcBufferSet[8] = decToBcd(value);
    HAL_I2C_Master_Transmit(&DS3231_I2C, DS3231_ADDRESS, rtcBufferSet, 17, HAL_MAX_DELAY);
}

void DS3231_setAlarm1Min(uint8_t value) {
    rtcBufferSet[9] = decToBcd(value);
    HAL_I2C_Master_Transmit(&DS3231_I2C, DS3231_ADDRESS, rtcBufferSet, 17, HAL_MAX_DELAY);
}

void DS3231_setAlarm1Hour(uint8_t value) {
    rtcBufferSet[10] = decToBcd(value);
    HAL_I2C_Master_Transmit(&DS3231_I2C, DS3231_ADDRESS, rtcBufferSet, 17, HAL_MAX_DELAY);
}

void DS3231_setAlarm1Day(uint8_t value) {
    rtcBufferSet[11] = decToBcd(value);
    HAL_I2C_Master_Transmit(&DS3231_I2C, DS3231_ADDRESS, rtcBufferSet, 17, HAL_MAX_DELAY);
}

void DS3231_setAlarm1Date(uint8_t value) {
    rtcBufferSet[12] = decToBcd(value);
    HAL_I2C_Master_Transmit(&DS3231_I2C, DS3231_ADDRESS, rtcBufferSet, 17, HAL_MAX_DELAY);
}

void DS3231_setAlarm2Min(uint8_t value) {
    rtcBufferSet[13] = decToBcd(value);
    HAL_I2C_Master_Transmit(&DS3231_I2C, DS3231_ADDRESS, rtcBufferSet, 17, HAL_MAX_DELAY);
}

void DS3231_setAlarm2Hour(uint8_t value) {
    rtcBufferSet[14] = decToBcd(value);
    HAL_I2C_Master_Transmit(&DS3231_I2C, DS3231_ADDRESS, rtcBufferSet, 17, HAL_MAX_DELAY);
}

void DS3231_setAlarm2Day(uint8_t value) {
    rtcBufferSet[15] = decToBcd(value);
    HAL_I2C_Master_Transmit(&DS3231_I2C, DS3231_ADDRESS, rtcBufferSet, 17, HAL_MAX_DELAY);
}

void DS3231_setAlarm2Date(uint8_t value) {
    rtcBufferSet[16] = decToBcd(value);
    HAL_I2C_Master_Transmit(&DS3231_I2C, DS3231_ADDRESS, rtcBufferSet, 17, HAL_MAX_DELAY);
}

uint64_t DS3231_getUnix(uint64_t zoneCorrection) {
    uint8_t timeYear = DS3231_getYear();
    uint8_t timeMonth = DS3231_getMonth();
    uint8_t timeDate = DS3231_getDate();
    uint8_t timeHour = DS3231_getHrs();
    uint8_t timeMin = DS3231_getMin();
    uint8_t timeSec = DS3231_getSec();

    const uint8_t dim[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    uint16_t dc;
    dc = timeDate;
    for (uint8_t i = 0; i < (timeMonth - 1); i++) dc += dim[i];
    if ((timeMonth > 2) && (((timeYear) % 4) == 0)) ++dc;
    dc = dc + (365 * (timeYear)) + (((timeYear) + 3) / 4) - 1;
    return ((((((dc * 24L) + timeHour) * 60) + timeMin) * 60) + timeSec) + 946684800 + zoneCorrection;
}

double normalize(double v) {
    v = v - floor(v);
    if (v < 0)
        v = v + 1;
    return v;
}

float DS3231_getMoonDay(void) {
    uint16_t Y = DS3231_getYear() + 2000;
    uint8_t M = DS3231_getMonth();
    uint8_t D = DS3231_getDate();
    float moonDay;
    double I;
    uint64_t YY, MM, K1, K2, K3, JD;
    YY = Y - floor((12 - M) / 10);
    MM = M + 9;
    if (MM >= 12) {
        MM = MM - 12;
    }
    K1 = floor(365.25 * (YY + 4712));
    K2 = floor(30.6 * MM + 0.5);
    K3 = floor(floor((YY / 100) + 49) * 0.75) - 38;
    JD = K1 + K2 + D + 59;
    if (JD > 2299160) {
        JD = JD - K3;
    }
    I = normalize((JD - 2451550.1) / 29.530588853);
    moonDay = I * 29.53;
    return moonDay;
}
