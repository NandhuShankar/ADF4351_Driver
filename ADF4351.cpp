/*
 * ADF4351.cpp - Arduino library for the ADF4351 wideband frequency synthesizer
 * 
 * Author: Nandhu
 * License: MIT
 */

#include "ADF4351.h"

ADF4351::ADF4351(uint8_t lePin) 
    : _lePin(lePin),
      _refFreqMHz(25.0),
      _outputFreqMHz(0.0),
      _pfdFreqMHz(25.0),
      _rCounter(1),
      _refDoubler(0),
      _refDiv2(0),
      _outputPower(3),
      _rfOutputEnable(1),
      _chargePumpCurr(7) {
}

void ADF4351::begin(double refFreqMHz) {
    _refFreqMHz = refFreqMHz;
    
    // Initialize LE pin
    pinMode(_lePin, OUTPUT);
    digitalWrite(_lePin, HIGH);
    
    // Initialize SPI
    SPI.begin();
    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(MSBFIRST);
    
    // Calculate PFD frequency
    _pfdFreqMHz = _refFreqMHz * (1 + _refDoubler) / (_rCounter * (1 + _refDiv2));
}

void ADF4351::setReference(double refFreqMHz, uint8_t rCounter, uint8_t refDoubler, uint8_t refDiv2) {
    _refFreqMHz = refFreqMHz;
    _rCounter = rCounter;
    _refDoubler = refDoubler;
    _refDiv2 = refDiv2;
    
    // Recalculate PFD frequency
    _pfdFreqMHz = _refFreqMHz * (1 + _refDoubler) / (_rCounter * (1 + _refDiv2));
}

bool ADF4351::setFrequency(double freqMHz, double channelSpacingMHz) {
    // Validate frequency range
    if (freqMHz < 35.0 || freqMHz > 4400.0) {
        return false;
    }
    
    _outputFreqMHz = freqMHz;
    return updateRegisters(channelSpacingMHz);
}

void ADF4351::setOutputPower(uint8_t power) {
    if (power > 3) power = 3;
    _outputPower = power;
}

void ADF4351::enableOutput(bool enable) {
    _rfOutputEnable = enable ? 1 : 0;
}

void ADF4351::setChargePumpCurrent(uint8_t current) {
    if (current > 15) current = 15;
    _chargePumpCurr = current;
}

double ADF4351::getFrequency() const {
    return _outputFreqMHz;
}

double ADF4351::getPFDFrequency() const {
    return _pfdFreqMHz;
}

void ADF4351::writeRegister(uint32_t data) {
    digitalWrite(_lePin, LOW);
    SPI.transfer((data >> 24) & 0xFF);
    SPI.transfer((data >> 16) & 0xFF);
    SPI.transfer((data >> 8) & 0xFF);
    SPI.transfer(data & 0xFF);
    digitalWrite(_lePin, HIGH);
    delayMicroseconds(5);
}

void ADF4351::selectOutputDivider(double freqMHz, double &outDivider, uint8_t &outRFdivSel) {
    outDivider = 1.0;
    outRFdivSel = 0;
    
    if (freqMHz >= 2200.0) {
        outDivider = 1.0;
        outRFdivSel = 0;
    } else if (freqMHz >= 1100.0) {
        outDivider = 2.0;
        outRFdivSel = 1;
    } else if (freqMHz >= 550.0) {
        outDivider = 4.0;
        outRFdivSel = 2;
    } else if (freqMHz >= 275.0) {
        outDivider = 8.0;
        outRFdivSel = 3;
    } else if (freqMHz >= 137.5) {
        outDivider = 16.0;
        outRFdivSel = 4;
    } else if (freqMHz >= 68.75) {
        outDivider = 32.0;
        outRFdivSel = 5;
    } else {
        outDivider = 64.0;
        outRFdivSel = 6;
    }
}

bool ADF4351::updateRegisters(double channelSpacingMHz) {
    // Select output divider
    double outputDivider;
    uint8_t RFdivSel;
    selectOutputDivider(_outputFreqMHz, outputDivider, RFdivSel);
    
    // Calculate VCO frequency
    double vcoFreqMHz = _outputFreqMHz * outputDivider;
    
    // Validate VCO frequency range (2200-4400 MHz)
    if (vcoFreqMHz < 2200.0 || vcoFreqMHz > 4400.0) {
        return false;
    }
    
    // Calculate PLL N value
    double N = vcoFreqMHz / _pfdFreqMHz;
    uint16_t N_int = (uint16_t)floor(N);
    
    // Calculate modulus for desired channel spacing
    uint16_t MOD = (uint16_t)round(_pfdFreqMHz / channelSpacingMHz);
    if (MOD > 4095) MOD = 4095;
    
    // Calculate fractional value
    uint16_t N_frac = (uint16_t)round((N - N_int) * MOD);
    if (N_frac >= MOD) {
        N_int += N_frac / MOD;
        N_frac = N_frac % MOD;
    }
    
    // Choose prescaler
    uint8_t prescaler = (N_int < 75) ? 0 : 1;
    
    // Determine if integer-N or fractional-N mode
    uint8_t ldp = (N_frac == 0) ? 1 : 0;
    uint8_t ldf = (N_frac == 0) ? 1 : 0;
    
    // Feedback select (1 = divided when using output divider)
    uint8_t feedbackSelect = (outputDivider > 1.0) ? 1 : 1;
    
    // Band select clock divider (target 125-500 kHz)
    uint16_t bandSelDiv = 200;
    
    // R0: Frequency setup
    uint32_t reg0 = 0x0;
    reg0 |= ((uint32_t)N_int << 15);
    reg0 |= ((uint32_t)N_frac << 3);
    
    // R1: MOD, phase, prescaler
    uint32_t reg1 = 0x1;
    reg1 |= ((uint32_t)MOD << 3);
    reg1 |= (1u << 15);
    reg1 |= ((uint32_t)prescaler << 27);
    
    // R2: Reference and phase detector settings
    uint32_t reg2 = 0x2;
    reg2 |= (0u << 3);                          // Counter reset
    reg2 |= (0u << 4);                          // CP three-state
    reg2 |= (0u << 5);                          // Power-down
    reg2 |= (1u << 6);                          // PD polarity (positive)
    reg2 |= ((uint32_t)ldp << 7);               // Lock detect precision
    reg2 |= ((uint32_t)ldf << 8);               // Lock detect function
    reg2 |= ((uint32_t)_chargePumpCurr << 9);   // Charge pump current
    reg2 |= (0u << 13);                         // Double buffer
    reg2 |= ((uint32_t)_rCounter << 14);        // R counter
    reg2 |= ((uint32_t)_refDiv2 << 24);         // Reference divide-by-2
    reg2 |= ((uint32_t)_refDoubler << 25);      // Reference doubler
    reg2 |= (0u << 26);                         // MUXOUT
    reg2 |= (0u << 29);                         // Low-noise mode
    
    // R3: Clock divider settings
    uint32_t reg3 = 0x3;
    reg3 |= (150u << 3);                        // Clock divider value
    reg3 |= (0u << 15);                         // Clock divider mode
    reg3 |= (0u << 18);                         // CSR
    reg3 |= (0u << 21);                         // Charge cancel
    reg3 |= (0u << 22);                         // Anti-backlash
    reg3 |= (0u << 23);                         // Band select clock mode
    
    // R4: Output settings
    uint32_t reg4 = 0x4;
    reg4 |= ((uint32_t)(_outputPower & 0x3) << 3);
    reg4 |= ((uint32_t)_rfOutputEnable << 5);
    reg4 |= (0u << 6);                          // Aux output power
    reg4 |= (0u << 8);                          // Aux output enable
    reg4 |= (0u << 9);                          // Aux output select
    reg4 |= (0u << 10);                         // Mute till lock
    reg4 |= (0u << 11);                         // VCO power down
    reg4 |= ((uint32_t)(bandSelDiv & 0xFF) << 12);
    reg4 |= ((uint32_t)(RFdivSel & 0x7) << 20);
    reg4 |= ((uint32_t)feedbackSelect << 23);
    
    // R5: Lock detect and reserved bits
    uint32_t reg5 = 0x5;
    reg5 |= (3u << 19);                         // Reserved (must be 11)
    reg5 |= (0u << 21);                         // Reserved (must be 0)
    reg5 |= (1u << 22);                         // Lock detect mode
    
    // Write registers (R5 to R0)
    writeRegister(reg5);
    writeRegister(reg4);
    writeRegister(reg3);
    writeRegister(reg2);
    writeRegister(reg1);
    writeRegister(reg0);
    
    return true;
}
