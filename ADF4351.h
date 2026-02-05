/*
 * ADF4351.h - Arduino library for the ADF4351 wideband frequency synthesizer
 * 
 * This library provides an easy-to-use interface for controlling the ADF4351
 * PLL frequency synthesizer via SPI.
 * 
 * Author: Nandhu
 * License: MIT
 * 
 * Features:
 * - Automatic VCO divider selection
 * - Fractional-N and Integer-N mode support
 * - Configurable reference frequency and channel spacing
 * - Simple frequency setting interface
 */

#ifndef ADF4351_H
#define ADF4351_H

#include <Arduino.h>
#include <SPI.h>

class ADF4351 {
public:
    /**
     * @brief Constructor
     * @param lePin Latch Enable (LE/CS) pin number
     */
    ADF4351(uint8_t lePin);
    
    /**
     * @brief Initialize the ADF4351 with default settings
     * @param refFreqMHz Reference frequency in MHz (default 25.0 MHz)
     */
    void begin(double refFreqMHz = 25.0);
    
    /**
     * @brief Set the output frequency
     * @param freqMHz Desired output frequency in MHz (35 - 4400 MHz)
     * @param channelSpacingMHz Frequency step/channel spacing in MHz (default 0.01 MHz = 10 kHz)
     * @return true if frequency was set successfully, false otherwise
     */
    bool setFrequency(double freqMHz, double channelSpacingMHz = 0.01);
    
    /**
     * @brief Set reference frequency configuration
     * @param refFreqMHz Reference input frequency in MHz
     * @param rCounter Reference divider (R counter)
     * @param refDoubler Enable reference doubler (0 or 1)
     * @param refDiv2 Enable reference divide-by-2 (0 or 1)
     */
    void setReference(double refFreqMHz, uint8_t rCounter = 1, uint8_t refDoubler = 0, uint8_t refDiv2 = 0);
    
    /**
     * @brief Set RF output power level
     * @param power Power level (0-3, where 3 is maximum)
     */
    void setOutputPower(uint8_t power);
    
    /**
     * @brief Enable or disable RF output
     * @param enable true to enable, false to disable
     */
    void enableOutput(bool enable);
    
    /**
     * @brief Set charge pump current
     * @param current Charge pump current setting (0-15)
     */
    void setChargePumpCurrent(uint8_t current);
    
    /**
     * @brief Get the currently set output frequency
     * @return Current output frequency in MHz
     */
    double getFrequency() const;
    
    /**
     * @brief Get the calculated PFD frequency
     * @return Phase detector frequency in MHz
     */
    double getPFDFrequency() const;

private:
    uint8_t _lePin;
    double _refFreqMHz;
    double _outputFreqMHz;
    double _pfdFreqMHz;
    
    // Reference settings
    uint8_t _rCounter;
    uint8_t _refDoubler;
    uint8_t _refDiv2;
    
    // Output settings
    uint8_t _outputPower;
    uint8_t _rfOutputEnable;
    uint8_t _chargePumpCurr;
    
    /**
     * @brief Write a 32-bit value to the ADF4351 via SPI
     * @param data 32-bit register value to write
     */
    void writeRegister(uint32_t data);
    
    /**
     * @brief Calculate and write all registers for current frequency
     * @param channelSpacingMHz Frequency step in MHz
     * @return true if successful
     */
    bool updateRegisters(double channelSpacingMHz);
    
    /**
     * @brief Select appropriate output divider for frequency range
     * @param freqMHz Output frequency in MHz
     * @param outDivider Reference to store divider value
     * @param outRFdivSel Reference to store RF divider select code
     */
    void selectOutputDivider(double freqMHz, double &outDivider, uint8_t &outRFdivSel);
};

#endif // ADF4351_H
