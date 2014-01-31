/**************************************************************************//**
* \brief Phonecard library for Arduino
* \author Copyright (C) 2014 Julien Le Sech - www.idreammicro.com
* \version 1.0
* \date 20143101
*
* This file is part of the Phonecard library for Arduino.
*
* This library is free software: you can redistribute it and/or modify it under
* the terms of the GNU Lesser General Public License as published by the Free
* Software Foundation, either version 3 of the License, or (at your option) any
* later version.
*
* This library is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
* FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more
* details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program. If not, see http://www.gnu.org/licenses/
*******************************************************************************/

/**************************************************************************//**
 * \headerfile Phonecard.cpp
 ******************************************************************************/

/******************************************************************************
 * Header file inclusions.
 ******************************************************************************/

#define __STDC_LIMIT_MACROS 
#include <stdint.h>
 
#include <Arduino.h>

#include "Phonecard.h"

/******************************************************************************
 * Private macro definitions.
 ******************************************************************************/

/**************************************************************************//**
 * \def TEST_UNIT_COUNT
 ******************************************************************************/
#define PHONECARD__TEST_UNIT_COUNT 9

/******************************************************************************
 * Public method definitions.
 ******************************************************************************/

/**************************************************************************//**
 * \fn Phonecard::Phonecard(uint8_t clockPin, uint8_t resetPin, uint8_t ioPin)
 *
 * \brief Constructor.
 *
 * \param   clockPin    
 * \param   resetPin
 * \param   ioPin
 ******************************************************************************/
Phonecard::Phonecard(uint8_t clockPin, uint8_t resetPin, uint8_t ioPin)
{
    m_rwPin = UINT8_MAX;
    m_clockPin = clockPin;
    m_resetPin = resetPin;
    m_ioPin = ioPin;
}

/**************************************************************************//**
 * \fn Phonecard::Phonecard(uint8_t rwPin, uint8_t clockPin, uint8_t resetPin, uint8_t ioPin)
 *
 * \brief Constructor.
 *
 * \param   rwPin
 * \param   clockPin    
 * \param   resetPin
 * \param   ioPin
 ******************************************************************************/
Phonecard::Phonecard(uint8_t rwPin, uint8_t clockPin, uint8_t resetPin, uint8_t ioPin)
{
    m_rwPin = rwPin;
    m_clockPin = clockPin;
    m_resetPin = resetPin;
    m_ioPin = ioPin;
}

/**************************************************************************//**
 * \fn uint16_t Phoncecard::getCodePro()
 *
 * \brief Get code pro.
 *
 * \return Code pro.
 ******************************************************************************/
uint16_t Phonecard::getCodePro()
{
    return m_codePro;
}

/**************************************************************************//**
 * \fn void Phonecard::getSerialNumber(char** p_serialNumber)
 *
 * \brief Get serial number.
 *
 * \param[in]   p_serialNumber
 ******************************************************************************/
void Phonecard::getSerialNumber(char** p_serialNumber)
{
    *p_serialNumber = m_serialNumber;
}

/**************************************************************************//**
 * \fn Phonecard::ServiceCode_t Phonecard::getServiceCode()
 *
 * \brief Get service code.
 *
 * \return Service code.
 ******************************************************************************/
Phonecard::ServiceCode_t Phonecard::getServiceCode()
{
    return m_serviceCode;
}

/**************************************************************************//**
 * \fn uint8_t Phonecard::getUnitNumber()
 *
 * \brief Get unit number.
 *
 * \return Unit number.
 ******************************************************************************/
uint8_t Phonecard::getUnitNumber()
{
    return m_unitNumber;
}

/**************************************************************************//**
 * \fn uint8_t Phonecard::getCounter512()
 *
 * \brief Get counter 512.
 *
 * \return Counter 512.
 ******************************************************************************/
uint8_t Phonecard::getCounter512()
{
    return m_counter512;
}

/**************************************************************************//**
 * \fn uint8_t Phonecard::getCounter64()
 *
 * \brief Get counter 64.
 *
 * \return Counter 64.
 ******************************************************************************/
uint8_t Phonecard::getCounter64()
{
    return m_counter64;
}

/**************************************************************************//**
 * \fn uint8_t Phonecard::getCounter8()
 *
 * \brief Get counter 8.
 *
 * \return Counter 8.
 ******************************************************************************/
uint8_t Phonecard::getCounter8()
{
    return m_counter8;
}

/**************************************************************************//**
 * \fn uint8_t Phonecard::getCounter1()
 *
 * \brief Get counter 1.
 *
 * \return Counter 1.
 ******************************************************************************/
uint8_t Phonecard::getCounter1()
{
    return m_counter1;
}

/**************************************************************************//**
 * \fn uint8_t Phonecard::getRemainingUnitCount()
 *
 * \brief Get remaining unit count.
 *
 * \return Remaining unit count.
 ******************************************************************************/
uint8_t Phonecard::getRemainingUnitCount()
{
    return m_remainingUnits;
}

/**************************************************************************//**
 * \fn void Phonecard::initialize()
 *
 * \brief Initialize library.
 * Configure the pins used by the library.
 ******************************************************************************/
void Phonecard::initialize()
{
    if (m_rwPin != UINT8_MAX)
    {
        pinMode(m_rwPin, OUTPUT);
    }
    pinMode(m_resetPin, OUTPUT);
    pinMode(m_clockPin, OUTPUT);
    pinMode(m_ioPin, INPUT);
}

/**************************************************************************//**
 * \fn void Phonecard::reset()
 *
 * \brief Reset phonecard.
 ******************************************************************************/
void Phonecard::reset()
{
    if (m_rwPin != UINT8_MAX)
    {
        digitalWrite(m_rwPin, LOW);
    }
    digitalWrite(m_resetPin, LOW);
    digitalWrite(m_clockPin, LOW); 
    delay(25);
    
    digitalWrite(m_resetPin, LOW);
    digitalWrite(m_clockPin, HIGH); 
    delay(25);
    
    digitalWrite(m_resetPin, LOW);
    digitalWrite(m_clockPin, LOW); 
    delay(50);
}

/**************************************************************************//**
 * \fn void Phonecard::read()
 *
 * \brief Read phonecard memory.
 * Dump complete memory and read it.
 ******************************************************************************/
void Phonecard::read()
{
    uint8_t memoryDump[PHONECARD__MEMORY_SIZE] = { 0 };
    dumpMemory(memoryDump);
    
    readCodePro(memoryDump);
    readSerialNumber(memoryDump);
    readServiceCode(memoryDump);
    readUnitNumber(memoryDump);
    readCounter512(memoryDump);
    readCounter64(memoryDump);
    readCounter8(memoryDump);
    readCounter1(memoryDump);
    computeRemaningUnitCount();
}

/**************************************************************************//**
 * \fn Phonecard::dumpMemory()
 *
 * \brief Dump memory.
 *
 * \param[in]   p_buffer    Buffer to store memory dump. Must not be null.
 ******************************************************************************/
void Phonecard::dumpMemory(uint8_t* p_buffer)
{
    for (uint8_t i = 0; i < PHONECARD__MEMORY_SIZE; i++)
    {
        p_buffer[i] = readByte();
    }
}

/******************************************************************************
 * Private method definitions.
 ******************************************************************************/

/**************************************************************************//**
 * \fn void Phonecard::readCodePro(uint8_t* p_dump)
 *
 * \brief Read code pro.
 *
 * \param[in]   p_dump  Memory dump.
 ******************************************************************************/
void Phonecard::readCodePro(uint8_t* p_dump)
{
    m_codePro = p_dump[0] << 8 | p_dump[1];
}

/**************************************************************************//**
 * \fn void Phonecard::readSerialNumber(uint8_t* p_dump)
 *
 * \brief Read serial number.
 *
 * \param[in]   p_dump  Memory dump.
 ******************************************************************************/
void Phonecard::readSerialNumber(uint8_t* p_dump)
{
    m_serialNumber[0] = (p_dump[2] >> 4) + 0x30;
    m_serialNumber[1] = (p_dump[2] & 0x0F) + 0x30;
    m_serialNumber[2] = (p_dump[3] >> 4) + 0x30;
    m_serialNumber[3] = (p_dump[3] & 0x0F) + 0x30;
    m_serialNumber[4] = (p_dump[4] >> 4) + 0x30;
    m_serialNumber[5] = (p_dump[4] & 0x0F) + 0x30;
    m_serialNumber[6] = (p_dump[5] >> 4) + 0x30;
    m_serialNumber[7] = (p_dump[5] & 0x0F) + 0x30;
    m_serialNumber[8] = (p_dump[6] >> 4) + 0x30;
    m_serialNumber[9] = '\0';
}

/**************************************************************************//**
 * \fn void Phonecard::readServiceCode(uint8_t* p_dump)
 *
 * \brief Read service code.
 *
 * \param[in]   p_dump  Memory dump.
 ******************************************************************************/
void Phonecard::readServiceCode(uint8_t* p_dump)
{
    uint8_t serviceCode = p_dump[6] & 0x0F;
    switch (serviceCode)
    {
        case 0:
            m_serviceCode = T2G;
        break;
        
        case 9:
            m_serviceCode = Eurostar;
        break;
        
        default:
        break;   
    }
}

/**************************************************************************//**
 * \fn void Phonecard::readUnitNumber(uint8_t* p_dump)
 *
 * \brief Read unit number.
 *
 * \param[in]   p_dump  Memory dump.
 ******************************************************************************/
void Phonecard::readUnitNumber(uint8_t* p_dump)
{
    uint8_t financialPower = p_dump[7] & 0x0F;
    switch (financialPower)
    {
        case 1:
            m_unitNumber = 5;
        break;
        
        case 3:
            m_unitNumber = 25;
        break;
        
        case 5:
            m_unitNumber = 50;
        break;
        
        case 12:
            m_unitNumber = 120;
        break;
        
        default:
        break;   
    }
}

/**************************************************************************//**
 * \fn void Phonecard::readCounter512(uint8_t* p_dump)
 *
 * \brief Read counter 512.
 *
 * \param[in]   p_dump  Memory dump.
 ******************************************************************************/
void Phonecard::readCounter512(uint8_t* p_dump)
{
    m_counter512 = 0;
    for (uint8_t i = 0; i < 8; i++)
    {
        m_counter512 += ((p_dump[8] >> i) & 0x01) ? 1 : 0;
    }
}

/**************************************************************************//**
 * \fn void Phonecard::readCounter64(uint8_t* p_dump)
 *
 * \brief Read counter 64.
 *
 * \param[in]   p_dump  Memory dump.
 ******************************************************************************/
void Phonecard::readCounter64(uint8_t* p_dump)
{
    m_counter64 = 0;
    for (uint8_t i = 0; i < 8; i++)
    {
        m_counter64 += ((p_dump[9] >> i) & 0x01) ? 1 : 0;
    }
}

/**************************************************************************//**
 * \fn void Phonecard::readCounter8(uint8_t* p_dump)
 *
 * \brief Read counter 8.
 *
 * \param[in]   p_dump  Memory dump.
 ******************************************************************************/
void Phonecard::readCounter8(uint8_t* p_dump)
{
    m_counter8 = 0;
    for (uint8_t i = 0; i < 8; i++)
    {
        m_counter8 += ((p_dump[10] >> i) & 0x01) ? 1 : 0;
    }
}

/**************************************************************************//**
 * \fn void Phonecard::readCounter1(uint8_t* p_dump)
 *
 * \brief Read counter 1.
 *
 * \param[in]   p_dump  Memory dump.
 ******************************************************************************/
void Phonecard::readCounter1(uint8_t* p_dump)
{
    m_counter1 = 0;
    for (uint8_t i = 0; i < 8; i++)
    {
        m_counter1 += ((p_dump[11] >> i) & 0x01) ? 1 : 0;
    }
}

/**************************************************************************//**
 * \fn void Phonecard::computeRemaningUnitCount()
 *
 * \brief Compute remaining unit count.
 ******************************************************************************/
void Phonecard::computeRemaningUnitCount()
{
    m_remainingUnits = m_unitNumber + PHONECARD__TEST_UNIT_COUNT -
        (m_counter512 * 512 + m_counter64 * 64 + m_counter8 * 8 + m_counter1 * 1);
}

/**************************************************************************//**
 * \fn uint8_t Phonecard::readByte()
 *
 * \brief Read a byte.
 *
 * \return Read byte.
 ******************************************************************************/
uint8_t Phonecard::readByte()
{
    if (m_rwPin != UINT8_MAX)
    {
        digitalWrite(m_rwPin, LOW);
    }
    
    digitalWrite(m_resetPin, HIGH);

    uint8_t data = 0;
    for (uint8_t i = 0; i < 8; i++)
    {
        digitalWrite(m_clockPin, LOW);
        delay(5);
        data |= (digitalRead(m_ioPin) << (7 - i));
        digitalWrite(m_clockPin, HIGH);
        delay(5);
    }

    return data;
}

