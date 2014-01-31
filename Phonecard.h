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
 * \headerfile Phonecard.h
 ******************************************************************************/

#ifndef Phonecard_h
#define Phonecard_h

/******************************************************************************
 * Header file inclusions.
 ******************************************************************************/

#include <stdint.h>

#define PHONECARD__MEMORY_SIZE              64
#define PHONECARD__SIZE_OF_SERIAL_NUMBER    10

/**************************************************************************//**
 * \class Phonecard
 *
 * \brief
 ******************************************************************************/
class Phonecard
{
    public:
    
        /******************************************************************//**
         * \enum ServiceCodes.
         * \brief Service codes.
         *
         * \type ServiceCode_t.
         * \brief Service code.
         **********************************************************************/
        typedef enum ServiceCodes
        {
            T2G,
            Eurostar
        } ServiceCode_t;
    
        /******************************************************************//**
         * \fn Phonecard(uint8_t clockPin, uint8_t resetPin, uint8_t ioPin)
         *
         * \brief Constructor.
         *
         * \param   clockPin    
         * \param   resetPin
         * \param   ioPin
         **********************************************************************/
        Phonecard(uint8_t clockPin, uint8_t resetPin, uint8_t ioPin);
    
        /******************************************************************//**
         * \fn Phonecard(uint8_t rwPin, uint8_t clockPin, uint8_t resetPin, uint8_t ioPin)
         *
         * \brief Constructor.
         *
         * \param   rwPin
         * \param   clockPin    
         * \param   resetPin
         * \param   ioPin
         **********************************************************************/
        Phonecard(uint8_t rwPin, uint8_t clockPin, uint8_t resetPin, uint8_t ioPin);
        
        /******************************************************************//**
         * \fn uint16_t getCodePro()
         *
         * \brief Get code pro.
         *
         * \return Code pro.
         **********************************************************************/
        uint16_t getCodePro();
        
        /******************************************************************//**
         * \fn void :getSerialNumber(char** p_serialNumber)
         *
         * \brief Get serial number.
         *
         * \param[in]   p_serialNumber
         **********************************************************************/
        void getSerialNumber(char** p_serialNumber);
        
        /******************************************************************//**
         * \fn ServiceCode_t getServiceCode()
         *
         * \brief Get service code.
         *
         * \return Service code.
         **********************************************************************/
        ServiceCode_t getServiceCode();
        
        /******************************************************************//**
         * \fn uint8_t getUnitNumber()
         *
         * \brief Get unit number.
         *
         * \return Unit number.
         **********************************************************************/
        uint8_t getUnitNumber();
        
        /******************************************************************//**
         * \fn uint8_t getCounter512()
         *
         * \brief Get counter 512.
         *
         * \return Counter 512.
         **********************************************************************/
        uint8_t getCounter512();
        
        /******************************************************************//**
         * \fn uint8_t getCounter64()
         *
         * \brief Get counter 64.
         *
         * \return Counter 64.
         **********************************************************************/
        uint8_t getCounter64();
        
        /******************************************************************//**
         * \fn uint8_t getCounter8()
         *
         * \brief Get counter 8.
         *
         * \return Counter 8.
         **********************************************************************/
        uint8_t getCounter8();
        
        /******************************************************************//**
         * \fn uint8_t getCounter1()
         *
         * \brief Get counter 1.
         *
         * \return Counter 1.
         **********************************************************************/
        uint8_t getCounter1();
        
        /******************************************************************//**
         * \fn uint8_t getRemainingUnitCount()
         *
         * \brief Get remaining unit count.
         *
         * \return Remaining unit count.
         **********************************************************************/
        uint8_t getRemainingUnitCount();
        
        /******************************************************************//**
         * \fn void initialize()
         *
         * \brief Initialize library.
         * Configure the pins used by the library.
         **********************************************************************/
        void initialize();

        /******************************************************************//**
         * \fn void reset()
         *
         * \brief Reset phonecard.
         **********************************************************************/
        void reset();

        /******************************************************************//**
         * \fn void read()
         *
         * \brief Read phonecard memory.
         * Dump complete memory and read it.
         **********************************************************************/
        void read();
        
        /******************************************************************//**
         * \fn dumpMemory()
         *
         * \brief Dump memory.
         *
         * \param[in]   p_buffer    Buffer to store memory dump. Must not be null.
         **********************************************************************/
        void dumpMemory(uint8_t* p_buffer);

    private:
        
        /******************************************************************//**
         * \fn uint8_t readByte()
         *
         * \brief Read a byte.
         *
         * \return Read byte.
         **********************************************************************/
        uint8_t readByte();
       
        /******************************************************************//**
         * \fn void readCodePro(uint8_t* p_dump)
         *
         * \brief Read code pro.
         *
         * \param[in]   p_dump  Memory dump.
         **********************************************************************/
        void readCodePro(uint8_t* p_dump);
        
        /******************************************************************//**
         * \fn void readSerialNumber(uint8_t* p_dump)
         *
         * \brief Read serial number.
         *
         * \param[in]   p_dump  Memory dump.
         **********************************************************************/
        void readSerialNumber(uint8_t* p_dump);
        
        /******************************************************************//**
         * \fn void readServiceCode(uint8_t* p_dump)
         *
         * \brief Read service code.
         *
         * \param[in]   p_dump  Memory dump.
         **********************************************************************/
        void readServiceCode(uint8_t* p_dump);
        
        /******************************************************************//**
         * \fn void readUnitNumber(uint8_t* p_dump)
         *
         * \brief Read unit number.
         *
         * \param[in]   p_dump  Memory dump.
         **********************************************************************/
        void readUnitNumber(uint8_t* p_dump);
        
        /******************************************************************//**
         * \fn void readCounter512(uint8_t* p_dump)
         *
         * \brief Read counter 512.
         *
         * \param[in]   p_dump  Memory dump.
         **********************************************************************/
        void readCounter512(uint8_t* p_dump);
        
        /******************************************************************//**
         * \fn void readCounter64(uint8_t* p_dump)
         *
         * \brief Read counter 64.
         *
         * \param[in]   p_dump  Memory dump.
         **********************************************************************/
        void readCounter64(uint8_t* p_dump);
        
        /******************************************************************//**
         * \fn void readCounter8(uint8_t* p_dump)
         *
         * \brief Read counter 8.
         *
         * \param[in]   p_dump  Memory dump.
         **********************************************************************/
        void readCounter8(uint8_t* p_dump);
        
        /******************************************************************//**
         * \fn void readCounter1(uint8_t* p_dump)
         *
         * \brief Read counter 1.
         *
         * \param[in]   p_dump  Memory dump.
         **********************************************************************/
        void readCounter1(uint8_t* p_dump);
        
        /******************************************************************//**
         * \fn void computeRemaningUnitCount()
         *
         * \brief Compute remaining unit count.
         **********************************************************************/
        void computeRemaningUnitCount();
        
        uint8_t m_rwPin;
        uint8_t m_clockPin;
        uint8_t m_resetPin;
        uint8_t m_ioPin;
        
        uint16_t m_codePro;
        char m_serialNumber[PHONECARD__SIZE_OF_SERIAL_NUMBER];
        ServiceCode_t m_serviceCode;
        uint8_t m_counter512;
        uint8_t m_counter64;
        uint8_t m_counter8;
        uint8_t m_counter1;
        uint8_t m_unitNumber;
        uint8_t m_remainingUnits;
};

#endif // Phonecard_h

