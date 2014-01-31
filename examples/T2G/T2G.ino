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
 * \file T2G.ino
 ******************************************************************************/

/******************************************************************************
 * Header file inclusions.
 ******************************************************************************/

#include <Arduino.h>

#include <Phonecard.h>

/******************************************************************************
 * Private variable declarations.
 ******************************************************************************/

static Phonecard card(3, 4, 5, 6);

static const uint8_t cardSwitch = 2;

static const uint8_t led = 13;

static volatile bool cardInserted = false;

/******************************************************************************
 * Public function definitions.
 ******************************************************************************/

/**************************************************************************//**
 * \fn void setup()
 ******************************************************************************/
void setup()   
{       
    // Initialize Serial.
    Serial.begin(9600); 
    
    // Configure pins.
    pinMode(cardSwitch, INPUT);
    pinMode(led, OUTPUT);
    
    // Look for card.
    if (digitalRead(cardSwitch) == HIGH)
    {
        Serial.println("Card inserted!");
        cardInserted = true;
    }

    // Initialize phonecard.
    card.initialize();
    
    // Attach interrupt raised when a card is inserted or removed.
    attachInterrupt(0, intHandler, CHANGE);
}

/**************************************************************************//**
 * \fn void loop()
 ******************************************************************************/
void loop ()
{
    if (cardInserted)
    {
        Serial.println("Initialize card...");
        card.reset();
        
        Serial.println("Read card...");
        card.read();
        
        Serial.print("Code pro = 0x");
        Serial.println(card.getCodePro(), HEX);
        
        char* p_serialNumber = NULL;
        card.getSerialNumber(&p_serialNumber);
        Serial.print("Serial number = ");
        while (*p_serialNumber != '\0')
        {
            Serial.print(*p_serialNumber);
            p_serialNumber++;
        }
        Serial.println("");
    
        Serial.print("Service code = ");
        Serial.println(card.getServiceCode(), DEC);
        
        Serial.print("Units = ");
        Serial.println(card.getUnitNumber(), DEC);

        Serial.print("Counter 512 = ");
        Serial.println(card.getCounter512(), DEC);

        Serial.print("Counter 64 = ");
        Serial.println(card.getCounter64(), DEC);
        
        Serial.print("Counter 8 = ");
        Serial.println(card.getCounter8(), DEC);

        Serial.print("Counter 1 = ");
        Serial.println(card.getCounter1(), DEC);

        Serial.print("Number of remaining units = ");
        Serial.println(card.getRemainingUnitCount(), DEC);
        
        Serial.println("");
    }
    else
    {
        Serial.println("No card.");
    }
    
    delay(5000);
}

/**************************************************************************//**
 * \fn void intHandler()
 ******************************************************************************/
void intHandler()
{
  cardInserted = !cardInserted;
  if (cardInserted)
  {
     digitalWrite(led, HIGH);
     Serial.println("Card inserted!");
  }
  else
  {
     digitalWrite(led, LOW);
     Serial.println("Card removed!");
  }
  delay(50);
}

