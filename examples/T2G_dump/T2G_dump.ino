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
 * \file T2G_dump.ino
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

static bool cardInserted = false;

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
    
    if (digitalRead(cardSwitch) == HIGH)
    {
        Serial.println("Card inserted!");
        cardInserted = true;
    }

    // Initialize phonecard.
    card.initialize();
    
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
        
        Serial.println("Dump memory...");
        uint8_t buffer[PHONECARD__MEMORY_SIZE] = { 0 };
        card.dumpMemory(buffer);
        
        for (uint8_t i = 0; i < PHONECARD__MEMORY_SIZE; i++)
        {
            Serial.print("@0x");
            Serial.print(i, HEX);
            Serial.print(" = ");
            Serial.print(buffer[i], BIN);
            Serial.println("b0");
        }
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

