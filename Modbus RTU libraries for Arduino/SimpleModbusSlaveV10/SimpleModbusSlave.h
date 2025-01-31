#ifndef SIMPLE_MODBUS_SLAVE_H
#define SIMPLE_MODBUS_SLAVE_H

/* 
   SimpleModbusSlave allows you to communicate
   with any slave using the Modbus RTU protocol.
   
   Copyright (C) 2011 Helton Duarte
   
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
   
   Modified by S. Cox for compatibility with Arduino 1.0
*/

#include "Arduino.h"
#include "HardwareSerial.h"

// Modbus haberleşme sabitleri
#define BUFFER_SIZE 128

// Global değişkenler
extern unsigned int registerOffset;  // Register offset değeri

// Modbus fonksiyonları
void modbus_configure(HardwareSerial *SerialPort,
                     long baud,
                     unsigned char byteFormat,
                     unsigned char _slaveID, 
                     unsigned char _TxEnablePin, 
                     unsigned int _holdingRegsSize,
                     unsigned int* _regs,
                     unsigned int _regOffset);

void modbus_update();

// Yardımcı fonksiyonlar
void float_to_registers(float value, uint16_t low_reg, uint16_t high_reg, uint16_t* regs);
float registers_to_float(uint16_t low_reg, uint16_t high_reg, uint16_t* regs);

#endif
