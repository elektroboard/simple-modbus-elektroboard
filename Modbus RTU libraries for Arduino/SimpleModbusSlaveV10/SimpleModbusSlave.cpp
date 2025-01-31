/*
 * SimpleModbusSlave.cpp - Arduino library for communicating with Modbus slaves
 * over RS232/USB/485 via RTU protocol.
 * Copyright (C) 2011 Helton Duarte
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "SimpleModbusSlave.h"

// Global değişkenler
unsigned char frame[BUFFER_SIZE];
unsigned int holdingRegsSize; 
unsigned int* regs; 
unsigned int registerOffset = 0;
unsigned char slaveID;
unsigned char function;
unsigned char TxEnablePin;
unsigned int errorCount;
unsigned int T1_5; 
unsigned int T3_5; 
HardwareSerial* ModbusPort;

// Modbus yapılandırma
void modbus_configure(HardwareSerial *SerialPort,
                     long baud,
                     unsigned char byteFormat,
                     unsigned char _slaveID, 
                     unsigned char _TxEnablePin, 
                     unsigned int _holdingRegsSize,
                     unsigned int* _regs,
                     unsigned int _regOffset)
{
    ModbusPort = SerialPort;
    ModbusPort->begin(baud);
    
    slaveID = _slaveID;
    TxEnablePin = _TxEnablePin;
    regs = _regs;
    holdingRegsSize = _holdingRegsSize;
    registerOffset = _regOffset;  // Offset değerini kaydet
    
    switch(byteFormat)
    {
        case 1:  // 8N1
            T1_5 = 750; 
            T3_5 = 1750; 
            break;
        case 2:  // 8E1
            T1_5 = 875; 
            T3_5 = 2045; 
            break;
        case 3:  // 8O1
            T1_5 = 875; 
            T3_5 = 2045; 
            break;
    }
    
    pinMode(TxEnablePin, OUTPUT);
    digitalWrite(TxEnablePin, LOW);
    
    errorCount = 0;
}

// Float değeri registerlara yazma
void float_to_registers(float value, uint16_t low_reg, uint16_t high_reg, uint16_t* regs)
{
    uint32_t temp = *(uint32_t*)&value;
    regs[low_reg] = temp & 0xFFFF;
    regs[high_reg] = temp >> 16;
}

// Registerlardan float değer okuma
float registers_to_float(uint16_t low_reg, uint16_t high_reg, uint16_t* regs)
{
    uint32_t temp = ((uint32_t)regs[high_reg] << 16) | regs[low_reg];
    return *(float*)&temp;
}

// CRC hesaplama
unsigned int calculateCRC(unsigned char bufferSize) 
{
    unsigned int temp, temp2, flag;
    temp = 0xFFFF;
    for (unsigned char i = 0; i < bufferSize; i++) {
        temp = temp ^ frame[i];
        for (unsigned char j = 1; j <= 8; j++) {
            flag = temp & 0x0001;
            temp >>= 1;
            if (flag)
                temp ^= 0xA001;
        }
    }
    // CRC baytlarının yerini değiştir
    temp2 = temp >> 8;
    temp = (temp << 8) | temp2;
    temp &= 0xFFFF;
    return temp;
}

// Hata yanıtı oluşturma
void exceptionResponse(unsigned char exception)
{
    errorCount++; // hata sayacını artır
    frame[2] = exception;
    unsigned int crc = calculateCRC(3); // CRC hesapla
    frame[3] = crc >> 8;
    frame[4] = crc & 0xFF;
    sendResponse(5);
}

// Yanıt gönderme
void sendResponse(unsigned char bufferSize)
{
    digitalWrite(TxEnablePin, HIGH);
    for (unsigned char i = 0; i < bufferSize; i++)
        ModbusPort->write(frame[i]);
    ModbusPort->flush();
    digitalWrite(TxEnablePin, LOW);
}

// Modbus frame'ini işleme
void process_modbus_RTU(unsigned char buffer)
{
    // Modbus RTU frame işleme kodları...
    unsigned int startingAddress;
    unsigned int quantity;
    unsigned int crc;
    unsigned char id;
    unsigned char function;
    unsigned char exception;
    
    id = frame[0];
    function = frame[1];
    
    if (id != slaveID && id != 0) return;
    
    // CRC kontrolü...
    crc = ((frame[buffer - 2] << 8) | frame[buffer - 1]);
    if (calculateCRC(buffer - 2) != crc) {
        return;
    }
    
    // Adres ve miktar hesaplama
    startingAddress = ((frame[2] << 8) | frame[3]);
    startingAddress -= registerOffset;  // Gelen adresten offset'i çıkar
    
    quantity = ((frame[4] << 8) | frame[5]);
    
    // Adres kontrolü
    if (startingAddress < 0 || startingAddress + quantity > holdingRegsSize) {
        exceptionResponse(0x02); // Illegal address
        return;
    }
    
    // Fonksiyon koduna göre işlem
    switch (function) {
        case 3: // Read Holding Registers
            // Okuma işlemleri...
            break;
            
        case 16: // Write Multiple Registers
            // Yazma işlemleri...
            break;
            
        default:
            // Desteklenmeyen fonksiyon
            exceptionResponse(0x01); // Illegal function
            break;
    }
}

// Modbus güncelleme
void modbus_update()
{
    static unsigned char buffer;
    static unsigned char overflow;
    
    if (ModbusPort->available()) {
        // Buffer kontrolü ve frame işleme...
    }
}
