/*
 * RoboPeak RPLIDAR Driver for Arduino
 * RoboPeak.com
 * 
 * Copyright (c) 2014, RoboPeak 
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY 
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR 
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "RPLidar.h"

RPLidar::RPLidar()
{
    _currentMeasurement.distance = 0;
    _currentMeasurement.angle = 0;
    _currentMeasurement.quality = 0;
    _currentMeasurement.startBit = 0;
}


RPLidar::~RPLidar()
{
    end();
}

// open the given serial interface and try to connect to the RPLIDAR
bool RPLidar::begin(HardwareSerial &serialobj)
{
    Serial.println("Begin Setup RPSerial !!!!!!!!!");
    // if (isOpen()) {
    //   end(); 
    // }
    _bined_serialdev = &serialobj;
    // _bined_serialdev->end();
    _bined_serialdev->begin(RPLIDAR_S2M1_SERIAL_BAUDRATE);
    Serial.println("End RPSerial !!!!!!!!!");
}

// close the currently opened serial interface
void RPLidar::end()
{
    if (isOpen()) {
       _bined_serialdev->end();
       _bined_serialdev = NULL;
    }
}


// check whether the serial interface is opened
bool RPLidar::isOpen()
{
    return _bined_serialdev?true:false; 
}

// ask the RPLIDAR for its health info
u_result RPLidar::getHealth(rplidar_response_device_health_t & healthinfo, _u32 timeout)
{
    _u32 currentTs = millis();
    _u32 remainingtime;
  
    _u8 *infobuf = (_u8 *)&healthinfo;
    _u8 recvPos = 0;

    rplidar_ans_header_t response_header;
    u_result  ans;


    if (!isOpen()) return RESULT_OPERATION_FAIL;

    // {
        if (IS_FAIL(ans = _sendCommand(RPLIDAR_CMD_GET_DEVICE_HEALTH, NULL, 0))) {
            return ans;
        }

        if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
            return ans;
        }

        // verify whether we got a correct header
        if (response_header.type != RPLIDAR_ANS_TYPE_DEVHEALTH) {
            return RESULT_INVALID_DATA;
        }

        if ((response_header.size) < sizeof(rplidar_response_device_health_t)) {
            return RESULT_INVALID_DATA;
        }
        
        while ((remainingtime=millis() - currentTs) <= timeout) {
            int currentbyte = _bined_serialdev->read();
            if (currentbyte < 0) continue;
            
            infobuf[recvPos++] = currentbyte;

            if (recvPos == sizeof(rplidar_response_device_health_t)) {
                return RESULT_OK;
            }
        }
    // }
    return RESULT_OPERATION_TIMEOUT;
}

// ask the RPLIDAR for its device info like the serial number
u_result RPLidar::getDeviceInfo(rplidar_response_device_info_t & info, _u32 timeout )
{
    _u8  recvPos = 0;
    _u32 currentTs = millis();
    _u32 remainingtime;
    _u8 *infobuf = (_u8*)&info;
    rplidar_ans_header_t response_header;
    u_result  ans;

    if (!isOpen()) return RESULT_OPERATION_FAIL;

    {
        if (IS_FAIL(ans = _sendCommand(RPLIDAR_CMD_GET_DEVICE_INFO,NULL,0))) {
            return ans;
        }

        if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
            return ans;
        }

        // verify whether we got a correct header
        if (response_header.type != RPLIDAR_ANS_TYPE_DEVINFO) {
            return RESULT_INVALID_DATA;
        }

        if (response_header.size < sizeof(rplidar_response_device_info_t)) {
            return RESULT_INVALID_DATA;
        }

        while ((remainingtime=millis() - currentTs) <= timeout) {
            _u8 currentbyte = _bined_serialdev->read();
            if (currentbyte<0) continue;    
            infobuf[recvPos++] = currentbyte;

            if (recvPos == sizeof(rplidar_response_device_info_t)) {
                return RESULT_OK;
            }
        }
    }
    
    return RESULT_OPERATION_TIMEOUT;
}

// reset lidar to recover 
void RPLidar::resetLidar(){
    
    u_result ans = _sendCommand(RPLIDAR_CMD_RESET,NULL,0);
    // return ans;
}
// stop the measurement operation
u_result RPLidar::stop()
{
    if (!isOpen()) return RESULT_OPERATION_FAIL;
    u_result ans = _sendCommand(RPLIDAR_CMD_STOP,NULL,0);
    return ans;
}

// start the measurement operation
u_result RPLidar::startScan(bool force, _u32 timeout)
{
    u_result ans;
    // uint8_t cmd;
    if (!isOpen()) return RESULT_OPERATION_FAIL;
    
    // stop(); //force the previous operation to stop

    {
        // ans = _sendCommand(force?RPLIDAR_CMD_FORCE_SCAN:RPLIDAR_CMD_SCAN, NULL, 0);
         ans = _sendCommand(RPLIDAR_CMD_SCAN, NULL, 0);
        if (IS_FAIL(ans)) {
            // Serial.println("Send start fail!!!!!");
            return ans;
        }
        // waiting for confirmation
        rplidar_ans_header_t response_header;
        if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
            // Serial.println("check header fail!!!!!");
            return ans;
        }
        else{
            // Serial.println("Check Header Successfull!!!!!");
        }

        // verify whether we got a correct header
        if (response_header.type != RPLIDAR_ANS_TYPE_MEASUREMENT) {
            return RESULT_INVALID_DATA;
        } else {
            // Serial.println("Check Type Header Successfull!!!!!");
        }

        if (response_header.size < sizeof(rplidar_response_measurement_node_t)) {
            return RESULT_INVALID_DATA;
        }
        else {
            // Serial.println("Check Size Header Successfull!!!!!");
        }
    }

    // Serial.println("Send Done!!!!!!!");
    return RESULT_OK;
}

// wait for one sample point to arrive
u_result RPLidar::waitPoint(_u32 timeout)
{
   _u32 currentTs = millis();
   _u32 remainingtime;
//    rplidar_response_measurement_node_t node;
//    _u8 *nodebuf = (_u8*)&node;
  uint8_t response[5];
   
   while ((remainingtime=millis() - currentTs) <= timeout) {
    if(_bined_serialdev->available() >= 5){
            _bined_serialdev->readBytes(response,5);

            uint8_t S = response[0] & 0x01;
            uint8_t S_dash = (response[0] >> 1) & 0x01;
            if (S == S_dash) {
                continue;
            }

            uint8_t C = response[1] & 0x01;
            if (C != 1) {
                continue;;
            }

            if (S == 1) {
                 _currentMeasurement.startBit = 1;
            } else {
                 _currentMeasurement.startBit = 0;
            }

             _currentMeasurement.angle = ((response[1] >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) | (response[2] << 7)) / 64.0;
             _currentMeasurement.distance = (response[3] | (response[4] << 8)) / 4.0;
  
            return RESULT_OK;
        } else{
            continue;
        }
    
        
   }

   return RESULT_OPERATION_TIMEOUT;
}



u_result RPLidar::_sendCommand(_u8 cmd, const void * payload, size_t payloadsize)
{

    rplidar_cmd_packet_t pkt_header;
    rplidar_cmd_packet_t * header = &pkt_header;
    _u8 checksum = 0;

    if (payloadsize && payload) {
        cmd |= RPLIDAR_CMDFLAG_HAS_PAYLOAD;
    }

    header->syncByte = RPLIDAR_CMD_SYNC_BYTE;
    header->cmd_flag = cmd;

    // send header first
    _bined_serialdev->write( (uint8_t *)header, 2);

    if (cmd & RPLIDAR_CMDFLAG_HAS_PAYLOAD) {
        checksum ^= RPLIDAR_CMD_SYNC_BYTE;
        checksum ^= cmd;
        checksum ^= (payloadsize & 0xFF);

        // calc checksum
        for (size_t pos = 0; pos < payloadsize; ++pos) {
            checksum ^= ((_u8 *)payload)[pos];
        }

        // send size
        _u8 sizebyte = payloadsize;
        _bined_serialdev->write((uint8_t *)&sizebyte, 1);

        // send payload
        _bined_serialdev->write((uint8_t *)&payload, sizebyte);

        // send checksum
        _bined_serialdev->write((uint8_t *)&checksum, 1);

    }
    // Serial.println("Sent cmd Done!!!!!");
    return RESULT_OK;
}

u_result RPLidar::_waitResponseHeader(rplidar_ans_header_t * header, _u32 timeout)
{
    _u8  recvPos = 0;
    _u32 currentTs = millis();
    _u32 remainingtime;
    _u8 *headerbuf = (_u8*)header;
    while ((remainingtime=millis() - currentTs) <= timeout) {
        
        _u8 currentbyte = _bined_serialdev->read();
        // Serial.print("Header Byte: ");
        // Serial.println(currentbyte, HEX);
        if (currentbyte<0) continue;
        switch (recvPos) {
        case 0:
            if (currentbyte != RPLIDAR_ANS_SYNC_BYTE1) {
                continue;
            }
            break;
        case 1:
            if (currentbyte != RPLIDAR_ANS_SYNC_BYTE2) {
                recvPos = 0;
                continue;
            }
            break;
        }
        headerbuf[recvPos++] = currentbyte;

        if (recvPos == sizeof(rplidar_ans_header_t)) {
            return RESULT_OK;
        }

  

    }
    // Serial.println("Time out!!!!!!!!");
    return RESULT_OPERATION_TIMEOUT;
}

