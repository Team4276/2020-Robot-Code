//*****************************************************************************************//
// The MIT License (MIT)                                                                   //
//                                                                                         //
// Copyright (c) 2017 - Marina High School FIRST Robotics Team 4276 (Huntington Beach, CA) //
//                                                                                         //
// Permission is hereby granted, free of charge, to any person obtaining a copy            //
// of this software and associated documentation files (the "Software"), to deal           //
// in the Software without restriction, including without limitation the rights            //
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell               //
// copies of the Software, and to permit persons to whom the Software is                   //
// furnished to do so, subject to the following conditions:                                //
//                                                                                         //
// The above copyright notice and this permission notice shall be included in              //
// all copies or substantial portions of the Software.                                     //
//                                                                                         //
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR              //
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,                //
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE             //
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER                  //
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,           //
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN               //
// THE SOFTWARE.                                                                           //
//*****************************************************************************************//
//*****************************************************************************************//
// We are a high school robotics team and always in need of financial support.             //
// If you use this software for commercial purposes please return the favor and donate     //
// (tax free) to "Marina High School Educational Foundation, attn: FRC team 4276"          //
// (Huntington Beach, CA)                                                                  //
//*****************************************************************************************//
package frc.systems.sensors;

import edu.wpi.first.wpilibj.SPI;

public class OpticalFlowSensor extends SPI {
    public short deltaX = 0;
    public short deltaY = 0;

    public OpticalFlowSensor(SPI.Port spiChipSelet) {
        super(spiChipSelet);
    }
    
    public void getMotion()
    {
        final byte[] dataToSend = {0x02,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
        final byte[] dataReceived = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
        final int rv = transaction(dataToSend, dataReceived, 12);
        if(rv != -1) {
            deltaX = (short) ((dataReceived[4] << 8) | (dataReceived[3] & 0xFF));
            deltaY = (short) ((dataReceived[6] << 8) | (dataReceived[5] & 0xFF));
        }
}
 }
