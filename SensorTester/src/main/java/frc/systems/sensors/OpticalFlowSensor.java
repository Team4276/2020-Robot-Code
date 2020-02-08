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

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class OpticalFlowSensor extends SPI {
    public short deltaX = 0;
    public short deltaY = 0;

    public OpticalFlowSensor(SPI.Port spiChipSelect) {
        super(spiChipSelect);
    }
    
    private static short toUShort(byte[] buf) {
        return (short) (((buf[0] & 0xFF) << 8) + ((buf[1] & 0xFF) << 0));
      }  
    
      private short readRegisterShort(final int reg) {
        ByteBuffer buf = ByteBuffer.allocate(2);
        buf.order(ByteOrder.BIG_ENDIAN);
        buf.put(0, (byte) 0);
        buf.put(1, (byte) (reg & 0x7f));

        write(buf, 2);
        read(false, buf, 2);
       
        SmartDashboard.putNumber("buf0", buf.array()[0]);
        SmartDashboard.putNumber("buf1", buf.array()[1]);

        return toUShort(buf.array());
    }

    private byte readRegisterByte(final int reg) {
        ByteBuffer buf = ByteBuffer.allocate(2);
        buf.order(ByteOrder.BIG_ENDIAN);
        buf.put(0, (byte) 0);
        buf.put(1, (byte) (reg & 0x7f));

        int nBytes = write(buf, 2);
        nBytes = read(false, buf, 1);
       
        return buf.get();
    }

    private void readAllRegisters() {
        // ByteBuffer buf = ByteBuffer.allocateDirect(2);
        final byte[] buf = new byte[12];
        // buf.order(ByteOrder.BIG_ENDIAN);
        buf[0] = 0;
        buf[1] = 0;
        buf[2] = 0;
        buf[3] = 0;
        buf[4] = 0;
        buf[5] = 0;
        buf[6] = 0;
        buf[7] = 0;
        buf[8] = 0;
        buf[9] = 0;
        buf[10] = 0;
        buf[11] = 0;

        write(buf, 12);
        read(false, buf, 12);

        return;  // toUShort(buf);
    }

    private void writeRegister(final int reg, final int val) {
        // ByteBuffer buf = ByteBuffer.allocateDirect(2);
        final byte[] buf = new byte[2];
        // low byte
        buf[0] = (byte) ((0x80 | reg) | 0x10);
        buf[1] = (byte) (val & 0xff);
        write(buf, 2);
        // high byte
        buf[0] = (byte) (0x81 | reg);
        buf[1] = (byte) (val >> 8);
        write(buf, 2);
    }

    private void initRegisters()
    {
        // Performance Optimization registers
        writeRegister(0x7F, 0x00);
        writeRegister(0x61, 0xAD);
        writeRegister(0x7F, 0x03);
        writeRegister(0x40, 0x00);
        writeRegister(0x7F, 0x05);
        writeRegister(0x41, 0xB3);
        writeRegister(0x43, 0xF1);
        writeRegister(0x45, 0x14);
        writeRegister(0x5B, 0x32);
        writeRegister(0x5F, 0x34);
        writeRegister(0x7B, 0x08);
        writeRegister(0x7F, 0x06);
        writeRegister(0x44, 0x1B);
        writeRegister(0x40, 0xBF);
        writeRegister(0x4E, 0x3F);

        writeRegister(0x7F, 0x08);
        writeRegister(0x65, 0x20);
        writeRegister(0x6A, 0x18);
        writeRegister(0x7F, 0x09);
        writeRegister(0x4F, 0xAF);
        writeRegister(0x5F, 0x40);
        writeRegister(0x48, 0x80);
        writeRegister(0x49, 0x80);
        writeRegister(0x57, 0x77);
        writeRegister(0x60, 0x78);
        writeRegister(0x61, 0x78);
        writeRegister(0x62, 0x08);
        writeRegister(0x63, 0x50);
        writeRegister(0x7F, 0x0A);
        writeRegister(0x45, 0x60);
        writeRegister(0x7F, 0x00);
        writeRegister(0x4D, 0x11);
        writeRegister(0x55, 0x80);
        writeRegister(0x74, 0x1F);
        writeRegister(0x75, 0x1F);
        writeRegister(0x4A, 0x78);
        writeRegister(0x4B, 0x78);
        writeRegister(0x44, 0x08);
        writeRegister(0x45, 0x50);
        writeRegister(0x64, 0xFF);
        writeRegister(0x65, 0x1F);
        writeRegister(0x7F, 0x14);
        writeRegister(0x65, 0x60);
        writeRegister(0x66, 0x08);
        writeRegister(0x63, 0x78);
        writeRegister(0x7F, 0x15);
        writeRegister(0x48, 0x58);
        writeRegister(0x7F, 0x07);
        writeRegister(0x41, 0x0D);
        writeRegister(0x43, 0x14);
        writeRegister(0x4B, 0x0E);
        writeRegister(0x45, 0x0F);
        writeRegister(0x44, 0x42);
        writeRegister(0x4C, 0x80);
        writeRegister(0x7F, 0x10);
        writeRegister(0x5B, 0x02);
        writeRegister(0x7F, 0x07);
        writeRegister(0x40, 0x41);
        writeRegister(0x70, 0x00);

        Timer.delay(0.1);
        writeRegister(0x32, 0x44);
        writeRegister(0x7F, 0x07);
        writeRegister(0x40, 0x40);
        writeRegister(0x7F, 0x06);
        writeRegister(0x62, 0xf0);
        writeRegister(0x63, 0x00);
        writeRegister(0x7F, 0x0D);
        writeRegister(0x48, 0xC0);
        writeRegister(0x6F, 0xd5);
        writeRegister(0x7F, 0x00);
        writeRegister(0x5B, 0xa0);
        writeRegister(0x4E, 0xA8);
        writeRegister(0x5A, 0x50);
        writeRegister(0x40, 0x80);
    }

    boolean reset() {
        /*
        setClockRate(1000000);
        setMSBFirst();
        setClockActiveLow();
        setChipSelectActiveLow();
    */
        // Power on reset
        writeRegister(0x3A, 0x5A);
        Timer.delay(0.5);
        // Test the SPI communication, checking chipId and inverse chipId
        short productId = readRegisterShort(0x00);
        productId &= 0xFF;
        short revId = readRegisterShort(0x01);
        revId &= 0xFF;
        if (productId != 0x49 && revId != 0x49) {
            return false;
        }

        // Reading the motion registers one time
        readRegisterByte(0x02);
        readRegisterByte(0x03);
        readRegisterByte(0x04);
        readRegisterByte(0x05);
        readRegisterByte(0x06);
        Timer.delay(0.1);

        initRegisters();

        return true;
    }

    public void getMotion()
    {
        // Read the motion register (freezes the delta-X and delta-Y register so they can be read)
        byte motionRegister = readRegisterByte(0x02);

        if(0 != (motionRegister & 0x80)) {
            // If Bit 7 is set, Delta_X_L, Delta_X_H, Delta_Y_L, Delta_Y_H, SQUAL and Shutter_Upper registers
            // should be read in sequence to get the accumulated motion.
            //      Note: If Delta_X_L, Delta_X_H, Delta_Y_L and Delta_Y_H registers are not read before the motion
            //      register is read for the second time, the data in Delta_X_L, Delta_X_H, Delta_Y_L and Delta_Y_H
            //      will be lost.   
            deltaX = readRegisterShort(0x03);
            deltaY = readRegisterShort(0x05);
            byte squal = readRegisterByte(0x07);
            byte shutter_upper = readRegisterByte(0x0C);
       }
   }
}

