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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisualOdometer {

    private final OpticalFlowSensor m_optFlow_R;
    //private final OpticalFlowSensor m_optFlow_L;
    private double m_gyroInitialHeading;  // Subtract from raw gyro to get field frame heading field frame
    private double m_gyroHeadingDegrees;  // field frame direction robot is pointing, but it might be skidding sideways
    private Pose2d m_currentPose;
 
    private final double PI = 3.14159265358979323846264;
    private final double CONVERT_RADIANS_TO_DEGREES = (180.0 / PI);
    private final double CONVERT_FLOW_TICKS_TO_INCHES = 23.45;  // TODO: Need to calibrate this
  
    public VisualOdometer()
    {
        m_optFlow_R = new OpticalFlowSensor(SPI.Port.kOnboardCS0);
        if(!m_optFlow_R.reset())
        {
            DriverStation.reportError("Optical Flow( R ) not found", false);
        }
        //m_optFlow_L = new OpticalFlowSensor(SPI.Port.kOnboardCS1);
        //if(!m_optFlow_L.reset())
        {
        //    DriverStation.reportError("Optical Flow( L ) not found", false);
        }
       m_currentPose = new Pose2d();
     }

    public void initPose(final Pose2d initialPositionAndDirection) {
        //m_gyroInitialHeading = Robot.m_imu.getAngle();
        m_currentPose = initialPositionAndDirection;
    }

    public void update() {
        m_optFlow_R.getMotion();
        //m_optFlow_L.getMotion();
        SmartDashboard.putNumber("OptR_X", m_optFlow_R.deltaX);
        SmartDashboard.putNumber("OptR_Y", m_optFlow_R.deltaY);
        //SmartDashboard.putNumber("OptL_X", m_optFlow_L.deltaX);
        //SmartDashboard.putNumber("OptL_Y", m_optFlow_L.deltaY);
        
        //m_gyroHeadingDegrees = Robot.m_imu.getAngle() - m_gyroInitialHeading;

        // sensors are on opposite sides of robot center, so when the robot rotates they cancel, and when it drives straight they add
        final double robotFrameDeltaX = 0;  //((m_optFlow_R.deltaX + m_optFlow_L.deltaX) / 2) / CONVERT_FLOW_TICKS_TO_INCHES;
        final double robotFrameDeltaY = 0;  //((m_optFlow_R.deltaY + m_optFlow_L.deltaY) / 2) / CONVERT_FLOW_TICKS_TO_INCHES;
        
        final double robotFrameMagnitude = Math.sqrt(Math.pow(robotFrameDeltaX, 2) + Math.pow(robotFrameDeltaY, 2));
        double robotFrameDirectionRadians = 0.0;  //Math.atan(robotFrameDeltaX / robotFrameDeltaY);
        if(robotFrameDeltaX < 0.0)
        {
            // 0.0 degrees is straight ahead in robot frame, 180.0 is straight back
            robotFrameDirectionRadians += PI;
        }
        final double fieldFrameDirection = (m_gyroHeadingDegrees * CONVERT_RADIANS_TO_DEGREES) + robotFrameDirectionRadians;

        final double fieldFrameDeltaX = Math.asin(fieldFrameDirection) * robotFrameMagnitude;
        final double fieldFrameDeltaY = Math.acos(fieldFrameDirection) * robotFrameMagnitude;  
 
        final Translation2d xlat = new Translation2d(fieldFrameDeltaX, fieldFrameDeltaY);
        final Transform2d xform = new Transform2d(xlat, Rotation2d.fromDegrees(fieldFrameDirection));
         m_currentPose.transformBy(xform);
         SmartDashboard.putNumber("Current Pose X", m_currentPose.getTranslation().getX());
         SmartDashboard.putNumber("Current Pose Y", m_currentPose.getTranslation().getY());
         SmartDashboard.putNumber("Current Pose HDG", m_currentPose.getRotation().getDegrees());
    }

    public Pose2d currentPose() {
        return m_currentPose;
    } 
}
