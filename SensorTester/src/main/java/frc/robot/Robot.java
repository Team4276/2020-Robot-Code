/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.cuforge.libcu.Lasershark;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.systems.sensors.VisualOdometer;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private Joystick m_leftStick;
  private Joystick m_rightStick;

  private boolean m_runCal = false;
  private boolean m_configCal = false;
  private boolean m_reset = false;
  private boolean m_setYawAxis = false;
  //public static final ADIS16448_IMU m_imu = new ADIS16448_IMU();
  public static Lasershark m_lasershark = new Lasershark(0);
  public static VisualOdometer m_visOdometer = new VisualOdometer();

  public static double m_lidarDistanceFeet = 0;

  @Override
  public void robotInit() {
    m_myRobot = new DifferentialDrive(new PWMVictorSPX(0), new PWMVictorSPX(1));
    m_leftStick = new Joystick(0);
    m_rightStick = new Joystick(1);
        
    double initialX = -3.0;
    double initialY = 10.0;
    Rotation2d initialRotationFieldRelative = Rotation2d.fromDegrees(180.0);
    Pose2d initialPose = new Pose2d(initialX, initialY, initialRotationFieldRelative);
    m_visOdometer.initPose(initialPose);
  }

  @Override
  public void teleopPeriodic() {
    m_myRobot.tankDrive(m_leftStick.getY(), m_rightStick.getY());

    m_visOdometer.update();
    m_lidarDistanceFeet = m_lasershark.getDistanceFeet();
    SmartDashboard.putNumber("LaserShark: ", m_lidarDistanceFeet);
  }

  @Override
  public void disabledPeriodic() {
     m_visOdometer.update();
     m_lidarDistanceFeet = m_lasershark.getDistanceFeet();
     SmartDashboard.putNumber("LaserShark: ", m_lidarDistanceFeet);
 }
}
