/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import frc.systems.Drivetrain;
import frc.systems.sensors.Limelight;
import com.analog.adis16448.frc.ADIS16448_IMU;


public class Robot extends TimedRobot {
  public static Joystick rightJoystick;
  public static Joystick leftJoystick;
  public static Joystick xboxJoystick;

  public static Timer systemTimer;
  public static Limelight mLimelight;
  
  private boolean m_runCal = false;
  private boolean m_configCal = false;
  private boolean m_reset = false;
  private boolean m_setYawAxis = false;
  public static ADIS16448_IMU m_imu = new ADIS16448_IMU();
  public static VisualOdometer m_visOdometer = new VisualOdometer();

  Notifier driveRateGroup;
  public static Drivetrain mDriveSystem;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    mLimelight = new Limelight();

    leftJoystick = new Joystick(0);
    rightJoystick = new Joystick(1);
    xboxJoystick = new Joystick(2);
    
    // Set IMU settings
    if (m_configCal) {
      m_imu.configCalTime(8);
      m_configCal = SmartDashboard.putBoolean("ConfigCal", false);
    }
    if (m_reset) {
      m_imu.reset();
      m_reset = SmartDashboard.putBoolean("Reset", false);
    }
    if (m_runCal) {
      m_imu.calibrate();
      m_runCal = SmartDashboard.putBoolean("RunCal", false);
    }

  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

	m_visOdometer.update();

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {

	m_visOdometer.update();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
