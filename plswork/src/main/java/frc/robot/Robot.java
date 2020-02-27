/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import com.analog.adis16448.frc.ADIS16448_IMU;

import frc.systems.sensors.Cameras;
import frc.systems.sensors.VisualOdometer;
//import frc.systems.sensors.ColorSen;
import frc.systems.sensors.Limelight;

import frc.utilities.RoboRioPorts;

import frc.systems.Drivetrain;
import frc.systems.Intake;
import frc.systems.Shooter;
import frc.systems.Turntable;
import frc.systems.ArmPivot;



/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static Joystick leftJoystick;
  public static Joystick rightJoystick;
  public static Joystick xboxJoystick;

  public static boolean isEnabled;

  public static Timer systemTimer;
  public static Limelight mLimelight;
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   * 
   * 
   */

   Notifier armGroup;
   public static ArmPivot mArmPivot;

  Notifier driveRateGroup;
  public static Drivetrain mDrivetrain;

  Notifier intakeRateGroup;
  public static Intake mIntake;

  Notifier shooterRateGroup;
  public static Shooter mShooter;

  Notifier turntableRateGroup;
  public static Turntable mTurntable;

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  @Override
  public void robotInit() {
    systemTimer = new Timer();

    leftJoystick = new Joystick(0);
    rightJoystick = new Joystick(1);
    xboxJoystick = new Joystick(2);

    systemTimer = new Timer();
    mLimelight = new Limelight();
    // m_ColorSen = new ColorSen(i2cPort);

    //robotCameraSystem = new Cameras();

    leftJoystick = new Joystick(0);
    rightJoystick = new Joystick(1);
    xboxJoystick = new Joystick(2);

    mDrivetrain = new Drivetrain(true, RoboRioPorts.CAN_DRIVE_L1, RoboRioPorts.CAN_DRIVE_L2, RoboRioPorts.CAN_DRIVE_L3,
        RoboRioPorts.CAN_DRIVE_R1, RoboRioPorts.CAN_DRIVE_R2, RoboRioPorts.CAN_DRIVE_R3,
        RoboRioPorts.DRIVE_DOUBLE_SOLENOID_FWD, RoboRioPorts.DRIVE_DOUBLE_SOLENOID_REV, RoboRioPorts.DIO_DRIVE_RIGHT_A,
        RoboRioPorts.DIO_DRIVE_RIGHT_B, RoboRioPorts.DIO_DRIVE_LEFT_A, RoboRioPorts.DIO_DRIVE_LEFT_B);

    mIntake = new Intake(RoboRioPorts.CAN_INTAKE_UP, RoboRioPorts.CAN_BALL_TRANSFER);// RoboRioPorts.CAN_INTAKE_PIV
       //);// RoboRioPorts.CAN_BALL_TRANSFER, RoboRioPorts.TRANSER_PISTON_REV, RoboRioPorts.TRANSFER_PISTON_FWD);

    mShooter = new Shooter(RoboRioPorts.CAN_SHOOTER_SHOOTA, RoboRioPorts.CAN_SHOOTER_SHOOTB,
        RoboRioPorts.CAN_SHOOTER_FLY, RoboRioPorts.TRANSER_PISTON_REV, RoboRioPorts.TRANSFER_PISTON_FWD,
        RoboRioPorts.CAN_BALL_TRANSFER);

    mArmPivot = new ArmPivot(RoboRioPorts.CAN_INTAKE_PIV);
/*
    mTurntable = new Turntable(RoboRioPorts.CAN_TURNTABLE_TURN, RoboRioPorts.CODY_PISTON_FWD,
        RoboRioPorts.CODY_PISTON_REV);
*/
    driveRateGroup = new Notifier(mDrivetrain::operatorDrive);
    intakeRateGroup = new Notifier(mIntake::performMainProcessing);
    shooterRateGroup = new Notifier(mShooter::performMainProcessing);
    armGroup = new Notifier(mArmPivot::performMainProcessing);
    //turntableRateGroup = new Notifier(mTurntable::performMainProcessing);

    driveRateGroup.startPeriodic(0.05);
    intakeRateGroup.startPeriodic(0.1);
    shooterRateGroup.startPeriodic(0.1);
    armGroup.startPeriodic(0.1);
    //turntableRateGroup.startPeriodic(0.1);
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}