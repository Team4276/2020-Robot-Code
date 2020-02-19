/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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

import frc.auton.PathTrajectory;
import frc.auton.SelectAuto;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  Timer defaultTimer;
  public SelectAuto AutoSelecter;
  private static final String kDefaultAuto = "Default";
  private static final String kstraightShoot = "Lined Up Straight";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  public static Joystick leftJoystick;
  public static Joystick rightJoystick;
  public static Joystick xboxJoystick;

  public static boolean isEnabled;

  public static Timer systemTimer;
  public static Limelight mLimelight;

  private boolean m_runCal = false;
  private boolean m_configCal = false;
  private boolean m_reset = false;
  private boolean m_setYawAxis = false;
  public static ADIS16448_IMU m_imu = new ADIS16448_IMU();
  public static VisualOdometer m_visOdometer = new VisualOdometer();
  // public static ColorSen m_ColorSen;

  public static PathTrajectory mTraj;

  Cameras robotCameraSystem;

  Notifier driveRateGroup;
  public static Drivetrain mDriveSystem;

  Notifier intakeRateGroup;
  public static Intake mIntake;

  Notifier shooterRateGroup;
  public static Shooter mShooter;

  Notifier turntableRateGroup;
  public static Turntable mTurntable;

  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    AutoSelecter = new SelectAuto();
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kstraightShoot);
    SmartDashboard.putData("Auto choices", m_chooser);

    systemTimer = new Timer();
    mLimelight = new Limelight();
    // m_ColorSen = new ColorSen(i2cPort);

    robotCameraSystem = new Cameras();

    leftJoystick = new Joystick(0);
    rightJoystick = new Joystick(1);
    xboxJoystick = new Joystick(2);

    mDriveSystem = new Drivetrain(true, RoboRioPorts.CAN_DRIVE_L1, RoboRioPorts.CAN_DRIVE_L2, RoboRioPorts.CAN_DRIVE_L3,
        RoboRioPorts.CAN_DRIVE_R1, RoboRioPorts.CAN_DRIVE_R2, RoboRioPorts.CAN_DRIVE_R3,
        RoboRioPorts.DRIVE_DOUBLE_SOLENOID_FWD, RoboRioPorts.DRIVE_DOUBLE_SOLENOID_REV, RoboRioPorts.DIO_DRIVE_RIGHT_A,
        RoboRioPorts.DIO_DRIVE_RIGHT_B, RoboRioPorts.DIO_DRIVE_LEFT_A, RoboRioPorts.DIO_DRIVE_LEFT_B);

    mIntake = new Intake(RoboRioPorts.CAN_INTAKE_LOW, RoboRioPorts.CAN_INTAKE_UP, RoboRioPorts.CAN_INTAKE_PIV,
        RoboRioPorts.CAN_BALL_TRANSFER, RoboRioPorts.TRANSER_PISTON_REV, RoboRioPorts.TRANSFER_PISTON_FWD);

    mShooter = new Shooter(RoboRioPorts.CAN_SHOOTER_SHOOTA, RoboRioPorts.CAN_SHOOTER_SHOOTB,
        RoboRioPorts.CAN_SHOOTER_FLY, RoboRioPorts.TRANSER_PISTON_REV, RoboRioPorts.TRANSFER_PISTON_FWD,
        RoboRioPorts.CAN_BALL_TRANSFER);

    mTurntable = new Turntable(RoboRioPorts.CAN_TURNTABLE_TURN, RoboRioPorts.CODY_PISTON_FWD,
        RoboRioPorts.CODY_PISTON_REV);

    driveRateGroup = new Notifier(mDriveSystem::operatorDrive);
    intakeRateGroup = new Notifier(mIntake::performMainProcessing);
    shooterRateGroup = new Notifier(mShooter::performMainProcessing);
    turntableRateGroup = new Notifier(mTurntable::performMainProcessing);

    driveRateGroup.startPeriodic(0.05);
    intakeRateGroup.startPeriodic(0.1);
    shooterRateGroup.startPeriodic(0.1);
    turntableRateGroup.startPeriodic(0.1);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    robotCameraSystem.mainCamera.setExposureHoldCurrent();
    mDriveSystem.methodInit = true;

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
    case kstraightShoot:
      AutoSelecter.setMode("StraightShoot");
      AutoSelecter.selectRoutine();
      break;
    case kDefaultAuto:
    default:
      // Put default auto code here
      mDriveSystem.assignMotorPower(0.5, -0.5);
      defaultTimer.delay(3);
      mDriveSystem.assignMotorPower(0, 0);
      break;
    }
    mLimelight.updateTelementry();
  }

  /**
   * 
   */
  @Override
  public void teleopInit() {
    isEnabled = true;

    super.teleopInit();
  }

  /**
   * 
   */
  @Override
  public void disabledInit() {
    isEnabled = false;

    super.disabledInit();
  }

  /**
   * 
   */
  @Override
  public void disabledPeriodic() {
    mDriveSystem.updateTelemetry();
    mLimelight.updateTelementry();
    mIntake.updateTelemetry();
    mShooter.updateTelemetry();
    super.disabledPeriodic();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    mLimelight.updateTelementry();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    mLimelight.updateTelementry();
  }
}
