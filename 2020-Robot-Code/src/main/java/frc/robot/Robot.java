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
//import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import com.analog.adis16448.frc.ADIS16448_IMU;

import frc.systems.sensors.Cameras;
//import frc.systems.sensors.VisualOdometer;
//import frc.systems.sensors.ColorSen;
import frc.systems.sensors.Limelight;

import frc.utilities.RoboRioPorts;
import frc.utilities.SoftwareTimer;

import frc.systems.Drivetrain;
import frc.systems.Indexer;
import frc.systems.Intake;
import frc.systems.Shooter;
//import frc.systems.Turntable;
import frc.systems.ArmPivot;
//import frc.systems.ArmPiv;
import frc.systems.Climber;

import frc.auton.SelectAuto;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  SoftwareTimer defaultTimer;
  Cameras robotCameraSystem;
  public SelectAuto AutoSelecter;
  private static final String kDefaultAuto = "Default";
  private static final String kstraightShoot = "Lined Up Straight";
  private static final String kDriveOffLine = "Drive off the line";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

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
  Notifier lime;
  Notifier armGroup;
  public static ArmPivot mArmPivot;
  Notifier armPivGroup;
  //public static ArmPiv mArmPiv;

  Notifier driveRateGroup;
  public static Drivetrain mDrivetrain;

  Notifier intakeRateGroup;
  public static Intake mIntake;
  public static Indexer mIndexer;
  Notifier indexerRateGroup;

  Notifier shooterRateGroup;
  public static Shooter mShooter;

  //Notifier turntableRateGroup;
  //public static Turntable mTurntable;
 
  Notifier climberRateGroup;
  public static Climber mClimber;


  //private final I2C.Port i2cPort = I2C.Port.kOnboard;

  boolean isFirstTime = true;

  @Override
  public void robotInit() {
    AutoSelecter = new SelectAuto();
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("Shoot Straight", kstraightShoot);
    m_chooser.addOption("Drive Off Line", kDriveOffLine);
    SmartDashboard.putData("Auto choices", m_chooser);

    systemTimer = new Timer();
    mLimelight = new Limelight();
    systemTimer = new Timer();

    leftJoystick = new Joystick(0);
    rightJoystick = new Joystick(1);
    xboxJoystick = new Joystick(2);

    systemTimer = new Timer();
    //mLimelight = new Limelight();
    // m_ColorSen = new ColorSen(i2cPort);

    //robotCameraSystem = new Cameras();

    //leftJoystick = new Joystick(0);
    //rightJoystick = new Joystick(1);
    //xboxJoystick = new Joystick(2);

    mDrivetrain = new Drivetrain(true, RoboRioPorts.CAN_DRIVE_L1, RoboRioPorts.CAN_DRIVE_L2, RoboRioPorts.CAN_DRIVE_L3,
        RoboRioPorts.CAN_DRIVE_R1, RoboRioPorts.CAN_DRIVE_R2, RoboRioPorts.CAN_DRIVE_R3,
        RoboRioPorts.DRIVE_DOUBLE_SOLENOID_FWD, RoboRioPorts.DRIVE_DOUBLE_SOLENOID_REV, RoboRioPorts.DIO_DRIVE_RIGHT_A,
        RoboRioPorts.DIO_DRIVE_RIGHT_B, RoboRioPorts.DIO_DRIVE_LEFT_A, RoboRioPorts.DIO_DRIVE_LEFT_B);

    mIntake = new Intake(RoboRioPorts.CAN_INTAKE_UP);
   // mIndexer new Indexer(RoboRioPorts.CAN_BALL_TRANSFER);
      mShooter = new Shooter(RoboRioPorts.CAN_SHOOTER_SHOOTA, RoboRioPorts.CAN_SHOOTER_SHOOTB, RoboRioPorts.CAN_SHOOTER_FLY, RoboRioPorts.TRANSER_PISTON_REV, RoboRioPorts.TRANSFER_PISTON_FWD);

    mArmPivot = new ArmPivot(RoboRioPorts.CAN_INTAKE_PIV);
    //mArmPiv = new ArmPiv(RoboRioPorts.CAN_CLIMBER_DUBSOLA, RoboRioPorts.CAN_CLIMBER_DUBSOLB); //using old climber pnue for lowering intake?
    //mArmPiv = new ArmPiv(RoboRioPorts.CAN_BALL_TRANSFER); //ball transfer was to get balls from intake to shooter
    mClimber = new Climber(RoboRioPorts.CAN_CLIMBER_DUBSOLA, RoboRioPorts.CAN_CLIMBER_DUBSOLB);

    /*
     * mTurntable = new Turntable(RoboRioPorts.CAN_TURNTABLE_TURN,
     * RoboRioPorts.CODY_PISTON_FWD, RoboRioPorts.CODY_PISTON_REV);
     */
    //lime = new Notifier(mLimelight::updateTelementry);
    driveRateGroup = new Notifier(mDrivetrain::operatorDrive);
    //intakeRateGroup = new Notifier(mIntake::performMainProcessing);
    //shooterRateGroup = new Notifier(mShooter::performMainProcessing);
    armGroup = new Notifier(mArmPivot::performMainProcessing);
    //climberRateGroup = new Notifier(mClimber::performMainProcessing);
    //indexerRateGroup = new Notifier(mIndexer :: performMainProcessing);
    //armPivGroup = new Notifier(mArmPiv::performMainProcessing);//Zook wrote this!
    // turntableRateGroup = new Notifier(mTurntable::performMainProcessing);

    driveRateGroup.startPeriodic(0.05);
    //intakeRateGroup.startPeriodic(0.1);
    //shooterRateGroup.startPeriodic(0.1);
   // indexerRateGroup.startPeriodic(0.1);
    armGroup.startPeriodic(0.1);
   // lime.startPeriodic(0.1);
   // climberRateGroup.startPeriodic(0.1);
    //armPivGroup.startPeriodic(0.1);//Zook wrote this!
    // turntableRateGroup.startPeriodic(0.1);
  }

  @Override
  public void robotPeriodic() {
    //mLimelight.updateTelementry();
    mDrivetrain.updateTelemetry();
    //mShooter.updateTelemetry();
  }

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  @Override
  public void autonomousPeriodic() {
    if (isFirstTime) {
      isFirstTime = false;
    }

    switch (m_autoSelected) {

    default:
      // Put default auto code here
      // mDrivetrain.assignMotorPower(0.2, -0.2);
      AutoSelecter.performAuto();
      break;

    }

  }

  @Override
  public void teleopInit() {

    isEnabled = true;

    super.teleopInit();
  }

  @Override
  public void teleopPeriodic() {
    // mShooter.updateTelemetry();
    mIntake.performMainProcessing();
  }

  @Override
  public void disabledInit() {
    isEnabled = false;

    super.disabledInit();
  }

  @Override
  public void disabledPeriodic() {
    mDrivetrain.updateTelemetry();
   // mShooter.updateTelemetry();
    mLimelight.updateTelementry();

    super.disabledPeriodic();
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
