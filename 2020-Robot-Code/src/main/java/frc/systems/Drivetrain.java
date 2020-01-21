
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.systems;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Notifier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;

import frc.utilities.Constants;
import frc.utilities.SoftwareTimer;
import frc.utilities.Toggler;
import frc.utilities.LogJoystick;

public class Drivetrain {
    private Encoder m_left_encoder;
    private Encoder m_right_encoder;

    private boolean hasCANNetwork = false;

    DoubleSolenoid gearShifter;
    public final int HI_SHIFTER = 4;
    public final int LO_SHIFTER = 3;
    SoftwareTimer shiftTimer;
    SoftwareTimer driveTimer;
    private boolean shiftInit = true;
    private boolean isShifting = false;
    private boolean isDeploying = false;
    private Gear currentGear = Gear.HI;
    private boolean brakeModeisEngaged = true;
    private final DriveMode DEFAULT_MODE = DriveMode.TANK;
    private DriveMode currentMode = DEFAULT_MODE;
    private String currentMode_s = "Arcade";

    VictorSP flDrive, mlDrive, blDrive, frDrive, mrDrive, brDrive;
    VictorSPX flDriveX, mlDriveX, blDriveX, frDriveX, mrDriveX, brDriveX;
    private double leftPower = 0;
    private double rightPower = 0;
    private double hatchDeployHiGear = -0.25;
    private double hatchDeployLoGear = -0.5;
    private double climbDelayTime = 1.5;// seconds
    private double driveFDelay = 0.8;
    private double retractDelayTime = 2;// seconds
    private double pullupDelayTime = 1;// seconds
    private double contDelayTime = 1.5;// seconds
    private double climbPower = .1;// seconds
    private int timerNum = 0;
    double desired_heading = 0;
    int count = 0;
    private Toggler modeToggler;
    public boolean methodInit = true;
    private Notifier m_follower_notifier;

    double deadband = 0.05;

    // Cheesy Drive Constants
    public static final double kDefaultQuickStopThreshold = 0.2;
    public static final double kDefaultQuickStopAlpha = 0.1;

    private double m_quickStopThreshold = kDefaultQuickStopThreshold;
    private double m_quickStopAlpha = kDefaultQuickStopAlpha;
    private double m_quickStopAccumulator;
    
    public Drivetrain(boolean isCAN, int FLport, int MLport, int BLport, int FRport, int MRport, int BRport,
            int shifterHi, int shifterLo, int m_right_encoderPortA, int m_right_encoderPortB, int m_left_encoderPortA,
            int m_left_encoderPortB) {

        modeToggler = new Toggler(LogJoystick.B1);
        modeToggler.setMechanismState(true); // sets to bracke mode

        shiftTimer = new SoftwareTimer();
        driveTimer = new SoftwareTimer();
        m_right_encoder = new Encoder(m_right_encoderPortA, m_right_encoderPortB);
        m_left_encoder = new Encoder(m_left_encoderPortA, m_left_encoderPortB);
        m_right_encoder.reset();
        m_left_encoder.reset();
        if (isCAN) {
            hasCANNetwork = true;

            flDriveX = new VictorSPX(FLport);
            mlDriveX = new VictorSPX(MLport);
            blDriveX = new VictorSPX(BLport);
            frDriveX = new VictorSPX(FRport);
            mrDriveX = new VictorSPX(MRport);
            brDriveX = new VictorSPX(BRport);
        } else {
            hasCANNetwork = false;

            flDrive = new VictorSP(FLport);
            mlDrive = new VictorSP(MLport);
            blDrive = new VictorSP(BLport);
            frDrive = new VictorSP(FRport);
            mrDrive = new VictorSP(MRport);
            brDrive = new VictorSP(BRport);
        }

        gearShifter = new DoubleSolenoid(shifterHi, shifterLo);

    }

    /**
     * 
     * @param rightPow right motor power
     * @param leftPow  left motor power
     */

    public void assignMotorPower(double rightPow, double leftPow) {
        if (hasCANNetwork) {
            SmartDashboard.putBoolean("Drive check", true);
            flDriveX.set(ControlMode.PercentOutput, leftPow);
            mlDriveX.set(ControlMode.PercentOutput, leftPow);
            blDriveX.set(ControlMode.PercentOutput, leftPow);
            frDriveX.set(ControlMode.PercentOutput, rightPow);
            mrDriveX.set(ControlMode.PercentOutput, rightPow);
            brDriveX.set(ControlMode.PercentOutput, rightPow);

        } else {
            flDrive.set(leftPow);
            mlDrive.set(leftPow);
            blDrive.set(leftPow);
            frDrive.set(rightPow);
            mrDrive.set(rightPow);
            brDrive.set(rightPow);
        }
        rightPower = rightPow;
        leftPower = leftPow;
    }

    /**
     * manual operator controlled drive
     */
    protected double limit(double value) {
        if (value > 1.0) {
            return 1.0;
        }
        if (value < -1.0) {
            return -1.0;
        }
        return value;
    }
    public void operatorDrive() {

        changeMode();
        checkForGearShift();

        if (Robot.rightJoystick.getRawButton(1)) {
            currentMode = DriveMode.AUTO;

        }  else {
            currentMode = DEFAULT_MODE;
        }

        isDeploying = false;

        if (currentMode == DriveMode.AUTO) {
            currentMode_s = "Auto";
        } else if (currentMode == DriveMode.ARCADE) {
            currentMode_s = "Arcade";
        } else {
            currentMode_s = "Tank";
        }

        double leftY = 0;
        double rightY = 0;

        switch (currentMode) {

        case AUTO:
            //rotateCam(4, Robot.visionTargetInfo.visionPixelX);
            // driveFwd(4, .25);
            LimelightRotate();


            break;

        case CHEESY:

            resetAuto();

            double xSpeed = 0;
            double zRotation = 0;
            double left = 0;
            double right = 0;
            boolean isQuickTurn;

            if (Math.abs(Robot.rightJoystick.getX()) > deadband) {
                zRotation = -Robot.rightJoystick.getX();
            }
            if (Math.abs(Robot.leftJoystick.getY()) > deadband) {
                xSpeed = Math.pow(Robot.leftJoystick.getY(), 3);
            }

            if (Robot.rightJoystick.getRawButton(2)) {
                isQuickTurn = true;
            } else {
                isQuickTurn = false;
            }

            xSpeed = limit(xSpeed);

            zRotation = limit(zRotation);

            double angularPower;
            boolean overPower;

            if (isQuickTurn) {
                if (Math.abs(xSpeed) < m_quickStopThreshold) {
                    m_quickStopAccumulator = (1 - m_quickStopAlpha) * m_quickStopAccumulator
                            + m_quickStopAlpha * limit(zRotation) * 2;
                }
                overPower = true;
                angularPower = zRotation;
            } else {
                overPower = false;
                angularPower = Math.abs(xSpeed) * zRotation - m_quickStopAccumulator;

                if (m_quickStopAccumulator > 1) {
                    m_quickStopAccumulator -= 1;
                } else if (m_quickStopAccumulator < -1) {
                    m_quickStopAccumulator += 1;
                } else {
                    m_quickStopAccumulator = 0.0;
                }
            }

            left = xSpeed + angularPower;
            right = xSpeed - angularPower;

            // If rotation is overpowered, reduce both outputs to within acceptable range
            if (overPower) {
                if (left > 1.0) {
                    right -= left - 1.0;
                    left = 1.0;
                } else if (right > 1.0) {
                    left -= right - 1.0;
                    right = 1.0;
                } else if (left < -1.0) {
                    right -= left + 1.0;
                    left = -1.0;
                } else if (right < -1.0) {
                    left -= right + 1.0;
                    right = -1.0;
                }
            }

            // Normalize the wheel speeds
            double maxMagnitude = Math.max(Math.abs(left), Math.abs(right));
            if (maxMagnitude > 1.0) {
                left /= maxMagnitude;
                right /= maxMagnitude;

            }
            if (!isShifting) {
                assignMotorPower(right, left);
            } else {

                assignMotorPower(0, 0);
            }

            break;

        case ARCADE:
            resetAuto();
            double linear = 0;
            double turn = 0;

            if (Math.abs(Robot.rightJoystick.getY()) > deadband) {
                linear = -Robot.rightJoystick.getY();
            }
            if (Math.abs(Robot.leftJoystick.getX()) > deadband) {
                turn = Math.pow(Robot.leftJoystick.getX(), 3);
            }

            leftY = -linear - turn;
            rightY = linear - turn;
            if (!isShifting) {
                assignMotorPower(rightY, leftY);
            } else {

                assignMotorPower(0, 0);
            }

            break;

        case TANK:

        //Robot.mLimelight.setCamMode(1);
        //Robot.mLimelight.setLedMode(1);

            resetAuto();
            if (Math.abs(Robot.rightJoystick.getY()) > deadband) {
                rightY = -Math.pow(Robot.rightJoystick.getY(), 3 / 2);
            }
            if (Math.abs(Robot.leftJoystick.getY()) > deadband) {
                leftY = Math.pow(Robot.leftJoystick.getY(), 3 / 2);
            }
            if (!isShifting) {
                assignMotorPower(rightY, leftY);
            } else {

                assignMotorPower(0, 0);
            }
            break;

        default:
            break;
        }

        updateTelemetry();
    }

    /**
     * Checks for joystick input to shift gears. Manages to logic and timing to not
     * power drive motors while shifting
     */
    public void checkForGearShift() {
        boolean shiftHi = Robot.leftJoystick.getRawButton(HI_SHIFTER);
        boolean shiftLo = Robot.leftJoystick.getRawButton(LO_SHIFTER);

        if (shiftHi) {
            currentGear = Gear.HI;
            if (shiftInit) {
                shiftTimer.setTimer(Constants.SHIFT_TIME);
                shiftInit = false;
            }
            if (shiftTimer.isExpired()) {
                isShifting = false;
                shiftInit = true;
            } else {
                isShifting = true;
            }
            gearShifter.set(Constants.HI_GEAR_VALUE);
        } else if (shiftLo) {
            currentGear = Gear.LO;
            if (shiftInit) {
                shiftTimer.setTimer(Constants.SHIFT_TIME);
                shiftInit = false;
            }
            if (shiftTimer.isExpired()) {
                isShifting = false;
                shiftInit = true;
            } else {
                isShifting = true;
            }
            gearShifter.set(Constants.LO_GEAR_VALUE);
        } else {
            isShifting = false;
        }

    }

    /**
     * current gear status
     */

    public enum Gear {
        HI, LO
    }

    public enum DriveMode {
        TANK, ARCADE, AUTO, CHEESY
    }

    /**
     * 
     * @param shiftTo desired gear
     */
    public void shiftGear(Gear shiftTo) {
        boolean shiftHi = false;
        boolean shiftLo = false;

        currentGear = shiftTo;

        if (shiftTo == Gear.HI) {
            shiftHi = true;
        } else {
            shiftLo = true;
        }

        if (shiftHi) {
            if (shiftInit) {
                shiftTimer.setTimer(Constants.SHIFT_TIME);
                shiftInit = false;
            }
            if (shiftTimer.isExpired()) {
                isShifting = false;
                shiftInit = true;
            } else {
                isShifting = true;
            }
            gearShifter.set(Constants.HI_GEAR_VALUE);
        } else if (shiftLo) {
            if (shiftInit) {
                shiftTimer.setTimer(Constants.SHIFT_TIME);
                shiftInit = false;
            }
            if (shiftTimer.isExpired()) {
                isShifting = false;
                shiftInit = true;
            } else {
                isShifting = true;
            }
            gearShifter.set(Constants.LO_GEAR_VALUE);
        } else {
            isShifting = false;
        }

    }

    public void changeMode() {
        modeToggler.updateMechanismStateLJoy();
        brakeModeisEngaged = modeToggler.getMechanismState();
        if (brakeModeisEngaged) {
            flDriveX.setNeutralMode(NeutralMode.Brake);
            mlDriveX.setNeutralMode(NeutralMode.Brake);
            blDriveX.setNeutralMode(NeutralMode.Brake);
            frDriveX.setNeutralMode(NeutralMode.Brake);
            mrDriveX.setNeutralMode(NeutralMode.Brake);
            brDriveX.setNeutralMode(NeutralMode.Brake);
        } else {
            flDriveX.setNeutralMode(NeutralMode.Coast);
            mlDriveX.setNeutralMode(NeutralMode.Coast);
            blDriveX.setNeutralMode(NeutralMode.Coast);
            frDriveX.setNeutralMode(NeutralMode.Coast);
            mrDriveX.setNeutralMode(NeutralMode.Coast);
            brDriveX.setNeutralMode(NeutralMode.Coast);
        }

    }

    /**
     * 
     * @param targetTime      distance to travel
     * @param powerMultiplier from 0 to 1
     * @return status of action (complete)
     */
    
    public void LimelightRotate(){
        
        Robot.mLimelight.RotateTracking();
        assignMotorPower(-Robot.mLimelight.rightSteering, Robot.mLimelight.leftSteering);

     }
    public void LimelightDrive(){

        Robot.mLimelight.DriveTracking();
        assignMotorPower(Robot.mLimelight.rightSteering, Robot.mLimelight.leftSteering);
        
    }

    public void resetAuto() {
        methodInit = true;
        timerNum = 1;
    }

    public boolean autoDriveTest(double desiredCoordinate){

        return false;
    }

    /*
     * () public void reset(){ assignMotorPower(rightPow, leftPow); }
     */
    /**
     * updates smartdashboard
     */
    public void updateTelemetry() {
        SmartDashboard.putNumber("TX", Robot.mLimelight.getTx());
        // encoder outputs
        SmartDashboard.putNumber("Right Encoder", m_right_encoder.getDistance());
        SmartDashboard.putNumber("Left Encoder", m_left_encoder.getDistance());
        // shifting status
        SmartDashboard.putBoolean("Shifting", isShifting);
        SmartDashboard.putString("Drive Mode", currentMode_s);
        // current gear
        SmartDashboard.putBoolean("HI Gear", (currentGear == Gear.HI));
        SmartDashboard.putBoolean("LOW Gear", (currentGear == Gear.LO));
        // power outputs
        SmartDashboard.putNumber("Right Power", rightPower);
        SmartDashboard.putNumber("Left Power", leftPower);

        // Deploy hatch
        SmartDashboard.putBoolean("Hatch Back Up", isDeploying);

        // Check Coast/Brake
        SmartDashboard.putBoolean("Brake Mode", brakeModeisEngaged);
    }

}