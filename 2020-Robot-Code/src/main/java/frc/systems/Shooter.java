/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.systems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Robot;
import frc.utilities.Xbox;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.Timer;

public class Shooter {
    CANSparkMax sideShootA;
    CANSparkMax sideShootB;
    TalonSRX flyWheel;
    Timer delayer;

    DoubleSolenoid transferPiston;

    CANEncoder encoderSideA;
    CANEncoder encoderSideB;

    CANPIDController pidA, pidB;

    double kP, kI, kD, kIz, kFF, kMax, kMin;
    public double shooterP = 0.0000090;
    public double shooterI = 0.0000002;
    public double shooterD = 0.0000009;
    public double shooterF = 0.00019;
    boolean usePID = false; // set to false if pid is not working properly

    double desiredRPM = 3000;
    final double desiredRPMA = -1 * desiredRPM;

    double CurrentRPMA;// = encoderSideA.getPosition();
    double CurrentRPMB;// = encoderSideB.getPosition();

    boolean isShooting = false;

    public Shooter(int shootportA, int shootSideB, int flyport, int pistonA, int pistonB) {
        // = new CANSparkMax(shootportA, MotorType.kBrushless);
        //sideShootB = new CANSparkMax(shootSideB, MotorType.kBrushless);
        flyWheel = new TalonSRX(flyport);
        SmartDashboard.putString("Intake","Init");

        //transferPiston = new DoubleSolenoid(pistonA, pistonB);
        

        //encoderSideA = sideShootA.getEncoder();
        //encoderSideB = sideShootB.getEncoder();

        //pidA = sideShootA.getPIDController();
        //pidB = sideShootB.getPIDController();

        //pidA.setFeedbackDevice(encoderSideA);
        //pidB.setFeedbackDevice(encoderSideB);

        // pid constants
        kP = .001;
        kI = 0;
        kD = 0;
        kIz = 0;
        kFF = 0;
        kMax = 1;
        kMin = -1;
        //pidA.setP(shooterP);
        //pidA.setI(shooterI);
        //pidA.setD(shooterD);
        // pidA.setIZone(kIz);
        //pidA.setFF(shooterF);
        //pidA.setOutputRange(kMin, kMax);
        //pidB.setP(shooterP);
        //pidB.setI(shooterI);
       // pidB.setD(shooterD);
        //pidB.setIZone(kIz);
        //pidB.setFF(shooterF);
        //pidB.setOutputRange(kMin, kMax);
    }

    public void performMainProcessing() {
        if (Robot.xboxJoystick.getRawAxis(Xbox.RT) > 0.2) {
            shoot();
            if (isShooting == false) {
                delayer.delay(0.75);
            }
            Robot.mIndexer.spin();
            isShooting = true;
        }
        else if (Robot.xboxJoystick.getRawAxis(Xbox.LT) > 0.2)
        {
            blue();
            if (isShooting == false) {
                delayer.delay(0.75);
            }
            Robot.mIndexer.spin();
            isShooting = true;
        }

         else if (Robot.xboxJoystick.getRawButton(Xbox.LB)) {
            outtake();
        } else {
            stop();
        }
        tunePID();
        updateTelemetry();
    }

    public void shoot() {
        if (usePID) {

            pidA.setReference(desiredRPMA, ControlType.kVelocity);
            pidB.setReference(desiredRPM, ControlType.kVelocity);

        } else {
            sideShootA.set(0.53);
            sideShootB.set(-0.53);
            //flyWheel.set(ControlMode.PercentOutput, -1.0);
        }
    }

    public void blue()
    {
        sideShootA.set(0.46);
            sideShootB.set(-0.46);
           // flyWheel.set(ControlMode.PercentOutput, -1.0);
    }

    public void outtake() {
       // sideShootA.set(-0.5);
       // sideShootB.set(0.5);
        flyWheel.set(ControlMode.PercentOutput, 0.75);
        //inBetween.set(ControlMode.PercentOutput, 0.6);
        // transferPiston.set(Value.kReverse);
        // inBetween.set(ControlMode.PercentOutput, -0.7);
    }

    public void stop() {
        // sideShootA.set(0);
        // sideShootB.set(0);
        //pidA.setReference(0, ControlType.kDutyCycle);
        //pidB.setReference(0, ControlType.kDutyCycle);
        //flyWheel.set(ControlMode.PercentOutput, 0);
        //inBetween.set(ControlMode.PercentOutput, 0);
        //isShooting = false;
    }

    /*public void ballTransfer() {
        flyWheel.set(ControlMode.PercentOutput, -0.65);
        transferPiston.set(Value.kReverse);
        
    }*/

    public void getEncoder() {
        CurrentRPMA = encoderSideA.getPosition();
    }

    public void setRPM(double RPM) {
        desiredRPM = RPM;
    }

    public double getRPM() {
        return desiredRPM;
    }

    private void tunePID() {
        if (Robot.leftJoystick.getRawButton(7) == true) {
            shooterP = shooterP + 10e-5;
        }
        if (Robot.leftJoystick.getRawButton(8) == true) {
            shooterP = shooterP - 10e-5;
        }
        if (Robot.leftJoystick.getRawButton(9) == true) {
            shooterI = shooterI + 1e-3;
        }
        if (Robot.leftJoystick.getRawButton(10) == true) {
            shooterI = shooterI - 1e-3;
        }
        if (Robot.leftJoystick.getRawButton(11) == true) {
            shooterD = shooterD + 1e-3;
        }
        if (Robot.leftJoystick.getRawButton(12) == true) {
            shooterD = shooterD - 1e-3;
        }
        // SmartDashboard.putNumber("Elevator Lower Kstatic",STATIC_GAIN_LOW);
        SmartDashboard.putNumber("Elevator Lower Kp*1e-3", shooterP);
        SmartDashboard.putNumber("Elevator Lower Ki*1e-3", shooterI);
        SmartDashboard.putNumber("Elevator Lower Kd*1e-3", shooterD);
    }

    public void updateTelemetry() {
        SmartDashboard.putNumber("side A encoder", encoderSideA.getVelocity());
        SmartDashboard.putNumber("side B encoder", encoderSideB.getVelocity());
        SmartDashboard.putNumber("Side A shooter", flyWheel.getSensorCollection().getQuadraturePosition());

        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Zone", kIz);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Max Output", kMax);
        SmartDashboard.putNumber("Min Output", kMin);
        SmartDashboard.putNumber("Set Rotations", desiredRPM);
    }

}
