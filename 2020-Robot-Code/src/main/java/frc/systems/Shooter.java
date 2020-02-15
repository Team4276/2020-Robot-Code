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
import edu.wpi.first.wpilibj.PIDBase;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter {
    CANSparkMax sideShootA;
    CANSparkMax sideShootB;
    TalonSRX flyWheel;
    CANEncoder encoderSideA;
    CANEncoder encoderSideB;
    CANPIDController pidA, pidB;
    double kP, kI, kD, kIz, kFF, kMax, kMin;
    final double rotations = 1000; 
    DoubleSolenoid piston;
    VictorSPX inBetween;

    double CurrentRPMA = encoderSideA.getPosition();
    double CurrentRPMB = encoderSideB.getPosition();
    

    public Shooter(int shootportA, int shootSideB, int flyport, int pistonA, int pistonB, int inBetweenPort) {
        sideShootA = new CANSparkMax(shootportA, MotorType.kBrushless);
        sideShootB = new CANSparkMax(shootSideB, MotorType.kBrushless);
        flyWheel = new TalonSRX(flyport);
        piston = new DoubleSolenoid(pistonA, pistonB);
        inBetween = new VictorSPX(inBetweenPort);
        encoderSideA = sideShootA.getEncoder();
        encoderSideB = sideShootB.getEncoder();
        pidA = sideShootA.getPIDController();
        pidB = sideShootB.getPIDController();
        kP = .1;
        kI = 1e-4;
        kD = 1;
        kIz = 0;
        kFF = 0;
        kMax = 1;
        kMin = -1;
        pidA.setP(kP);
        pidA.setI(kI);
        pidA.setD(kD);
        pidA.setIZone(kIz);
        pidA.setFF(kFF);
        pidA.setOutputRange(kMin,kMax);
        pidB.setP(kP);
        pidB.setI(kI);
        pidB.setD(kD);
        pidB.setIZone(kIz);
        pidB.setFF(kFF);
        pidB.setOutputRange(kMin,kMax);
        
    }


    public void performMainProcessing() {
        if (Robot.xboxJoystick.getRawAxis(Xbox.RT) > 0.2) {
            shoot();
            
            if(CurrentRPMA > rotations - 200 && CurrentRPMA < rotations + 200){
                ballTransfer();
            }

        } else {
            stop();
        }
    }

    public void shoot() {
        pidA.setReference(rotations, ControlType.kPosition);
        pidB.setReference(rotations, ControlType.kPosition);
        flyWheel.set(ControlMode.PercentOutput, 1.0);
    }

    public void outtake() {
        sideShootA.set(1.0);
        sideShootB.set(-1.0);
        flyWheel.set(ControlMode.PercentOutput, -1.0);
    }

    public void stop() {
        sideShootA.set(0);
        sideShootB.set(0);
        flyWheel.set(ControlMode.PercentOutput, 0);
    }

    public void ballTransfer()
    {
        piston.set(Value.kReverse);
        inBetween.set(ControlMode.PercentOutput, 0.7);

    }

    public void updateTelemetry() {
        SmartDashboard.putNumber("side A encoder", encoderSideA.getPosition());
        SmartDashboard.putNumber("side B encoder", encoderSideB.getPosition());
        SmartDashboard.putNumber("Side A shooter", flyWheel.getSensorCollection().getQuadraturePosition());

        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Zone", kIz);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Max Output", kMax);
        SmartDashboard.putNumber("Min Output", kMin);
        SmartDashboard.putNumber("Set Rotations", 0);
    }

}
