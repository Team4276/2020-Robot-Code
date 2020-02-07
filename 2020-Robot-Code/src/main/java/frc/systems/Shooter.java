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
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Robot;
import frc.utilities.Xbox;

import com.revrobotics.CANEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;;

public class Shooter {
    CANSparkMax sideShootA;
    CANSparkMax sideShootB;
    TalonSRX flyWheel;
    CANEncoder encoderSideA;
    CANEncoder encoderSideB;

    public Shooter(int shootportA, int shootSideB, int flyport) {
        sideShootA = new CANSparkMax(shootportA, MotorType.kBrushless);
        sideShootB = new CANSparkMax(shootSideB, MotorType.kBrushless);
        flyWheel = new TalonSRX(flyport);
        encoderSideA = sideShootA.getEncoder();
        encoderSideB = sideShootB.getEncoder();
    }

    public void performMainProcessing() {
        if (Robot.xboxJoystick.getRawAxis(Xbox.RT) > 0.2) {
            shoot();
        } else {
            stop();
        }
    }

    public void shoot() {
        sideShootA.set(-1.0);
        sideShootB.set(1.0);
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

    public void updateTelemetry() {
        SmartDashboard.putNumber("side A encoder", encoderSideA.getPosition());
        SmartDashboard.putNumber("side B encoder", encoderSideB.getPosition());
        SmartDashboard.putNumber("Side A shooter", flyWheel.getSensorCollection().getQuadraturePosition());
    }

}
