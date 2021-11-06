/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.systems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import frc.robot.Robot;
import frc.utilities.Xbox;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.utilities.SoftwareTimer;
import edu.wpi.first.wpilibj.Timer;

public class Intake {
    double intakeUpSpeed = -0.90;
    double intakeDownSpeed = -1.0;

    double outtakeUpSpeed = 0.90;

    //VictorSPX lowerMotor;
    VictorSPX armMotor;
    //VictorSPX codyMotor;
    TalonSRX pivoteMotor;
    int n = 10;//initial timer for pulse method
    //SoftwareTimer extendTimer;
    boolean isExtend = false;
    Timer delayerArm;

    public Intake( int upperport ){//int pivoteport, int inBetweenPort, int pistonA, int pistonB) {
        armMotor = new VictorSPX(upperport);
       // pivoteMotor = new TalonSRX(pivoteport);
    }

    public void performMainProcessing() {
        if (Robot.xboxJoystick.getRawAxis(Xbox.RAxisY)>0.2) {
            outtake();
            SmartDashboard.putString("Intake","Outtakking");
        }
        else if(Robot.xboxJoystick.getRawAxis(Xbox.RAxisY)<-0.2){
            intake();
            SmartDashboard.putString("Intake","Intakking");
        }
         else {
            stop();
            SmartDashboard.putString("Intake","Stopped");
        }
        /*if ( Robot.xboxJoystick.getRawAxis(Xbox.LAxisY) > 0.2 )
        {
            if( !isExtend )
            {
                delayerArm.delay(1.0);
            }
            if( Robot.xboxJoystick.getRawAxis(Xbox.LAxisY) > 0.2 )
            {
             pulse();
             isExtend = Robot.xboxJoystick.getRawAxis(Xbox.LAxisY) > 0.2;
            }
        }
        else if( !isExtend)
        {
            stop();
            isExtend = Robot.xboxJoystick.getRawAxis(Xbox.LAxisY) > 0.2;
        }*/
/*
        extendTimer.setTimer(n);
        if (n == 0)
        {
            if ( isExtend )
            {
                pulse();
                extendTimer.setTimer(1);
                n = 1;
                isExtend = Robot.xboxJoystick.getRawAxis(Xbox.LAxisY) > 0.2;
            }
            else
            {
                Robot.mIndexer.stop();
                isExtend = Robot.xboxJoystick.getRawAxis(Xbox.LAxisY) > 0.2;
            }
        }
        else if( isExtend )
        {
            isExtend = Robot.xboxJoystick.getRawAxis(Xbox.LAxisY) > 0.2;
            extendTimer.setTimer(0);
        }
        else
        {
            Robot.mIndexer.stop();
            extendTimer.setTimer(0);
            isExtend = Robot.xboxJoystick.getRawAxis(Xbox.LAxisY) > 0.2;
        }
        */
    }

    public void intake() {
    
        armMotor.set(ControlMode.PercentOutput, intakeUpSpeed);
        //Robot.mIndexer.spin();

    }

    public void outtake() {
        
        armMotor.set(ControlMode.PercentOutput, outtakeUpSpeed);
        //Robot.mIndexer.back();
        Robot.mShooter.outtake();
    }

    public void stop() {
        //lowerMotor.set(ControlMode.PercentOutput, 0);
        armMotor.set(ControlMode.PercentOutput, 0);
        //Robot.mIndexer.stop();
        //shoot.inBetween.set(ControlMode.PercentOutput, 0);
    }

    

    public void pulse()
    {
        //armMotor.set(ControlMode.PercentOutput, intakeUpSpeed);
        Robot.mIndexer.spin();
    }

    /*public void pivoteup() {
        pivoteMotor.set(ControlMode.PercentOutput, 1.0);
    }

    public void pivotedown() {
        pivoteMotor.set(ControlMode.PercentOutput, -1.0);
    }

    public void updateTelemetry() {
        SmartDashboard.putNumber("pivote angle", pivoteMotor.getSensorCollection().getQuadraturePosition());
    }*/
}
