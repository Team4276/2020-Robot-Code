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
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class Intake {
    double intakeUpSpeed = 1.0;
    double intakeDownSpeed = -1.0;
    VictorSPX lowerMotor;
    VictorSPX upperMotor;
    TalonSRX pivoteMotor;
    public Intake (int lowerport, int upperport, int pivoteport) {
        lowerMotor = new VictorSPX(lowerport);
        upperMotor = new VictorSPX(upperport);
        pivoteMotor = new TalonSRX(pivoteport);
    }
    public void performMainProcessing(){
    if (Robot.xboxJoystick.getRawButton(Xbox.RB)){
        intake();
    }
    else if (Robot.xboxJoystick.getRawButton(Xbox.LB)){
        outtake();
    }
    else{
        stop();
    }
    if (Robot.xboxJoystick.getRawAxis(Xbox.RAxisY) > 0.1){
        pivoteup();
    }
    else if (Robot.xboxJoystick.getRawAxis(Xbox.RAxisY) < -0.1){
        pivotedown();
    }
    else{
        pivoteMotor.set(ControlMode.PercentOutput, 0);
    }

    }
    public void intake(){
        lowerMotor.set(ControlMode.PercentOutput, intakeDownSpeed);
        upperMotor.set(ControlMode.PercentOutput, intakeUpSpeed);
    }
    public void outtake(){
        lowerMotor.set(ControlMode.PercentOutput, intakeUpSpeed);
        upperMotor.set(ControlMode.PercentOutput, intakeDownSpeed);
    }
    public void stop(){
        lowerMotor.set(ControlMode.PercentOutput, 0);
        upperMotor.set(ControlMode.PercentOutput, 0);
    }
    public void pivoteup(){
        pivoteMotor.set(ControlMode.PercentOutput, 1.0);
    }
    public void pivotedown(){
        pivoteMotor.set(ControlMode.PercentOutput, -1.0);
    }
    public void updateTelemetry(){
        SmartDashboard.putNumber("pivote angle", pivoteMotor.getSensorCollection().getQuadraturePosition());
    }
}
