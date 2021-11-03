/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.systems;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.utilities.Xbox;

public class Climber {
    DoubleSolenoid dubSol;

    public Climber(int dubSolA, int dubSolB){
        dubSol = new DoubleSolenoid(dubSolA, dubSolB);
    }

    public void performMainProcessing() {
        if (Robot.xboxJoystick.getRawAxis(Xbox.LAxisY) > 0.2){
            dubSol.set(Value.kForward);
        }
        else {
            dubSol.set(Value.kReverse);
        }
    }
}
