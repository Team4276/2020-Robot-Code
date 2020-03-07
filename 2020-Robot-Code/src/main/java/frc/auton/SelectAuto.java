/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.auton;

import frc.robot.Robot;
import frc.systems.Drivetrain;
import frc.systems.Shooter;
import frc.systems.Intake;
import frc.systems.sensors.Limelight;
import frc.utilities.SoftwareTimer;
import edu.wpi.first.wpilibj.Timer;

public class SelectAuto {
    SoftwareTimer delay;
    Timer d = new Timer();
    Drivetrain Drive = Robot.mDrivetrain;
    Shooter shot = Robot.mShooter;
    Intake inta = Robot.mIntake;
    Limelight lime = Robot.mLimelight;
    boolean firstTime = true;

    public enum AutoMode {
        DriveOffLine, StraightShoot, MiddleShoot, SideShoot, DoNothing;
    }

    public AutoMode currentSelection = AutoMode.StraightShoot;

    public SelectAuto() {
        delay = new SoftwareTimer();

    }

    public void selectRoutine() {
        //firstTime = true;
        switch (currentSelection) {
        case DoNothing:
            Drive.assignMotorPower(0, 0);
            break;
        case DriveOffLine:
        
        System.out.println(d.get());
        if(firstTime) {
            firstTime = false;
            
            
            Drive.assignMotorPower(0.2, -0.2);
            d.start();
        }
            else {
                while (d.get()>0.5) {
                    if(d.get()<0.4)
                    Drive.assignMotorPower(0.2, -0.2);
                    else{
                        Drive.assignMotorPower(0, 0);
                    }
                }
            } 
            break;
        // basic command to find goal and shoot if lined up with it
        case StraightShoot:

            delay.setTimer(5);
            Drive.LimelightRotate();
            if (lime.LimelightHasValidTarget) {
                shot.shoot();
            }
            if (delay.isExpired()) {
                shot.stop();
                Drive.assignMotorPower(0, 0);
            }
            delay.setTimer(0.5);
            if (delay.isExpired()) {
                Drive.assignMotorPower(-0.5, 0.5);
            }

            break;

        // if other robot wants to start in front of goal
        case MiddleShoot:

            break;

        // if we have to start on the far side
        case SideShoot:

            break;
        }
    }

    public void setMode(final String select) {
        if (select.equals("StraightShoot")) {
            currentSelection = AutoMode.StraightShoot;
        } else if (select.equals("MiddleShoot")) {
            currentSelection = AutoMode.MiddleShoot;
        } else if (select.equals("SideShoot")) {
            currentSelection = AutoMode.SideShoot;
        } else if (select.equals("DriveOffLine")) {
            currentSelection = AutoMode.StraightShoot;
        } else {
            currentSelection = AutoMode.DoNothing;
        }
    }

}
