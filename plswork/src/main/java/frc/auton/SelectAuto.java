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

public class SelectAuto {
    SoftwareTimer delay;
    Drivetrain Drive = Robot.mDrivetrain;
    Shooter shot = Robot.mShooter;
    Intake inta = Robot.mIntake;
    Limelight lime = Robot.mLimelight;

    public enum AutoMode {
        DriveOffLine, StraightShoot, MiddleShoot, SideShoot, DoNothing;
    }

    public AutoMode currentSelection = AutoMode.StraightShoot;

    public SelectAuto() {
        delay = new SoftwareTimer();

    }

    public void selectRoutine() {
        switch (currentSelection) {
        case DoNothing:
            Drive.assignMotorPower(0, 0);
        break;
        case DriveOffLine:
            Drive.assignMotorPower(0.5, -0.5);
            delay.setTimer(3);
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

    public void setMode(String select) {
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
