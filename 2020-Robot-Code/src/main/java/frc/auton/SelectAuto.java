/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.auton;

import frc.robot.Robot;
//import frc.systems.Drivetrain;
//import frc.systems.Shooter;
//import frc.systems.Intake;
//import frc.systems.sensors.Limelight;
import frc.utilities.SoftwareTimer;

import edu.wpi.first.wpilibj.Timer;

public class SelectAuto {
    SoftwareTimer delay;
    Timer d = new Timer();

    boolean firstTime = true;

    // State Machine Step Definitions
    private final int BACK_UP = 0;
    private final int TARGET_HIGH_GOAL = 1;
    private final int START_SHOOTER = 2;
    private final int FEED_BALLS = 3;
    private final int INTAKE = 4;
    private final int END_AUTO = 5;

    // State Machine Common Control Parameters
    private int currentStep = 0;
    private boolean performInitProcessing = true;
    private boolean proceedToNextState = false;
    SoftwareTimer stateDeadmanTimer = new SoftwareTimer();

    public enum AutoMode {
        DriveOffLine, StraightShoot, MiddleShoot, SideShoot, DoNothing;
    }

    public AutoMode currentSelection = AutoMode.StraightShoot;

    public SelectAuto() {
        delay = new SoftwareTimer();
    }

    public void performAuto() {
        switch (currentStep) {
        case BACK_UP:
            // Step Entry
            if (performInitProcessing) {
                stateDeadmanTimer.setTimer(2);
                performInitProcessing = false;
                proceedToNextState = false;
            }
            // Step Processing
            Robot.mDrivetrain.assignMotorPower(-0.35, 0.35);
            // Step Exit Criteria Check
            if (stateDeadmanTimer.isExpired()) {
                proceedToNextState = true;
            }
            // Step Exit
            if (proceedToNextState) {
                currentStep++;
                performInitProcessing = true;
                proceedToNextState = false;
                Robot.mDrivetrain.assignMotorPower(0, 0);
            }
            break;
        case TARGET_HIGH_GOAL:
            // Step Entry
            if (performInitProcessing) {
                stateDeadmanTimer.setTimer(2);
                performInitProcessing = false;
                proceedToNextState = false;
            }
            // Step Processing
            Robot.mDrivetrain.LimelightRotate();
            // Step Exit Criteria Check
            if (stateDeadmanTimer.isExpired()) {
                proceedToNextState = true;
            }
            // Step Exit
            if (proceedToNextState) {
                currentStep++;
                performInitProcessing = true;
                proceedToNextState = false;
            }
            break;
        case START_SHOOTER:
            // Step Entry
            if (performInitProcessing) {
                stateDeadmanTimer.setTimer(1);
                performInitProcessing = false;
                proceedToNextState = false;
            }
            // Step Processing
            //Robot.mShooter.shoot();
            // Step Exit Criteria Check
            if (stateDeadmanTimer.isExpired()) {
                proceedToNextState = true;
            }
            // Step Exit
            if (proceedToNextState) {
                currentStep++;
                performInitProcessing = true;
                proceedToNextState = false;
            }
            break;
        case FEED_BALLS:
            // Step Entry
            if (performInitProcessing) {
                stateDeadmanTimer.setTimer(5);
                performInitProcessing = false;
                proceedToNextState = false;
            }
            // Step Processing
            //Robot.mShooter.shoot();
            //Robot.mShooter.ballTransfer();
            // Step Exit Criteria Check
            if (stateDeadmanTimer.isExpired()) {
                proceedToNextState = true;
            }
            // Step Exit
            if (proceedToNextState) {
                currentStep++;
                performInitProcessing = true;
                proceedToNextState = false;
            }
            break;
        case INTAKE:
            // Step Entry
            if (performInitProcessing) {
                stateDeadmanTimer.setTimer(10);
                performInitProcessing = false;
                proceedToNextState = false;
            }
            // Step Processing
            //Robot.mShooter.shoot();
            //Robot.mShooter.ballTransfer();
            Robot.mIntake.intake();
            // Step Exit Criteria Check
            if (stateDeadmanTimer.isExpired()) {
                proceedToNextState = true;
            }
            // Step Exit
            if (proceedToNextState) {
                currentStep++;
                performInitProcessing = true;
                proceedToNextState = false;
            }
            break;
        case END_AUTO:
            break;
        }

    }

    /*
     * public void selectRoutine() { // firstTime = true; switch (currentSelection)
     * { case DoNothing: Drive.assignMotorPower(0, 0); break; case DriveOffLine:
     * 
     * System.out.println(d.get()); if (firstTime) { firstTime = false;
     * 
     * Drive.assignMotorPower(0.2, -0.2); d.start(); } else { while (d.get() > 0.5)
     * { if (d.get() < 0.4) Drive.assignMotorPower(0.2, -0.2); else {
     * Drive.assignMotorPower(0, 0); } } } break; // basic command to find goal and
     * shoot if lined up with it case StraightShoot:
     * 
     * delay.setTimer(5); Drive.LimelightRotate(); if (lime.LimelightHasValidTarget)
     * { shot.shoot(); } if (delay.isExpired()) { shot.stop();
     * Drive.assignMotorPower(0, 0); } delay.setTimer(0.5); if (delay.isExpired()) {
     * Drive.assignMotorPower(-0.5, 0.5); }
     * 
     * break;
     * 
     * // if other robot wants to start in front of goal case MiddleShoot:
     * 
     * break;
     * 
     * // if we have to start on the far side case SideShoot:
     * 
     * break; } }
     */
    public void setMode(final String select) {
        if (select.equals("StraightShoot")) {
            currentSelection = AutoMode.StraightShoot;
        } else if (select.equals("MiddleShoot")) {
            currentSelection = AutoMode.MiddleShoot;
        } else if (select.equals("SideShoot")) {
            currentSelection = AutoMode.SideShoot;
        } else if (select.equals("DriveOffLine")) {
            currentSelection = AutoMode.DriveOffLine;
        } else {
            currentSelection = AutoMode.DoNothing;
        }
    }

}
