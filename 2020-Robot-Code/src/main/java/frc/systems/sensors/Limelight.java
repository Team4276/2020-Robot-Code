/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.systems.sensors;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class Limelight {

  public boolean LimelightHasValidTarget = false;
  public double rightSteering = 0.0;
  public double leftSteering = 0.0;

  double distance = 0.0;

  // steering commands
  double Kp = 0.03;
  double kI = 0.03;

  double errorI = 0;
  double timeStep = 0;
  final double minSteer = 0.00005;

  double steerCmd;
  

  // driving commands
  double DRIVE_K = 0.26;
  final double DESIRED_TARGET_AREA = 13.0;
  final double MAX_DRIVE = 0.7;

  public double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
  public double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  public double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  public double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
  public double ts = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(-90);

  public Limelight() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
  }

  public void RotateTracking() {
    setLedMode(3);

    setCamMode(0);
    if(Robot.mDrivetrain.getAutoPress()){
      kI = 0;
    }
    
    if (tv < 1.0) {
      LimelightHasValidTarget = false;
      rightSteering = 0.0;
      leftSteering = 0.0;
      return;
    }
    else{
    LimelightHasValidTarget = true;

    steerCmd = 0;
    errorI += tx * timeStep;

    if (tx > 1.0) {
      steerCmd = Kp * tx - minSteer + kI * errorI;
    } else if (tx < 1.0) {
      steerCmd = Kp * tx + minSteer + kI * errorI;
    }
    if(tx<2.0 && tx>-2.0){
      steerCmd = 0;
    }
    

    // steerCmd = Kp * tx;

    rightSteering = steerCmd;
    leftSteering = -steerCmd;

    // setCamMode(1);
    // setLedMode(1);
  }

  
  }
  public void DriveTracking() {

    double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

    rightSteering += drive_cmd;
    leftSteering += drive_cmd;

  }

  public void setLedMode(int mode) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(mode);
  }

  public void setCamMode(int mode) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(mode);
  }

  public void setPipeline(int mode) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(mode);
  }

  public double getDistance() {

    distance = (DESIRED_TARGET_AREA - ta);
    return distance;
  }

  public double getSteer(double steer) {
    return steer;
  }

  public double getTx() {
    return tx;
  }

  private void tunePID() {
    if (Robot.leftJoystick.getRawButton(7) == true) {
      Kp = Kp + 10e-3;
    }
    if (Robot.leftJoystick.getRawButton(8) == true) {
      Kp = Kp - 10e-3;
    }
    if (Robot.leftJoystick.getRawButton(9) == true) {
      DRIVE_K = DRIVE_K + 10e-3;
    }
    if (Robot.leftJoystick.getRawButton(10) == true) {
      DRIVE_K = DRIVE_K - 10e-3;
    }
  }

  public void updateTelementry() {
    tunePID();
    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    ts = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(0);

    SmartDashboard.putBoolean("Has Target", LimelightHasValidTarget);
    SmartDashboard.putNumber("KP Steering", Kp);
    SmartDashboard.putNumber("ts", ts);
    SmartDashboard.putNumber("tv", tv);
    SmartDashboard.putNumber("tx", tx);
  }

}