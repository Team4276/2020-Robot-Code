/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.systems.sensors;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.smartdashboard.*;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;



public class ColorSen {
  String output;
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 m_colorSensor;
    Color detectedColor;

    public ColorSen(Port ixPort) {
       // m_colorSensor = new ColorSensorV3(i2cPort);
        m_colorSensor = new ColorSensorV3(ixPort);
    }
    public void UpdateTelemetry(){
        detectedColor = m_colorSensor.getColor();
        double Red = detectedColor.red;
        double Blue = detectedColor.blue;
        double Green = detectedColor.green;
    double IR = m_colorSensor.getIR();
    if (Red > 0.29 && Green > 0.52 && Blue < 0.2){
      output = "Yellow";
    }
    else if (Red > 0.49 && Green > 0.3 && Blue < 0.2){
      output = "Red";
    }
    else if (Red < 0.23 && Green > 0.55 && Blue > 0.23){
      output = "Green";
    }
    else if (Red < 0.2 && Green > 0.42 && Blue > 0.4){
      output =  "Blue";
    }
    else{
      output = "Not Those Colors";
    }
    

    SmartDashboard.putString("Color?!", output);
        SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("IR", IR);
    int proximity = m_colorSensor.getProximity();
    SmartDashboard.putNumber("Proximity", proximity);
  

    }
}
