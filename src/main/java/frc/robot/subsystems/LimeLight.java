// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Solenoid;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.TreeMap;

public class LimeLight extends SubsystemBase {
  Solenoid light;
  /** Creates a new LimeLight. */
  public LimeLight() {
    setLightState(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setStreamMode(int Stream){
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(Stream);
  }

  public void setPipeline(int Pipeline){
	  NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(Pipeline);
  }
  
  public void setCammode(int Cammode){
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(Cammode);
  }

  /**
   * Set the LED light on or off, this 
   * includes the LED from the PCM
   * @param LightState 1 for off, 3 for On
   */
  public void setLightState(int LightState){
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(LightState);  //controls if limelight is on or not // 3 is on, 1 is off
    light.set(3 == LightState);
  }

  /**
   * True if the limelight sees a target
   * @return
   */
  public boolean haveTarget(){
    return (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1); //returns true if it detects a target
  }

  /**
   * horizontal angle to the target.
   * (+ is left of target, - is right of target)
   * @return
   */
  public double angleToTarget(){
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0); //returns the angle offset (+ is left of target, - is right of target)
  }

  public double verticalAngleToTarget(){
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0); //returns the vertical angle offset
  }

  public double getShooterHighSpeed(){
    return 0.0;//TODO:link to look up table
  }

  public double getShooterLowSpeed(){
    return 0.0;//TODO:link to look up table
  }

  public double getHoodHighAngle(){
    return 0.0;//TODO:link to look up table
  }

  public double getHoodLowAngle(){
    return 0.0;//TODO:link to look up table
  }
}
