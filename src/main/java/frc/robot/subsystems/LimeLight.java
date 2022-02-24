// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLight extends SubsystemBase {
  // Solenoid light;
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
    // light.set(3 == LightState);//this is for a secondary light powered by PCM
  }

  /**
   * True if the limelight sees a target
   * @return
   */
  public boolean hasTarget(){
    return (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1); //returns true if it detects a target
  }

  /**
   * horizontal angle to the target.
   * (+ is left of target, - is right of target)
   * @return
   */
  public double angleToTarget(){
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0); //returns the angle offset (+ is left of target, - is right of target)<- think that is backwards
  }

  public double angleToTarget(double offsetDistance){
    //TODO: using the distance to target compute an angle that will result in the desired offset
    //if offset is 0 then return angleToTarget
    //start by pulling the distance using distance to target and adding Target radius(one side of triangle)
    //take the arctangent(use atan not atan2) of the offsetDistance divided by the prior distance
    //convert the resulting angle from Radians to degrees.
    //make the output the angleToTarget minus (the converted angle times the sign of the angle)
    return 0.0;
  }

  public double verticalAngleToTarget(){
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0); //returns the vertical angle offset
  }

  public double distanceToTarget(){
    return Constants.LL_TARGET_TO_ROBOT_HEIGHT / Math.tan(Math.toRadians(verticalAngleToTarget() + Constants.LL_MOUNT_ANGLE));
  }

  public double distanceFrontToTarget(){
    return distanceToTarget() + Constants.LL_ROBOT_DISTANCE_TO_FRONT;
  }

  public double distanceCenterToCenter(){
    return distanceToTarget() + Constants.LL_DISTANCE_TO_CENTER + Constants.LL_TARGET_RADIUS;
  }

  public double getShooterHighSpeed(){
    return Constants.SHOOTER_HIGH_SPEEDS_TABLE.lookup(distanceCenterToCenter());
  }

  public double getShooterLowSpeed(){
    return Constants.SHOOTER_LOW_SPEEDS_TABLE.lookup(distanceCenterToCenter());
  }

  public double getHoodHighAngle(){
    return Constants.HOOD_HIGH_POSITION_TABLE.lookup(distanceCenterToCenter());
  }

  public double getHoodLowAngle(){
    return Constants.HOOD_LOW_POSITION_TABLE.lookup(distanceCenterToCenter());
  }
}
