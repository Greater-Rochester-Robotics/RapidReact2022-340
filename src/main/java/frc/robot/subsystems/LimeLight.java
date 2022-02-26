// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

import java.util.Collection;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class LimeLight extends SubsystemBase {
  static Map<String,Boolean> subsystemsUsingLight = new HashMap<String,Boolean>();
  Solenoid light;

  /** Creates a new LimeLight. */
  public LimeLight() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    light = new Solenoid(PneumaticsModuleType.REVPH, Constants.LL_LIGHT);
    light.set(false);
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
  public void setLightState(boolean lightOn, SubsystemBase subsystem){
    subsystemsUsingLight.put(subsystem.getName(), lightOn);
    Collection<Boolean> values = subsystemsUsingLight.values();
    boolean sumOfBooleans = false;
    for (Boolean v : values) {
      sumOfBooleans |=  v;
    }
    System.out.println("sumBool"+sumOfBooleans);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(sumOfBooleans? 3 : 1);  //controls if limelight is on or not // 3 is on, 1 is off
    light.set(sumOfBooleans);//this is for a secondary light powered by PCM
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

  /**
   * Uses the distance to target to compute an angle
   * that will result in the desired offset
   * @param offsetDistance distance to offset by
   * @return
   */
  public double angleToTarget(double offsetDistance){
    double angleToTarget = angleToTarget();
    if(offsetDistance != 0.0) {
      double distanceToTarget = distanceToTarget();
      distanceToTarget += Constants.LL_TARGET_RADIUS;
      double angle = Math.toDegrees(Math.atan(offsetDistance / distanceToTarget));
      return angleToTarget - (angle * Math.signum(angleToTarget));
    } else {
      return angleToTarget;
    }
  }

  public double verticalAngleToTarget(){
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0); //returns the vertical angle offset
  }

  public double distanceFrontToFender(){
    return distanceToTarget() - Constants.LL_DISTANCE_TO_FRONT - Constants.LL_TARGET_TO_FENDER;
  }
  
  public double distanceToTarget(){
    return Constants.LL_TARGET_TO_LL_HEIGHT / Math.tan(Math.toRadians(verticalAngleToTarget() + Constants.LL_MOUNT_ANGLE));
  }

  public double distanceFrontToTarget(){
    return distanceToTarget() + Constants.LL_DISTANCE_TO_FRONT;
  }

  public double distanceCenterToCenter(){
    return distanceToTarget() + Constants.LL_DISTANCE_TO_ROBOT_CENTER + Constants.LL_TARGET_RADIUS;
  }

  public double getShooterHighSpeed(){
    return Constants.SHOOTER_HIGH_SPEEDS_TABLE.lookup(distanceFrontToFender());
  }

  public double getShooterLowSpeed(){
    return Constants.SHOOTER_LOW_SPEEDS_TABLE.lookup(distanceFrontToFender());
  }

  public double getHoodHighAngle(){
    return Constants.HOOD_HIGH_POSITION_TABLE.lookup(distanceFrontToFender());
  }

  public double getHoodLowAngle(){
    return Constants.HOOD_LOW_POSITION_TABLE.lookup(distanceFrontToFender());
  }
}
