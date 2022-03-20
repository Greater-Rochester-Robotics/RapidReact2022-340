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
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimeLight extends SubsystemBase {
  Solenoid light;
  PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

  static Map<String,Boolean> subsystemsUsingLight = new HashMap<String,Boolean>();
  int count = 0;

  /** Creates a new LimeLight. */
  public LimeLight() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    light = new Solenoid(PneumaticsModuleType.REVPH, Constants.LL_LIGHT);
    light.set(false);
    subsystemsUsingLight.put(this.getName(), false);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(count > 50){
      count = 0;
    }

    if(count == 0) {
      Collection<Boolean> values = subsystemsUsingLight.values();
      boolean sumOfBooleans = false;
      for (Boolean v : values) {
        sumOfBooleans |=  v;

      }
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(sumOfBooleans? 3 : 1);  //controls if limelight is on or not // 3 is on, 1 is off
      light.set(sumOfBooleans);
      
    }
    count++;
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
   * 
   * @param isOn
   */
  public void setLimeLightPower(boolean isOn){
    pdh.setSwitchableChannel(isOn);

  }

  /**
   * Set the LED light on or off, this 
   * includes the LED from the PH
   * This method lets all subsystems say whether they use the light, 
   * or not, when all subsystems chose not to use the light it turns off.
   * 
   * @param boolean true for on, false for off
   * @param SubsystemBase name of the subsystem(command) using limelight LED
   */
  public void setLightState(boolean lightOn, SubsystemBase subsystem){
    subsystemsUsingLight.put(subsystem.getName(), lightOn);
    Collection<Boolean> values = subsystemsUsingLight.values();
    boolean sumOfBooleans = false;
    System.out.println("Starting: ");
    for (Boolean v : values) {
      sumOfBooleans |=  v;
      System.out.println("v: " + v);
      System.out.println("End");
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

  /**
   * the vertical angle of the target, in inches, as measured 
   * from the center of the camera. used to findt he distance 
   * to the target.
   * 
   * @return angle in degrees
   */
  public double verticalAngleToTarget(){
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0); //returns the vertical angle offset
  }

  /**
   * distance from the front of the robot to the fender, if the front of the 
   * robot is parallel with the fender. This is used as all shooter and hood 
   * data use this as the standard of distance.
   * 
   * @return distance in inches
   */
  public double distanceFrontToFender(){
    return distanceToTarget() - Constants.LL_DISTANCE_TO_FRONT - Constants.LL_TARGET_TO_FENDER;
  }
  
  /**
   * distance from the limelight to the target
   * @return distance in inches
   */
  public double distanceToTarget(){

    SmartDashboard.putNumber("DistanceToTarget", Constants.LL_TARGET_TO_LL_HEIGHT / Math.tan(Math.toRadians(verticalAngleToTarget() + Constants.LL_MOUNT_ANGLE))); 
    return Constants.LL_TARGET_TO_LL_HEIGHT / Math.tan(Math.toRadians(verticalAngleToTarget() + Constants.LL_MOUNT_ANGLE));
  }

  /**
   * distance from the front of the robot to the target
   * @return distance in inches
   */
  public double distanceFrontToTarget(){
    return distanceToTarget() + Constants.LL_DISTANCE_TO_FRONT;
  }

  /**
   * distance between the center of the robot and the center of the goal
   * @return distance in inches
   */
  public double distanceCenterToCenter(){
    return distanceToTarget() + Constants.LL_DISTANCE_TO_ROBOT_CENTER + Constants.LL_TARGET_RADIUS;
  }

  /**
   * returns the speed the shooter should be for the high goal, based on the limelight
   * @return speed in native units
   */
  public double getShooterHighSpeed(){
    return Constants.SHOOTER_HIGH_SPEEDS_TABLE.lookup(distanceFrontToFender());
  }

  /**
   * returns the speed the shooter should be for the low goal, based on the limelight
   * @return speed in native units
   */
  public double getShooterLowSpeed(){
    return Constants.SHOOTER_LOW_SPEEDS_TABLE.lookup(distanceFrontToFender());
  }

  /**
   * returns the angle the hood should be for the high goal, based on the limelight
   * @return angle in degrees
   */
  public double getHoodHighAngle(){
    return Constants.HOOD_HIGH_POSITION_TABLE.lookup(distanceFrontToFender());
  }

  /**
   * returns the angle the hood should be for the low goal, based on the limelight
   * @return angle in degrees
   */
  public double getHoodLowAngle(){
    return Constants.HOOD_LOW_POSITION_TABLE.lookup(distanceFrontToFender());
  }
}
