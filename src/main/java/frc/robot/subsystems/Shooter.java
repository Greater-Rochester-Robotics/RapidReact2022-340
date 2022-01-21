// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  //TODO: instantiate two shooter motors(main/follower) Falcon500
  /** Creates a new Shooter. */
  public Shooter() {
    //TODO:construct shooter motors
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //TODO: methods getSpeed(), setSpeed, isAtSpeed, stopMotors
  public double getSpeed(){
    return  0.0;
  }
  public void setSpeed(double speed){
    
  }
  public boolean isAtSpeed(){
    return false;
  }
  public void stopMotors(){
    
  }
}
