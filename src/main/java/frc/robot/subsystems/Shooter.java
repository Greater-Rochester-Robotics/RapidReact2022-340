// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This is the class that controls the shooter.
 * Currently, it controls a single main motor.
 * TODO: Revisit when mechanical figures out what the shooter is
 * 
 */
public class Shooter extends SubsystemBase { 
  private static final double speedError = 0.01;
  private double goalSpeed;
  TalonFX mainMotor;
  TalonFX followMotor;
  /** Creates a new Shooter. */
  public Shooter(int mainMotorID, int followMotorID) {
    mainMotor = new TalonFX(mainMotorID);
    followMotor = new TalonFX(followMotorID);
    //TODO: write more to configure the motors.
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getSpeed(){
    return mainMotor.getSelectedSensorVelocity();
  }

  public void setSpeed(double speed){
    mainMotor.set(TalonFXControlMode.Velocity, speed);
  }
  
  /**
   * @return if it is at speed within tolerance
   */
  public boolean isAtSpeed(){
    //TalonFX has an isAtSpeed() with tolerance already
    return ((goalSpeed * (1.00 - speedError) <= getSpeed()) && 
            (goalSpeed * (1.00 + speedError) >= getSpeed()));
  }

  public void stopMotors(){
    mainMotor.set(TalonFXControlMode.PercentOutput, 0.0);
  }
}
