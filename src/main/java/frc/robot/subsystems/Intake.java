// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Intake extends SubsystemBase {

  DoubleSolenoid harvesterTilt;
  //TODO: find what motor harvesterMotor is.
  CANSparkMax harvesterMotor;
  
  /** Creates a new Intake. */
  public Intake() {
    harvesterTilt = new DoubleSolenoid(PneumaticsModuleType.REVPH, 
      Constants.HARVESTER_TILT_IN, Constants.HARVESTER_TILT_OUT);
    harvesterMotor = new CANSparkMax(Constants.HARVESTER_MOTOR, MotorType.kBrushless);
    //TODO: Configure CANSparkMax motor.
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void tiltIn(){
    harvesterTilt.set(Value.kReverse);
  }

  public void tiltOut(){
    harvesterTilt.set(Value.kForward);
  }

  public void intake(){
    harvesterMotor.set(Constants.HARVESTER_INTAKE_SPEED);
  }

  public void extake(){
    harvesterMotor.set(Constants.HARVESTER_EXTAKE_SPEED);
  }

  public void stopMotors(){
    harvesterMotor.set(0);
  }
  
}
