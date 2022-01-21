// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  //TODO:Instantiate DoubleSolenoid for harvester in/out
  DoubleSolenoid doubleSolenoid;
  //TODO: find what motor harvesterMotor is.
  CANSparkMax harvesterMotor;
  /** Creates a new Intake. */
  public Intake() {
    //TODO:construct
    doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 
      Constants.HARVESTER_TILT_IN, Constants.HARVESTER_TILT_OUT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  //TODO:make modifier methods outHarvester inHarvester
  
}
