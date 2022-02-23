// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Hood extends SubsystemBase {
  CANSparkMax hoodMotor;
  RelativeEncoder hoodEncoder;
  SparkMaxPIDController pidController;
  DigitalInput hoodLimitSwitch;

  /** Creates a new Hood. */
  public Hood() {
    hoodMotor = new CANSparkMax(Constants.SHOOTER_HOOD_MOTOR, MotorType.kBrushless);
    hoodMotor.restoreFactoryDefaults();
    hoodMotor.setIdleMode(IdleMode.kBrake);
    hoodMotor.enableVoltageCompensation(10.5);
    hoodMotor.setInverted(false); //this is the right direction

    hoodEncoder = hoodMotor.getEncoder();
    hoodEncoder.setPositionConversionFactor(Constants.HOOD_DEGREE_CONVERSION);

    pidController = hoodMotor.getPIDController();
    pidController.setP(Constants.HOOD_MOTOR_P);
    pidController.setI(Constants.HOOD_MOTOR_I);
    pidController.setD(Constants.HOOD_MOTOR_D);
    pidController.setFF(Constants.HOOD_MOTOR_FF);//changed gearbox, no FF needed

    hoodMotor.burnFlash();

    hoodLimitSwitch = new DigitalInput(Constants.SHOOTER_HOOD_SWITCH);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("HoodLimit", !hoodLimitSwitch.get());
    SmartDashboard.putNumber("Hood Position", getHoodPosition());
  }

  /**
   * A method to use the pidController to set the hood to an angle
   */
  public void setHoodPosition(double hoodPos) {
    pidController.setReference(hoodPos, ControlType.kPosition);
  }

  /**
   * Accessor method for the position of the hood. 
   */
  public double getHoodPosition() {
    return hoodEncoder.getPosition();
  }

  /**
   * Create reset position modifier method that will set the position of the encoder object. this will act as our reset device, use setPosion of encoder object
   */
  public void resetHoodEncoderPosition(){
    resetHoodEncoderPosition(0);
  }
  
  /**
   * Set the current position of the hood to the 
   * @param position
   */
  public void resetHoodEncoderPosition(double position) {
    hoodEncoder.setPosition(position);
  }

  /**
   * A homeMethod that returns a true when limit switch is 
   * pressed, and drives motor backwards with percentVoltage.
   * It resets the hood encoder with resetHoodEncoderPosition
   * when the switch is pressed.
   */
  public boolean homeHoodPosition() {
    
    if(!hoodLimitSwitch.get()) {
      resetHoodEncoderPosition();
      hoodMotor.set(0.0);
      return true;
    }else{
      hoodMotor.set(Constants.HOOD_HOMING_SPEED);
      return false;
    }
  }

  /**
   * Stops the hood motor
   */
  public void stopHoodMotor(){
    hoodMotor.set(0.0);
  }
}
