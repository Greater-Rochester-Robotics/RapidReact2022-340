// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Hood extends SubsystemBase {
  CANSparkMax motor;
  RelativeEncoder encoder;
  SparkMaxPIDController pidController;
  DigitalInput limitSwitch;
  boolean hasBeenHomed;

  /** Creates a new Hood. */
  public Hood() {
    motor = new CANSparkMax(Constants.SHOOTER_HOOD_MOTOR, MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kBrake);
    motor.enableVoltageCompensation(10.5);
    motor.setInverted(false); //this is the right direction

    encoder = motor.getEncoder();
    encoder.setPositionConversionFactor(Constants.HOOD_DEGREE_CONVERSION);

    pidController = motor.getPIDController();
    pidController.setP(Constants.HOOD_MOTOR_P);
    pidController.setI(Constants.HOOD_MOTOR_I);
    pidController.setD(Constants.HOOD_MOTOR_D);
    pidController.setFF(Constants.HOOD_MOTOR_FF);//changed gearbox, no FF needed

    motor.setSoftLimit(SoftLimitDirection.kForward, (float) (Constants.HOOD_FORWARD_LIMIT_DEGREES/Constants.HOOD_DEGREE_CONVERSION));
    motor.enableSoftLimit(SoftLimitDirection.kForward, true);//This doesn't work, right, added code in set position

    motor.burnFlash();

    limitSwitch = new DigitalInput(Constants.SHOOTER_HOOD_SWITCH);

    hasBeenHomed = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("HoodLimit", !limitSwitch.get());
    // SmartDashboard.putNumber("Hood Position", getPosition());
  }

  public boolean hasBeenHomed(){
    return hasBeenHomed;
  }

  public void setBeenHomed(boolean hasBeenHomed){
    this.hasBeenHomed = hasBeenHomed;
  }

  /**
   * A method to use the pidController to set the hood to an angle
   */
  public void setPosition(double position) {
    if(position > Constants.HOOD_FORWARD_LIMIT_DEGREES){
      position = Constants.HOOD_FORWARD_LIMIT_DEGREES;
    }
    pidController.setReference(position, ControlType.kPosition);
  }

  /**
   * Accessor method for the position of the hood. 
   */
  public double getPosition() {
    return encoder.getPosition();
  }

  /**
   * Create reset position modifier method that will set the position of the encoder object. this will act as our reset device, use setPosion of encoder object
   */
  public void resetEncoderPosition(){
    resetEncoderPosition(0);
  }
  
  /**
   * Set the current position of the hood to the 
   * @param position
   */
  public void resetEncoderPosition(double position) {
    encoder.setPosition(position);
  }

  /**
   * A homeMethod that returns a true when limit switch is 
   * pressed, and drives motor backwards with percentVoltage.
   * It resets the hood encoder with resetHoodEncoderPosition
   * when the switch is pressed.
   */
  public boolean homePosition() {
    
    if(!limitSwitch.get()) {
      resetEncoderPosition();
      setBeenHomed(true);
      motor.set(0.0);
      return true;
    }else{
      motor.set(Constants.HOOD_HOMING_SPEED);
      return false;
    }
  }

  /**
   * Stops the hood motor
   */
  public void stopMotor(){
    motor.set(0.0);
  }
}
