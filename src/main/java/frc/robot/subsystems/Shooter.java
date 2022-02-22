// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * This is the class that controls the shooter.
 * Currently, it controls a single main motor.
 * TODO: Revisit mechanical knows what this looks like now
 * 
 */
public class Shooter extends SubsystemBase { 
  private static final double speedError = 0.01;
  private double goalSpeed;
  TalonFX mainMotor;
  CANSparkMax hoodMotor;
  RelativeEncoder hoodEncoder;
  SparkMaxPIDController pidController;
  DigitalInput hoodLimitSwitch;

  /** Creates a new Shooter. */
  public Shooter() {
    mainMotor = new TalonFX(Constants.MAIN_SHOOTER_MOTOR);
    mainMotor.configFactoryDefault();
    // mainMotor.configSelectedFeedbackCoefficient(coefficient);//TODO: set to an RPM based on the gear ratio
    mainMotor.setNeutralMode(NeutralMode.Brake);
    mainMotor.setInverted(false);//TODO: check if, on robot, forward is out.
    mainMotor.enableVoltageCompensation(true);
    mainMotor.configVoltageCompSaturation(Constants.MAXIMUM_VOLTAGE);
    mainMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10);
    mainMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);

    mainMotor.config_kP(0, Constants.SHOOTER_MAIN_MOTOR_P);
    mainMotor.config_kI(0, Constants.SHOOTER_MAIN_MOTOR_I);
    mainMotor.config_kD(0, Constants.SHOOTER_MAIN_MOTOR_D);
    mainMotor.config_kF(0, Constants.SHOOTER_MAIN_MOTOR_F);
    mainMotor.configAllowableClosedloopError(0, Constants.SHOOTER_MOTOR_ALLOWABLE_ERROR);
    //TODO: uncomment, this was done because they needed to test an incomplete shooter
    hoodMotor = new CANSparkMax(Constants.SHOOTER_HOOD_MOTOR, MotorType.kBrushless);
    hoodMotor.restoreFactoryDefaults();
    hoodMotor.setIdleMode(IdleMode.kBrake);
    hoodMotor.enableVoltageCompensation(10.5);
    hoodMotor.setInverted(false); //TODO: check if this is right

    hoodEncoder = hoodMotor.getEncoder();
    hoodEncoder.setPositionConversionFactor(Constants.SHOOTER_HOOD_DEGREE_CONVERSION);

    pidController = hoodMotor.getPIDController();
    pidController.setP(Constants.SHOOTER_HOOD_MOTOR_P);
    pidController.setI(Constants.SHOOTER_HOOD_MOTOR_I);
    pidController.setD(Constants.SHOOTER_HOOD_MOTOR_D);
    pidController.setFF(Constants.SHOOTER_HOOD_MOTOR_FF);//TODO: May need arbitrary ff

    hoodMotor.burnFlash();

    hoodLimitSwitch = new DigitalInput(Constants.SHOOTER_HOOD_SWITCH);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("HoodLimit", !hoodLimitSwitch.get());
    SmartDashboard.putNumber("Hood Position", getHoodPosition());
    SmartDashboard.putNumber("Shooter Speed", getSpeed());
  }

  public double getSpeed(){
    return mainMotor.getSelectedSensorVelocity();
  }

  public void setSpeed(double speed){
    mainMotor.set(TalonFXControlMode.Velocity, speed);
  }

  public void setOutput(double percentOutput){
    mainMotor.set(TalonFXControlMode.PercentOutput, percentOutput);
  }
  
  /**
   * @return if it is at speed within tolerance
   */
  public boolean isAtSpeed(double goalSpeed) {
    //TalonFX has an isAtSpeed() with tolerance already
    return ((goalSpeed * (1.00 - speedError) <= getSpeed()) && 
            (goalSpeed * (1.00 + speedError) >= getSpeed()));
  }

  public void stopShooterMotor(){
    mainMotor.set(TalonFXControlMode.PercentOutput, 0.0);
  }

  public void stopHoodMotor(){
    hoodMotor.set(0.0);
  }

  /**
   * create a setHoodPosition for the hoodMotor. use the pidController object to set in method
   */
  public void setHoodPosition(double hoodPos) {
    pidController.setReference(hoodPos, ControlType.kPosition);
  }

  /**
   * create accessor method for the position of the hood. use the sparkmax encoderObject as source
   */
  public double getHoodPosition() {
    return hoodEncoder.getPosition();
  }

  /**
   * create reset position modifier method that will set the position of the encoder object. this will act as our reset device, use setPosion of encoder object
   */
  public void resetHoodEncoderPosition(){
    resetHoodEncoderPosition(0);
  }
  
  public void resetHoodEncoderPosition(double position) {
    hoodEncoder.setPosition(position);
  }

  /**
   * create a homeMethod that returns a true when limit switch is pressed, and drives motor backwards with percentVoltage.(this would be using hoodMotor.set) use the previous resetPosition
   */
  public boolean homeHoodPosition() {
    
    if(!hoodLimitSwitch.get()) {
      resetHoodEncoderPosition();
      hoodMotor.set(0.0);
      return true;
    }else{
      hoodMotor.set(-.10);
      return false;
    }
  }
}
