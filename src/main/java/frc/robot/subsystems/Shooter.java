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
  TalonFX shooterMotor;
  CANSparkMax hoodMotor;
  RelativeEncoder hoodEncoder;
  SparkMaxPIDController pidController;
  DigitalInput hoodLimitSwitch;

  /** Creates a new Shooter. */
  public Shooter() {
    shooterMotor = new TalonFX(Constants.SHOOTER_SHOOTING_MOTOR);
    shooterMotor.configFactoryDefault();
    // mainMotor.configSelectedFeedbackCoefficient(coefficient);//do not set, best to let motor be in ticks
    shooterMotor.setNeutralMode(NeutralMode.Coast);//TODO: find if the was suposed to be in brake
    shooterMotor.setInverted(false);//on robot no inversion needed
    shooterMotor.enableVoltageCompensation(true);
    shooterMotor.configVoltageCompSaturation(Constants.MAXIMUM_VOLTAGE);
    shooterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10);
    shooterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);

    shooterMotor.config_kP(0, Constants.SHOOTER_MAIN_MOTOR_P);
    shooterMotor.config_kI(0, Constants.SHOOTER_MAIN_MOTOR_I);
    shooterMotor.config_kD(0, Constants.SHOOTER_MAIN_MOTOR_D);
    shooterMotor.config_kF(0, Constants.SHOOTER_MAIN_MOTOR_F);
    shooterMotor.configAllowableClosedloopError(0, Constants.SHOOTER_MOTOR_ALLOWABLE_ERROR);
    
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
    SmartDashboard.putNumber("Shooter Speed", getSpeed());
  }

  /**
   * Gets the RPM of the shooter
   * @return in revolutions per minute
   */
  public double getSpeed(){
    return shooterMotor.getSelectedSensorVelocity() / Constants.SHOOTER_MOTOR_PUSLES_PER_REV ;
  }

  /**
   * sets the setpoint on the shooter motor,
   * @param speed speed in RPM
   */
  public void setSpeed(double speed){
    shooterMotor.set(TalonFXControlMode.Velocity, speed * Constants.SHOOTER_MOTOR_PUSLES_PER_REV);
  }

  public void setOutput(double percentOutput){
    shooterMotor.set(TalonFXControlMode.PercentOutput, percentOutput);
  }
  
  /**
   * @return if shooter is at speed, within tolerance
   */
  public boolean isAtSpeed() {
    return Math.abs(shooterMotor.getClosedLoopError()) < Constants.SHOOTER_MOTOR_ALLOWABLE_ERROR;
  }

  /**
   * stops the shooter motor
   */
  public void stopShooterMotor(){
    shooterMotor.set(TalonFXControlMode.PercentOutput, 0.0);
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
