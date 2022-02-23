// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

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
  TalonFX motor;

  /** Creates a new Shooter. */
  public Shooter() {
    motor = new TalonFX(Constants.SHOOTER_SHOOTING_MOTOR);
    motor.configFactoryDefault();
    // mainMotor.configSelectedFeedbackCoefficient(coefficient);//do not set, best to let motor be in ticks
    motor.setNeutralMode(NeutralMode.Coast);//TODO: find if the was suposed to be in brake
    motor.setInverted(false);//on robot no inversion needed
    motor.enableVoltageCompensation(true);
    motor.configVoltageCompSaturation(Constants.MAXIMUM_VOLTAGE);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);

    motor.config_kP(0, Constants.SHOOTER_MAIN_MOTOR_P);
    motor.config_kI(0, Constants.SHOOTER_MAIN_MOTOR_I);
    motor.config_kD(0, Constants.SHOOTER_MAIN_MOTOR_D);
    motor.config_kF(0, Constants.SHOOTER_MAIN_MOTOR_F);
    motor.configAllowableClosedloopError(0, Constants.SHOOTER_MOTOR_ALLOWABLE_ERROR);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Speed", getSpeed());
  }

  /**
   * Gets the RPM of the shooter
   * @return in revolutions per minute
   */
  public double getSpeed(){
    return motor.getSelectedSensorVelocity() / Constants.SHOOTER_MOTOR_PUSLES_PER_REV ;
  }

  /**
   * sets the setpoint on the shooter motor,
   * @param speed speed in RPM
   */
  public void setSpeed(double speed){
    motor.set(TalonFXControlMode.Velocity, speed * Constants.SHOOTER_MOTOR_PUSLES_PER_REV);
  }

  public void setOutput(double percentOutput){
    motor.set(TalonFXControlMode.PercentOutput, percentOutput);
  }
  
  /**
   * @return if shooter is at speed, within tolerance
   */
  public boolean isAtSpeed() {
    return Math.abs(motor.getClosedLoopError()) < Constants.SHOOTER_MOTOR_ALLOWABLE_ERROR;
  }

  /**
   * stops the shooter motor
   */
  public void stopMotor(){
    motor.set(TalonFXControlMode.PercentOutput, 0.0);
  }
}
