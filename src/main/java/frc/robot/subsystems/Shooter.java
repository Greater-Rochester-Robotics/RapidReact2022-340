// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.DemandType;
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
 * 
 */
public class Shooter extends SubsystemBase { 
  // private static final double speedError = 0.01;
  TalonFX motor;
  

  /** Creates a new Shooter. */
  public Shooter() {
    motor = new TalonFX(Constants.SHOOTER_SHOOTING_MOTOR);
    motor.configFactoryDefault();
    // mainMotor.configSelectedFeedbackCoefficient(coefficient);//do not set, best to let motor be in ticks
    motor.setNeutralMode(NeutralMode.Coast);//This should Never be brake
    motor.setInverted(false);//on robot no inversion needed
    motor.enableVoltageCompensation(true);
    motor.configVoltageCompSaturation(Constants.MAXIMUM_VOLTAGE);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);//this is the shooter speed we need
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 251);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 241);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 239);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 233);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 229);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus, 255);

    motor.config_kP(0, Constants.SHOOTER_MAIN_MOTOR_P);
    motor.config_kI(0, Constants.SHOOTER_MAIN_MOTOR_I);
    motor.config_kD(0, Constants.SHOOTER_MAIN_MOTOR_D);
    motor.config_kF(0, Constants.SHOOTER_MAIN_MOTOR_F);
    motor.configAllowableClosedloopError(0, Constants.SHOOTER_MOTOR_ALLOWABLE_ERROR);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Shooter Speed", getSpeed());
    // SmartDashboard.putBoolean("AtSpeed",isAtSpeed());
  }

  /**
   * Gets the RPM of the shooter
   * @return in revolutions per minute
   */
  public double getSpeed(){
    return motor.getSelectedSensorVelocity() / Constants.SHOOTER_MOTOR_PULSES_PER_REV ;
  }

  /**
   * sets the setpoint on the shooter motor,
   * @param speed speed in RPM
   */
  public void setSpeed(double speed){
    motor.set(TalonFXControlMode.Velocity, speed * Constants.SHOOTER_MOTOR_PULSES_PER_REV);//, DemandType.ArbitraryFeedForward, (speed / 15000));
  }

  /**
   * set the motor to a percent output, for use in testing and spitting balls out.
   * @param percentOutput
   */
  public void setOutput(double percentOutput){
    motor.set(TalonFXControlMode.PercentOutput, percentOutput);
  }
  
  /**
   * don't use till PID is on setpoint, which it runs clearly short of.
   * @return true if shooter is at speed, within tolerance
   */
  @Deprecated
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
