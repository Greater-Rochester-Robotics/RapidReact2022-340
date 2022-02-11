// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Climber extends SubsystemBase {
  DoubleSolenoid tiltRobot; // attatched to the fixed arm
  TalonFX extendoMotorLeft; // attatched to the left extending arm
  TalonFX extendoMotorRight; // attatched to the right extending arm, not mechanically linked to left
  TalonFXSensorCollection leftBottomSwitch;
  TalonFXSensorCollection rightBottomSwitch;

  /** Creates a new Climber. */
  public Climber() {
    // tiltRobot = new DoubleSolenoid(PneumaticsModuleType.REVPH, 
      // Constants.CLIMBER_TILT_IN, Constants.CLIMBER_TILT_OUT);

    // Configures the Left Extendo Motor
    extendoMotorLeft = new TalonFX(Constants.CLIMBER_LEFT_ARM);
    extendoMotorLeft.configFactoryDefault();
    // use the integrated sensor with the primary closed loop and timeout is 0.
    extendoMotorLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    extendoMotorLeft.configSelectedFeedbackCoefficient(Constants.EXTENDO_INCHES_PER_PULSE_CONVERSION_FACTOR);
    extendoMotorLeft.setNeutralMode(NeutralMode.Brake);
    extendoMotorLeft.setInverted(true);// Set motor inverted(set to true for left)
    extendoMotorLeft.setSensorPhase(false);
    extendoMotorLeft.enableVoltageCompensation(true);
    extendoMotorLeft.configVoltageCompSaturation(Constants.MAXIMUM_VOLTAGE);
    extendoMotorLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10);
    extendoMotorLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);
    extendoMotorLeft.setSelectedSensorPosition(0.0);
    extendoMotorLeft.config_kP(0, Constants.EXTENDO_MOTOR_P);
    extendoMotorLeft.config_kI(0, Constants.EXTENDO_MOTOR_I);
    extendoMotorLeft.config_kD(0, Constants.EXTENDO_MOTOR_D);
    extendoMotorLeft.config_kF(0, Constants.EXTENDO_MOTOR_F);
    extendoMotorLeft.configMotionCruiseVelocity(Constants.EXTENDO_CRUISE_VELOCITY);
    extendoMotorLeft.configMotionAcceleration(Constants.EXTENDO_ACCELERATION);
    this.setLeftSwitchEnabled(true);
    extendoMotorLeft.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyOpen);

    // Configures the Right Extendo Motor
    extendoMotorRight = new TalonFX(Constants.CLIMBER_RIGHT_ARM);
    extendoMotorRight.configFactoryDefault();
    // use the integrated sensor with the primary closed loop and timeout is 0.
    extendoMotorRight.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    extendoMotorRight.configSelectedFeedbackCoefficient(Constants.EXTENDO_INCHES_PER_PULSE_CONVERSION_FACTOR);
    extendoMotorRight.setNeutralMode(NeutralMode.Brake);
    extendoMotorRight.setInverted(false);// Set motor inverted(set to false for right)
    extendoMotorRight.setSensorPhase(true);
    extendoMotorRight.enableVoltageCompensation(true);
    extendoMotorRight.configVoltageCompSaturation(Constants.MAXIMUM_VOLTAGE);
    extendoMotorRight.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10);
    extendoMotorRight.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);
    extendoMotorRight.setSelectedSensorPosition(0.0);
    extendoMotorRight.config_kP(0, Constants.EXTENDO_MOTOR_P);
    extendoMotorRight.config_kI(0, Constants.EXTENDO_MOTOR_I);
    extendoMotorRight.config_kD(0, Constants.EXTENDO_MOTOR_D);
    extendoMotorRight.config_kF(0, Constants.EXTENDO_MOTOR_F);
    extendoMotorRight.configMotionCruiseVelocity(Constants.EXTENDO_CRUISE_VELOCITY);
    extendoMotorRight.configMotionAcceleration(Constants.EXTENDO_ACCELERATION);
    this.setRightSwitchEnabled(true);
    extendoMotorRight.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyOpen);
    
    // Tells us if the extendo arms are all the way in.
    leftBottomSwitch = extendoMotorLeft.getSensorCollection();// DigitalInput(Constants.CLIMBER_LEFT_BOTTOM_SWITCH);
    rightBottomSwitch = extendoMotorRight.getSensorCollection();// DigitalInput(Constants.CLIMBER_RIGHT_BOTTOM_SWITCH);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Encoder", extendoMotorLeft.getSelectedSensorPosition());
    SmartDashboard.putNumber("Right Encoder", extendoMotorRight.getSelectedSensorPosition());
    System.out.println(extendoMotorRight.getSelectedSensorPosition());
  }

  /* Functions for the fixed arm */

  public void climberTiltOut(){
    tiltRobot.set(Value.kForward);
  }

  public void climberTiltIn(){
    tiltRobot.set(Value.kReverse);
  }

  /* Functions for the right extendo arm */

  public void extendoRightSetPos(double pos) {
    extendoMotorRight.set(TalonFXControlMode.MotionMagic, pos);
  }

  public void extendoArmRightIn(){
    extendoMotorRight.set(TalonFXControlMode.PercentOutput, 
    Constants.CLIMBER_EXTENDO_SPEED_IN);
  }

  public void extendoArmRightOut(){
    extendoMotorRight.set(TalonFXControlMode.PercentOutput, 
    Constants.CLIMBER_EXTENDO_SPEED_OUT);
  }

  public void stopExtendoRightArm() {
    extendoMotorRight.set(TalonFXControlMode.PercentOutput, 0);
  }

  public double getExtendoRightCurrent() {
    return extendoMotorRight.getSupplyCurrent();
  }

  public boolean getExtendoRightSwitch() {
    return rightBottomSwitch.isRevLimitSwitchClosed()==1;
  }

  public void setRightSwitchEnabled(boolean enable){
    if(enable){
      extendoMotorRight.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    }else{
      extendoMotorRight.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyOpen);
    }
  }
  
  public double getExtendoRightEncPos() {
    return extendoMotorRight.getSelectedSensorPosition();
  }

  public double getExtendoRightEncVel() {
    return extendoMotorRight.getSelectedSensorVelocity();
  }

  public void setExtendoRightEnc(double sensorPos) {
    extendoMotorRight.setSelectedSensorPosition(sensorPos);
  }

  /* Functions for the left extendo arm */
  
  public void extendoLeftSetPos(double pos) {
    extendoMotorLeft.set(TalonFXControlMode.MotionMagic, pos);
  }

  public void extendoArmLeftIn(){
    extendoMotorLeft.set(TalonFXControlMode.PercentOutput, 
    Constants.CLIMBER_EXTENDO_SPEED_IN);
  }
  
  public void extendoArmLeftOut(){
    extendoMotorLeft.set(TalonFXControlMode.PercentOutput, 
    Constants.CLIMBER_EXTENDO_SPEED_OUT);
  }

  public void stopExtendoLeftArm() {
    extendoMotorLeft.set(TalonFXControlMode.PercentOutput, 0);
  }

  public double getExtendoLeftCurrent() {
    return extendoMotorLeft.getSupplyCurrent();
  }

  public boolean getExtendoLeftSwitch() {
    return leftBottomSwitch.isRevLimitSwitchClosed()==1;
  }
  
  public void setLeftSwitchEnabled(boolean enable){
    if(enable){
      extendoMotorLeft.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    }else{
      extendoMotorLeft.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyOpen);
    }
  }

  public double getExtendoLeftEncPos() {
    return extendoMotorLeft.getSelectedSensorPosition();
  }

  public double getExtendoLeftEncVel() {
    return extendoMotorLeft.getSelectedSensorVelocity();
  }

  public void setExtendoLeftEnc(double sensorPos) {
    extendoMotorLeft.setSelectedSensorPosition(sensorPos);
  }

}
