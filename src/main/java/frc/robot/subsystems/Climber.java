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
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Climber extends SubsystemBase {
  DoubleSolenoid tiltRobot; // attatched to the fixed arm
  TalonFX extendoMotorLeft; // attatched to the left extending arm
  TalonFX extendoMotorRight; // attatched to the right extending arm, not mechanically linked to left
  DigitalInput leftBottomSwitch;
  DigitalInput rightBottomSwitch;

  /** Creates a new Climber. */
  public Climber() {
    tiltRobot = new DoubleSolenoid(PneumaticsModuleType.REVPH, 
      Constants.CLIMBER_TILT_IN, Constants.CLIMBER_TILT_OUT);

    extendoMotorLeft = new TalonFX(Constants.CLIMBER_LEFT_ARM);
    extendoMotorLeft.configFactoryDefault();
    // use the integrated sensor with the primary closed loop and timeout is 0.
    extendoMotorLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    extendoMotorLeft.configSelectedFeedbackCoefficient(1);//TODO: based on gearing, find out
    extendoMotorLeft.setNeutralMode(NeutralMode.Brake);
    extendoMotorLeft.setInverted(false);// Set motor inverted(set to false) TODO:Is this right?
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

    extendoMotorRight = new TalonFX(Constants.CLIMBER_RIGHT_ARM);
    extendoMotorRight.configFactoryDefault();
    // use the integrated sensor with the primary closed loop and timeout is 0.
    extendoMotorRight.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    extendoMotorRight.configSelectedFeedbackCoefficient(1);//TODO: based on gearing,find out
    extendoMotorRight.setNeutralMode(NeutralMode.Brake);
    extendoMotorRight.setInverted(false);// Set motor inverted(set to false) TODO:Is this right?
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
    
    leftBottomSwitch = new DigitalInput(Constants.CLIMBER_LEFT_BOTTOM_SWITCH);
    rightBottomSwitch = new DigitalInput(Constants.CLIMBER_RIGHT_BOTTOM_SWITCH);
  }

  @Override
  public void periodic() {}

  public void climberTiltOut(){
    tiltRobot.set(Value.kForward);
  }

  public void climberTiltIn(){
    tiltRobot.set(Value.kReverse);
  }

  //TODO: Make a method to move the extendoArm to a specific distance(use built-in MotionProfile?), def use postitioncontrol
  // public void extendoArmOut(){
  //   BufferedTrajectoryPointStream stream = new BufferedTrajectoryPointStream(); // Creates a set of points telling the motor how to move
  //   TrajectoryPoint middleTrajectoryPoint = new TrajectoryPoint(); // Creates single point telling the motor how to move
  //   TrajectoryPoint lastTrajectoryPoint = new TrajectoryPoint();

  //   //TODO: Might need to make a start position

  //   // The following sets our middle position and speed
  //   middleTrajectoryPoint.position = 10;
  //   middleTrajectoryPoint.velocity = Constants.CLIMBER_EXTENDO_SPEED_OUT;
  //   stream.Write(middleTrajectoryPoint); // Add trajectory point to stream
  //   // The following sets our last position and speed
  //   lastTrajectoryPoint.position = 20;
  //   lastTrajectoryPoint.velocity = 0; // Must be set to 0 or motor will spin indefinitely
  //   stream.Write(lastTrajectoryPoint);

  //   // Creates a motion profile using set of points in stream and the control mode
  //   extendoMotorRight.startMotionProfile(stream, extendoMotorRight.getMotionProfileTopLevelBufferCount(), ControlMode.Position);
  // }

  public void extendoRightSetPos(double pos) {
    extendoMotorRight.set(TalonFXControlMode.MotionMagic, pos);
  }

  public void extendoArmRightIn(){
    extendoMotorRight.set(TalonFXControlMode.PercentOutput, 
    Constants.CLIMBER_EXTENDO_SPEED_IN);
  }

  public void stopExtendoRightArm() {
    extendoMotorRight.set(TalonFXControlMode.PercentOutput, 0);
  }

  public double getExtendoRightCurrent() {
    return extendoMotorRight.getSupplyCurrent();
  }

  public boolean getExtendoRightSwitch() {
    return rightBottomSwitch.get();
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

  //TODO: Make match extendoArmOut()
  // public void fixedArmOut(){
  //   BufferedTrajectoryPointStream stream = new BufferedTrajectoryPointStream();
  //   TrajectoryPoint trajectoryPoint = new TrajectoryPoint();

  //   trajectoryPoint.position = 10;
  //   trajectoryPoint.velocity = Constants.CLIMBER_EXTENDO_SPEED_OUT;
  //   stream.Write(trajectoryPoint);
    
  //   int minBufferedPts = extendoMotorLeft.getMotionProfileTopLevelBufferCount();
  //   extendoMotorLeft.startMotionProfile(stream, minBufferedPts, ControlMode.Position);
  // }

  public void extendoLeftSetPos(double pos) {
    extendoMotorLeft.set(TalonFXControlMode.MotionMagic, pos);
  }

  public void extendoArmLeftIn(){
    extendoMotorLeft.set(TalonFXControlMode.PercentOutput, 
    Constants.CLIMBER_EXTENDO_SPEED_IN);
  }

  public void stopExtendoLeftArm() {
    extendoMotorLeft.set(TalonFXControlMode.PercentOutput, 0);
  }

  public double getExtendoLeftCurrent() {
    return extendoMotorLeft.getSupplyCurrent();
  }

  public boolean getExtendoLeftSwitch() {
    return leftBottomSwitch.get();
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
