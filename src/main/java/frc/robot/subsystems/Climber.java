// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Climber extends SubsystemBase {
  DoubleSolenoid tiltRobot; // attatched to the fixed arm
  DoubleSolenoid dampingBar;
  TalonFX extendoMotorLeft; // attatched to the left extending arm
  TalonFX extendoMotorRight; // attatched to the right extending arm, not mechanically linked to left
  TalonFXSensorCollection leftBottomSwitch;
  TalonFXSensorCollection rightBottomSwitch;

  /** Creates a new Climber. */
  public Climber() {
    tiltRobot = new DoubleSolenoid(PneumaticsModuleType.REVPH, 
      Constants.CLIMBER_TILT_IN, Constants.CLIMBER_TILT_OUT);

    dampingBar = new DoubleSolenoid(PneumaticsModuleType.REVPH,
      Constants.CLIMBER_DAMPING_RIGHT, Constants.CLIMBER_DAMPING_LEFT);

    // Configures the Left Extendo Motor
    extendoMotorLeft = new TalonFX(Constants.CLIMBER_LEFT_ARM);
    extendoMotorLeft.configFactoryDefault();
    // use the integrated sensor with the primary closed loop and timeout is 0.
    extendoMotorLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    extendoMotorLeft.configSelectedFeedbackCoefficient(Constants.SELECTED_FEEDBACK_COEFFICIENT);
    extendoMotorLeft.setNeutralMode(NeutralMode.Brake);
    extendoMotorLeft.setInverted(true);// Set motor inverted(set to true for left)
    extendoMotorLeft.setSensorPhase(false);
    extendoMotorLeft.enableVoltageCompensation(true);
    extendoMotorLeft.configVoltageCompSaturation(Constants.MAXIMUM_VOLTAGE);
    extendoMotorLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);//for limit switch
    extendoMotorLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);//for encoder
    extendoMotorLeft.setSelectedSensorPosition(0.0);
    extendoMotorLeft.config_kP(0, Constants.EXTENDO_MOTOR_P);
    extendoMotorLeft.config_kI(0, Constants.EXTENDO_MOTOR_I);
    extendoMotorLeft.config_kD(0, Constants.EXTENDO_MOTOR_D);
    extendoMotorLeft.config_kF(0, Constants.EXTENDO_MOTOR_F);
    extendoMotorLeft.configMotionCruiseVelocity(Constants.EXTENDO_CRUISE_VELOCITY);
    extendoMotorLeft.configMotionAcceleration(Constants.EXTENDO_ACCELERATION);
    this.setLeftSwitchEnabled(true);//enable the reverse limit
    //disable the forward limit, nothing should be plugged in.
    extendoMotorLeft.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyOpen);

    // Configures the Right Extendo Motor
    extendoMotorRight = new TalonFX(Constants.CLIMBER_RIGHT_ARM);
    extendoMotorRight.configFactoryDefault();
    // use the integrated sensor with the primary closed loop and timeout is 0.
    extendoMotorRight.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    extendoMotorRight.configSelectedFeedbackCoefficient(Constants.SELECTED_FEEDBACK_COEFFICIENT);
    extendoMotorRight.setNeutralMode(NeutralMode.Brake);
    extendoMotorRight.setInverted(false);// Set motor not inverted(set to false for right)
    extendoMotorRight.setSensorPhase(true);
    extendoMotorRight.enableVoltageCompensation(true);
    extendoMotorRight.configVoltageCompSaturation(Constants.MAXIMUM_VOLTAGE);
    extendoMotorRight.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);//for the limit switch
    extendoMotorRight.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);//for encoder
    extendoMotorRight.setSelectedSensorPosition(0.0);
    extendoMotorRight.config_kP(0, Constants.EXTENDO_MOTOR_P);
    extendoMotorRight.config_kI(0, Constants.EXTENDO_MOTOR_I);
    extendoMotorRight.config_kD(0, Constants.EXTENDO_MOTOR_D);
    extendoMotorRight.config_kF(0, Constants.EXTENDO_MOTOR_F);
    extendoMotorRight.config_kP(1, Constants.EXTENDO_MOTOR_P_COMPENSATE);
    extendoMotorRight.config_kI(1, Constants.EXTENDO_MOTOR_I_COMPENSATE);
    extendoMotorRight.config_kD(1, Constants.EXTENDO_MOTOR_D_COMPENSATE);
    extendoMotorRight.config_kF(1, Constants.EXTENDO_MOTOR_F_COMPENSATE);
    extendoMotorRight.configMotionCruiseVelocity(Constants.EXTENDO_CRUISE_VELOCITY);
    extendoMotorRight.configMotionAcceleration(Constants.EXTENDO_ACCELERATION);
    this.setRightSwitchEnabled(true);//enable the reverse limit
    //disable the forward limit, nothing should be plugged in.
    extendoMotorRight.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyOpen);

    // Tells us if the extendo arms are all the way in. 
    leftBottomSwitch = extendoMotorLeft.getSensorCollection();// DigitalInput(Constants.CLIMBER_LEFT_BOTTOM_SWITCH);
    rightBottomSwitch = extendoMotorRight.getSensorCollection();// DigitalInput(Constants.CLIMBER_RIGHT_BOTTOM_SWITCH);
  }

  @Override
  public void periodic() {
    if(!(getCurrentCommand() == null)) {
      SmartDashboard.putNumber("Left Climb Encoder", getExtendoLeftEncPos());
      SmartDashboard.putNumber("Right Climb Encoder", getExtendoRightEncPos());
      SmartDashboard.putBoolean("Left Climber Switch", getExtendoLeftSwitch());
      SmartDashboard.putBoolean("Right Climber Switch", getExtendoRightSwitch());
    }
  }

  /* ==================== Functions for the damping bars ==================== */

  public void dampingBarOut(){
    dampingBar.set(Value.kReverse);
  }

  public void dampingBarIn(){
    dampingBar.set(Value.kForward);
  }

  /* ==================== Functions for the fixed arms ==================== */

  /**
   * swing fixed arms out
   */
  public void climberTiltOut(){
    tiltRobot.set(Value.kReverse);
  }

  /**
   * swing fixed arms in
   */
  public void climberTiltIn(){
    tiltRobot.set(Value.kForward);
  }

/* ======================Functions for the pair of arms=====================*/
  /**
   * a setup method designed to link the right and left climber arms together. 
   * The right is set as Main and the left follower. Right reads the left's 
   * encoder and sends an altered ouput based on position difference. this 
   * the the Aux PID.
   * 
   * This can be run just once.
   */
  public void assignRemoteSensor(){
    TalonFXConfiguration rightConfig = new TalonFXConfiguration();
    TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    rightConfig.remoteFilter1.remoteSensorDeviceID = extendoMotorLeft.getDeviceID(); //Device ID of Remote Source
		rightConfig.remoteFilter1.remoteSensorSource = RemoteSensorSource.TalonFX_SelectedSensor; //Remote Source Type
    /* Master is not inverted, both sides are positive so we can di+ff them. */
    rightConfig.diff0Term = TalonFXFeedbackDevice.RemoteSensor1.toFeedbackDevice();    //Aux Selected Sensor
    rightConfig.diff1Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); //Local IntegratedSensor
    rightConfig.auxiliaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorDifference.toFeedbackDevice(); //Sum0 + Sum1
    /* With current diff terms, a counterclockwise rotation results in negative heading with a right master */
    rightConfig.auxPIDPolarity = true;
    //rightConfig.auxiliaryPID.selectedFeedbackCoefficient = Constants.kTurnTravelUnitsPerRotation / Constants.kEncoderUnitsPerRotation;
    int closedLoopTimeMs = 1;
		rightConfig.slot0.closedLoopPeriod = closedLoopTimeMs;
		rightConfig.slot1.closedLoopPeriod = closedLoopTimeMs;
    leftConfig.peakOutputForward = +1.0;
		leftConfig.peakOutputReverse = -1.0;
		rightConfig.peakOutputForward = +1.0;
		rightConfig.peakOutputReverse = -1.0;
    leftConfig.neutralDeadband = .001;
		rightConfig.neutralDeadband = .001;
    leftConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
    extendoMotorLeft.configAllSettings(leftConfig);
		extendoMotorRight.configAllSettings(rightConfig);
  }


  public void speedSensorFeedback(){
    extendoMotorRight.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20);
		extendoMotorRight.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20);
		extendoMotorRight.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20);
		extendoMotorLeft.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5);
  }
  
  public void slowSensorFeedback(){
    extendoMotorRight.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 1000);
		extendoMotorRight.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 1000);
		extendoMotorRight.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 1000);
		extendoMotorLeft.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 1000);
  }

  /**
   * set both motor positions using dual pids
   * @param position
   */
  public void setBothMotors(double position){
    extendoMotorRight.set(TalonFXControlMode.Position, position / Constants.EXTENDO_INCHES_PER_PULSE_CONVERSION_FACTOR, DemandType.AuxPID, 0.0);
		extendoMotorLeft.follow(extendoMotorRight, FollowerType.AuxOutput1);
  }

  /**
   * stop both motors, stops all PID funcions
   */
  public void stopBothMotors(){
    extendoMotorRight.set(ControlMode.PercentOutput, 0.0);
    extendoMotorLeft.set(ControlMode.PercentOutput, 0.0);
  }

  /* ==================== Functions for the right extendo arm ==================== */

  public void extendoRightSetPos(double pos) {
    extendoMotorRight.set(TalonFXControlMode.Position, (pos / Constants.EXTENDO_INCHES_PER_PULSE_CONVERSION_FACTOR));
  }

  /**
   * a method to force the arm down, this could burnout 
   * the motor if limit siwtch fails, so it must be used 
   * sparingly. This is used to compensate for belt 
   * stretching.
   */
  public void extendoArmRightForceIn(){
    extendoMotorRight.set(TalonFXControlMode.PercentOutput, 
      Constants.CLIMBER_EXTENDO_FORCE_IN);
  }

  /**
   * Intendend for testing and homing, drives arm in at low power
   */
  public void extendoArmRightIn(){
    extendoMotorRight.set(TalonFXControlMode.PercentOutput, 
    Constants.CLIMBER_EXTENDO_SPEED_IN);
  }

  /**
   * Intendend for testing, drives arm out at low power
   */
  public void extendoArmRightOut(){
    extendoMotorRight.set(TalonFXControlMode.PercentOutput, 
    Constants.CLIMBER_EXTENDO_SPEED_OUT);
  }

  public void stopExtendoRightArm() {
    extendoMotorRight.set(TalonFXControlMode.PercentOutput, 0);
  }

  /**
   *polls the supply current of the right motor 
   * @return current in Amps
   */
  public double getExtendoRightCurrent() {
    return extendoMotorRight.getSupplyCurrent();
  }

  /**
   * accessor for the right limit switch
   * @return true means it is pressed
   */
  public boolean getExtendoRightSwitch() {
    return rightBottomSwitch.isRevLimitSwitchClosed()==1;
  }

  /**
   * used to enable and disable the rear limit switch
   * @param enable true for limit switch on
   */
  public void setRightSwitchEnabled(boolean enable){
    if(enable){
      extendoMotorRight.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    }else{
      extendoMotorRight.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyOpen);
    }
  }
  
  /**
   * accessor for the right climber's  encoder
   * @return distance up in inches
   */
  public double getExtendoRightEncPos() {
    return extendoMotorRight.getSelectedSensorPosition() * Constants.EXTENDO_INCHES_PER_PULSE_CONVERSION_FACTOR;
  }

  public double getExtendoRightEncVel() {
    return extendoMotorRight.getSelectedSensorVelocity() * Constants.EXTENDO_INCHES_PER_PULSE_CONVERSION_FACTOR;
  }

  public void setExtendoRightEnc(double sensorPos) {
    extendoMotorRight.setSelectedSensorPosition(sensorPos / Constants.EXTENDO_INCHES_PER_PULSE_CONVERSION_FACTOR);
  }

  /* ==================== Functions for the left extendo arm ==================== */
  
  public void extendoLeftSetPos(double pos) {
    extendoMotorLeft.set(TalonFXControlMode.Position, pos / Constants.EXTENDO_INCHES_PER_PULSE_CONVERSION_FACTOR);
  }

/**
   * a method to force the arm down, this could burnout 
   * the motor if limit siwtch fails, so it must be used 
   * sparingly. This is used to compensate for belt 
   * stretching.
   */
  public void extendoArmLeftForceIn(){
    extendoMotorLeft.set(TalonFXControlMode.PercentOutput, 
      Constants.CLIMBER_EXTENDO_FORCE_IN);
  }

  /**
   * Intendend for testing and homing, drives arm in at low power
   */
  public void extendoArmLeftIn(){
    extendoMotorLeft.set(TalonFXControlMode.PercentOutput, 
      Constants.CLIMBER_EXTENDO_SPEED_IN);
  }
  
  /**
   * Intendend for testing, drives arm out at low power
   */
  public void extendoArmLeftOut(){
    extendoMotorLeft.set(TalonFXControlMode.PercentOutput, 
      Constants.CLIMBER_EXTENDO_SPEED_OUT);
  }

  public void stopExtendoLeftArm() {
    extendoMotorLeft.set(TalonFXControlMode.PercentOutput, 0);
  }

  /**
   *polls the supply current of the left motor 
   * @return current in Amps
   */
  public double getExtendoLeftCurrent() {
    return extendoMotorLeft.getSupplyCurrent();
  }

  /**
   * accessor for the right limit switch
   * @return true means it is pressed
   */
  public boolean getExtendoLeftSwitch() {
    return leftBottomSwitch.isRevLimitSwitchClosed()==1;
  }
  
  /**
   * used to enable and disable the rear limit switch
   * @param enable true for limit switch on
   */
  public void setLeftSwitchEnabled(boolean enable){
    if(enable){
      extendoMotorLeft.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    }else{
      extendoMotorLeft.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyOpen);
    }
  }

  /**
   * accessor for the left climber's  encoder
   * @return distance up in inches
   */
  public double getExtendoLeftEncPos() {
    return extendoMotorLeft.getSelectedSensorPosition() * Constants.EXTENDO_INCHES_PER_PULSE_CONVERSION_FACTOR;
  }

  public double getExtendoLeftEncVel() {
    return extendoMotorLeft.getSelectedSensorVelocity() * Constants.EXTENDO_INCHES_PER_PULSE_CONVERSION_FACTOR;
  }

  public void setExtendoLeftEnc(double sensorPos) {
    extendoMotorLeft.setSelectedSensorPosition(sensorPos / Constants.EXTENDO_INCHES_PER_PULSE_CONVERSION_FACTOR);
  }

  public boolean getExtendoBothSwitch(){
    return getExtendoLeftSwitch() && getExtendoRightSwitch();
  }

}
