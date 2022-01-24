// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Climber extends SubsystemBase {
  DoubleSolenoid tiltRobot;
  TalonFX fixedMotor;
  TalonFX extendoMotor;

  /** Creates a new Climber. */
  public Climber() {
    tiltRobot = new DoubleSolenoid(PneumaticsModuleType.REVPH, 
      Constants.CLIMBER_TILT_IN, Constants.CLIMBER_TILT_OUT);

    fixedMotor = new TalonFX(Constants.CLIMBER_FIXED_ARM);
    fixedMotor.configFactoryDefault();
    // use the integrated sensor with the primary closed loop and timeout is 0.
    fixedMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    fixedMotor.configSelectedFeedbackCoefficient(1);//TODO: based on gearing, find out
    fixedMotor.setNeutralMode(NeutralMode.Brake);
    fixedMotor.setInverted(false);// Set motor inverted(set to false) TODO:Is this right?
    fixedMotor.enableVoltageCompensation(true);
    fixedMotor.configVoltageCompSaturation(Constants.MAXIMUM_VOLTAGE);
    fixedMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10);
    fixedMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);
    fixedMotor.setSelectedSensorPosition(0.0);

    extendoMotor = new TalonFX(Constants.CLIMBER_EXTENDO_ARM);
    extendoMotor.configFactoryDefault();
    // use the integrated sensor with the primary closed loop and timeout is 0.
    extendoMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    extendoMotor.configSelectedFeedbackCoefficient(1);//TODO: based on gearing,find out
    extendoMotor.setNeutralMode(NeutralMode.Brake);
    extendoMotor.setInverted(false);// Set motor inverted(set to false) TODO:Is this right?
    extendoMotor.enableVoltageCompensation(true);
    extendoMotor.configVoltageCompSaturation(Constants.MAXIMUM_VOLTAGE);
    extendoMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10);
    extendoMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);
    extendoMotor.setSelectedSensorPosition(0.0);
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
  public void extendoArmOut(){
    extendoMotor.set(TalonFXControlMode.PercentOutput, 
    Constants.CLIMBER_EXTENDO_SPEED_OUT);;
  }

  public void extendoArmIn(){
    extendoMotor.set(TalonFXControlMode.PercentOutput, 
    Constants.CLIMBER_EXTENDO_SPEED_IN);
  }

  //TODO: Make a method to move the fixedArm to a speciffic distance(Use built-in MotionProfile?), def use postitioncontrol
  public void fixedArmOut(){
    fixedMotor.set(TalonFXControlMode.PercentOutput, 
    Constants.CLIMBER_FIXED_SPEED_OUT);
  }

  public void fixedArmIn(){
    fixedMotor.set(TalonFXControlMode.PercentOutput, 
    Constants.CLIMBER_FIXED_SPEED_IN);
  }


}
