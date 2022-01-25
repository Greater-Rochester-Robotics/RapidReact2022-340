// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlMode;
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
    BufferedTrajectoryPointStream stream = new BufferedTrajectoryPointStream(); // Creates a set of points telling the motor how to move
    TrajectoryPoint middleTrajectoryPoint = new TrajectoryPoint(); // Creates single point telling the motor how to move
    TrajectoryPoint lastTrajectoryPoint = new TrajectoryPoint();

    //TODO: Might need to make a start position

    // The following sets our middle position and speed
    middleTrajectoryPoint.position = 10;
    middleTrajectoryPoint.velocity = Constants.CLIMBER_EXTENDO_SPEED_OUT;
    stream.Write(middleTrajectoryPoint); // Add trajectory point to stream
    // The following sets our last position and speed
    lastTrajectoryPoint.position = 20;
    lastTrajectoryPoint.velocity = 0; // Must be set to 0 or motor will spin indefinitely
    stream.Write(lastTrajectoryPoint);

    // Creates a motion profile using set of points in stream and the control mode
    extendoMotor.startMotionProfile(stream, extendoMotor.getMotionProfileTopLevelBufferCount(), ControlMode.Position);
    }

  public void extendoArmIn(){
    extendoMotor.set(TalonFXControlMode.PercentOutput, 
    Constants.CLIMBER_EXTENDO_SPEED_IN);
  }

  //TODO: Make match extendoArmOut()
  public void fixedArmOut(){
    BufferedTrajectoryPointStream stream = new BufferedTrajectoryPointStream();
    TrajectoryPoint trajectoryPoint = new TrajectoryPoint();

    trajectoryPoint.position = 10;
    trajectoryPoint.velocity = Constants.CLIMBER_FIXED_SPEED_OUT;
    stream.Write(trajectoryPoint);
    
    int minBufferedPts = fixedMotor.getMotionProfileTopLevelBufferCount();
    fixedMotor.startMotionProfile(stream, minBufferedPts, ControlMode.Position);
  }

  public void fixedArmIn(){
    fixedMotor.set(TalonFXControlMode.PercentOutput, 
    Constants.CLIMBER_FIXED_SPEED_IN);
  }


}
