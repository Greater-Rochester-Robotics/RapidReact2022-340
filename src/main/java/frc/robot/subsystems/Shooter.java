// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
  TalonFX followMotor;//TODO: remove this motor, we hve a one wheel shooter
  CANSparkMax hoodMotor;
  //TODO: instantiate spark motor encoder object
  //TODO: instantiate sparkmax Pidcontroller
  //TODO: instantiate DigitalInput for hoodLimit switch

  /** Creates a new Shooter. */
  public Shooter() {
    mainMotor = new TalonFX(Constants.MAIN_SHOOTER_MOTOR);
    mainMotor.configFactoryDefault();
    // mainMotor.configSelectedFeedbackCoefficient(coefficient);//TODO: set to an RPM based on the gear ratio
    mainMotor.setNeutralMode(NeutralMode.Coast);
    mainMotor.setInverted(false);//TODO: check if, on robot, forward is out.
    mainMotor.enableVoltageCompensation(true);
    mainMotor.configVoltageCompSaturation(Constants.MAXIMUM_VOLTAGE);
    mainMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10);
    mainMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);
    //TODO: set up PID for the shooter, use config_kP through config_kF (P,I,D, F) put constants in Constants place in slot0
    //TODO: configAllowableCloseLoopError, put constant in Constants, again slot0

    followMotor = new TalonFX(Constants.FOLLOW_SHOOTER_MOTOR);
    hoodMotor = new CANSparkMax(Constants.SHOOTER_HOOD_MOTOR, MotorType.kBrushless);
    //TODO: write configure the hoodMotor. restoreDefaults, idle to brake, voltage comp on and at 10.5, set inverted(not sure T or F) but write it 
    //TODO: pull encoder from motor object, set position convertion factor on encoder (USE SHOOTER_HOOD_DEGREE_CONVERSION)
    //TODO: pull pidController from sparkmax motor object, then set P, I, D, FF. methods are setP() etc put constants in Constants 
    //TODO: burn flash of sparkmax

    //TODO: construct hoodLimit switch
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getSpeed(){
    return mainMotor.getSelectedSensorVelocity();
  }

  public void setSpeed(double speed){
    mainMotor.set(TalonFXControlMode.Velocity, speed);
  }
  
  /**
   * @return if it is at speed within tolerance
   */
  public boolean isAtSpeed(){
    //TalonFX has an isAtSpeed() with tolerance already
    return ((goalSpeed * (1.00 - speedError) <= getSpeed()) && 
            (goalSpeed * (1.00 + speedError) >= getSpeed()));
  }

  public void stopMotors(){
    mainMotor.set(TalonFXControlMode.PercentOutput, 0.0);
  }

  //TODO: create a setHoodPosition for the hoodMotor. use the pidController object to set in method

  //TODO: create accessor method for the position of the hood. use the sparkmax encoderObject as source

  //TODO: create reset position modifier method that will set the position of the encoder object. this will act as our reset device, use setPosion of encoder object
  
  //TODO: create a homeMethod that returns a true when limit switch is pressed, and drives motor backwards with percentVoltage.(this would be using hoodMotor.set) use the previous resetPosition
}
