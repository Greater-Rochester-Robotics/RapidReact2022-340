// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Hood extends SubsystemBase {
  CANSparkMax motor;
  RelativeEncoder encoder;
  SparkMaxPIDController pidController;
  DigitalInput limitSwitch;
  boolean hasBeenHomed;

  /** Creates a new Hood. */
  public Hood() {
    motor = new CANSparkMax(Constants.SHOOTER_HOOD_MOTOR, MotorType.kBrushless);
    motor.restoreFactoryDefaults();//begin by reseting the controller to default
    motor.setIdleMode(IdleMode.kBrake);//need brake to keep hood in place
    motor.enableVoltageCompensation(10.5);//scale power with voltage drop
    motor.setInverted(false); //this is the right direction 2022

    encoder = motor.getEncoder();//pull encoder to separate object for brevity
    encoder.setPositionConversionFactor(Constants.HOOD_DEGREE_CONVERSION);//convert to degrees

    pidController = motor.getPIDController();
    pidController.setP(Constants.HOOD_MOTOR_P);
    pidController.setI(Constants.HOOD_MOTOR_I);
    pidController.setD(Constants.HOOD_MOTOR_D);
    pidController.setFF(Constants.HOOD_MOTOR_FF);//changed gearbox, no FF needed

    motor.setSoftLimit(SoftLimitDirection.kForward, (float) (Constants.HOOD_FORWARD_LIMIT_DEGREES/Constants.HOOD_DEGREE_CONVERSION));
    motor.enableSoftLimit(SoftLimitDirection.kForward, true);//This doesn't work right, added code in set position
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 62000);//reduce the rate of this packet, basic stuff //TODO: can this be higher
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 61000);//reduce the rate of this packet, which is the encoder speed
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);//Position
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 60000);//analog sensor(not used at all)

    motor.clearFaults();//clear motor fault so we can check when rebooted(check when rebotted doesn't work)

    motor.burnFlash();//make the previous settings hold after power cycle, must be last
    
    limitSwitch = new DigitalInput(Constants.SHOOTER_HOOD_SWITCH);

    hasBeenHomed = false;//on first boot, the hood has not been homed
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("HoodLimit", !limitSwitch.get());
    
    //If the hood is running a command...
    if(!(getCurrentCommand() == null)) {
      //pass the current hood position to the dashboard
      SmartDashboard.putNumber("Hood Position", getPosition());
    }
    // SmartDashboard.putBoolean("HoodHasPowerCycled", motor.getFault(FaultID.kHasReset));//this doesn't work, left for later testing

  }

  /**
   * accessor for whether to determine if the hood 
   * has been homed.
   * 
   * @return true if hood is homed
   */
  public boolean hasBeenHomed(){
    return hasBeenHomed;
  }

  /**
   * sets the hasBeenHomed variable to the input. 
   * @param hasBeenHomed boolean
   */
  public void setBeenHomed(boolean hasBeenHomed){
    this.hasBeenHomed = hasBeenHomed;
  }

  /**
   * A method to check if the motor controller has 
   * power cycled, if so the hasBeenHomed is reset.
   * 
   * !currently this doesn't work!
   */
  @Deprecated
  public void checkForPowerCycle(){
    if(motor.getFault(FaultID.kHasReset)){
      setBeenHomed(false);
      motor.clearFaults();
    }

  }

  /**
   * A method to use the pidController to set the hood to an angle
   */
  public void setPosition(double position) {
    if(position > Constants.HOOD_FORWARD_LIMIT_DEGREES){
      //if we set beyond the forward limit, change position to forward limit
      position = Constants.HOOD_FORWARD_LIMIT_DEGREES;
    }
    //set pidController
    pidController.setReference(position, ControlType.kPosition);
  }

  /**
   * Accessor method for the position of the hood. 
   * 
   * @return in degrees from llimit switch
   */
  public double getPosition() {
    return encoder.getPosition();
  }

  /**
   * Resets position of the encoder object to zero. 
   * this will act as our reset device
   */
  public void resetEncoderPosition(){
    resetEncoderPosition(0);
  }
  
  /**
   * Set the current position of the hood to the 
   * input position.
   * 
   * @param position
   */
  public void resetEncoderPosition(double position) {
    encoder.setPosition(position);
  }

  /**
   * A homeMethod that returns a true when limit switch is 
   * pressed, and drives motor backwards with percentVoltage.
   * It resets the hood encoder with resetHoodEncoderPosition
   * when the switch is pressed.
   * 
   * @return true when done, false when not done
   */
  public boolean homePosition() {
    
    if(!limitSwitch.get()) {
      resetEncoderPosition();
      setBeenHomed(true);
      motor.set(0.0);
      return true;
    }else{
      motor.set(Constants.HOOD_HOMING_SPEED);
      return false;
    }
  }

  /**
   * Stops the hood motor
   */
  public void stopMotor(){
    motor.set(0.0);
  }
}
