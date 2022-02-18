// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class BallHandler extends SubsystemBase {

  DoubleSolenoid harvesterTilt;
  VictorSPX harvesterMotor; 
  CANSparkMax handlerMotors[];
  RelativeEncoder handleEncoders[];
  double[] speeds = new double[4];
  double[] currentSpeeds = new double[] { 0.0, 0.0, 0.0, 0.0};
  DigitalInput ballSensor0;
  DigitalInput ballSensor1;
  ColorSensorV3 colorSensor; 
  Timer selectorTimer = new Timer();

  public enum State {
   kOff, kFillTo1, kFillTo0, kShoot1, kShoot0, kSpitLow, kSpitMid, kSpitHigh
  }

  private State state = State.kOff;
  private State prevState = State.kOff;

  private static final double HARV_IN = Constants.HARVESTER_INTAKE_SPEED;
  private static final double HARV_OUT = Constants.HARVESTER_EXTAKE_SPEED;
  private static final double BALL0_IN = Constants.BALL_HANDLER_0_INTAKE_SPEED;
  private static final double BALL0_OUT = Constants.BALL_HANDLER_0_EXTAKE_SPEED;
  private static final double BALL1_IN = Constants.BALL_HANDLER_1_INTAKE_SPEED;
  private static final double BALL1_OUT = Constants.BALL_HANDLER_1_EXTAKE_SPEED;
  private static final double BALL2_OUT = Constants.BALL_HANDLER_2_EXTAKE_SPEED;
  private static final double BALL2_SHOOT = Constants.BALL_HANDLER_2_SHOOT_SPEED;
  private static final double BALL1_SHOOT = Constants.BALL_HANDLER_1_SHOOT_SPEED;
  private static final double BALL0_SHOOT = Constants.BALL_HANDLER_0_SHOOT_SPEED;
    
  
  /** Creates a new Intake. */
  public BallHandler() {
    harvesterTilt = new DoubleSolenoid(PneumaticsModuleType.REVPH, 
      Constants.HARVESTER_TILT_IN, Constants.HARVESTER_TILT_OUT);
    harvesterMotor = new VictorSPX(Constants.HARVESTER_MOTOR);
    harvesterMotor.setInverted(false);
    harvesterMotor.setNeutralMode(NeutralMode.Coast);
    harvesterMotor.configVoltageCompSaturation(10.5);

    handlerMotors = new CANSparkMax[]{
      new CANSparkMax(Constants.BALL_HANDLER_MOTOR_0, MotorType.kBrushless),//axleWheels,
      new CANSparkMax(Constants.BALL_HANDLER_MOTOR_1, MotorType.kBrushless),
      new CANSparkMax(Constants.BALL_HANDLER_MOTOR_2, MotorType.kBrushless)
    };

    for (int i = 0; i <= 2; i++) {
      handlerMotors[i].restoreFactoryDefaults();
      handlerMotors[i].setIdleMode(IdleMode.kBrake);// set brake mode, so motors stop on a dime
      handlerMotors[i].enableVoltageCompensation(10.50);// enable volatge compensation mode
      handlerMotors[i].setInverted(i == 1);// only the second NEO in the ballHandler needs to be inverted.

      // handleEncoders[i] = handlerMotors[i].getEncoder();//TODO: figure out if we need to get the encoder. also why did it cause the code to crash

      handlerMotors[i].burnFlash();//this saves settings, BUT MUST BE DONE LAST, SparkMAX won't accept commands for a moment after this call
    }
    ballSensor0 = new DigitalInput(Constants.BALL_SENSOR_0);
    ballSensor1 = new DigitalInput(Constants.BALL_SENSOR_1);
    
    colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

    selectorTimer.reset();
    selectorTimer.start();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Red Color", colorSensor.getRed());
    SmartDashboard.putNumber("Blue Color", colorSensor.getBlue());
    SmartDashboard.putNumber("Proximity", colorSensor.getProximity());
    SmartDashboard.putBoolean("connected",colorSensor.isConnected());

    //BallHandler State Machine
    switch(state){
      case kShoot1:
        speeds = new double[] { 0, 0, BALL1_SHOOT, BALL2_SHOOT };
        break;
      case kShoot0:
        speeds = new double[] { 0, BALL0_SHOOT, BALL1_SHOOT, BALL2_SHOOT };
        break;
      case kSpitHigh:
        speeds = new double[] { HARV_IN, BALL0_OUT, BALL1_OUT, BALL2_OUT};
        break;
      case kSpitMid:
        speeds = new double[] { 0, BALL0_OUT, BALL1_OUT, BALL2_OUT};
        break;
      case kSpitLow:
        speeds = new double[] { HARV_OUT, BALL0_OUT, BALL1_OUT, BALL2_OUT };
        break;
      case kFillTo1:
        speeds = new double[] { HARV_IN, BALL0_IN, BALL1_IN, 0 };
        if(isBall1()){
          state = State.kFillTo0;
        }
        else{
          break;
        }
      case kFillTo0:
        speeds = new double[] { HARV_IN, BALL0_IN, 0, 0 };

        if(isBall0()){
          state = State.kOff;
        }
        else{
          break;
        }
      case kOff:
        speeds = new double[] { 0, 0, 0, 0 };
        break;
      default:
        speeds = new double[] { 0, 0, 0, 0 };
        System.out.println("default ball handler case reached");
    }

    //if state has changed, check to move harvester in or out
    if(state != prevState){
      if(state == State.kFillTo1 || state == State.kFillTo0){
        tiltOut();
      }else{
        tiltIn();
      }
    }

    //If speed have changed, update the motor output speeds
    if(!DriverStation.isDisabled() && 
      !Arrays.equals(speeds,currentSpeeds)){
      harvesterMotor.set(VictorSPXControlMode.PercentOutput, speeds[0]);
      for (int i = 1; i <= 3; i++) {
          handlerMotors[i-1].set(speeds[i]);
      }
      //store the speeds sent to the motors as current speeds
      currentSpeeds = speeds;
    }

    //If the wrong ball color is detected, reset spitout timer
    if((state == State.kFillTo1 || state == State.kFillTo0) && !shouldIntakeBall()){
      selectorTimer.reset();
    }

    //if timer reset, run spit out timer for half second
    if(!selectorTimer.hasElapsed(0.5)){
      handlerMotors[0].setInverted(true);
    }
    else{
      handlerMotors[0].setInverted(false);
    }

    //store current state as prevState for next loop
    prevState = state;
  }
  
  public void tiltIn(){
    harvesterTilt.set(Value.kReverse);
  }

  public void tiltOut(){
    harvesterTilt.set(Value.kForward);
  }

  // public void intake(){
  //   harvesterMotor.set(TalonSRXControlMode.PercentOutput, Constants.HARVESTER_INTAKE_SPEED);
  // }

  // public void extake(){
  //   harvesterMotor.set(TalonSRXControlMode.PercentOutput, Constants.HARVESTER_EXTAKE_SPEED);
  // }

  // public void stopHarvester(){
  //   harvesterMotor.set(TalonSRXControlMode.PercentOutput, 0);
  // }

  public boolean isBall0(){
    return ballSensor0.get();
  }
  
  public boolean isBall1(){
    return ballSensor1.get();
  }

  public boolean shouldIntakeBall(){
    int proximity = colorSensor.getProximity();
    
    int blue = colorSensor.getBlue();
    int red = colorSensor.getRed();

    if(!colorSensor.isConnected() || (proximity == 0 && blue == 0 && red == 0)) {
      return true;
    }
    //Checking proximity to intake balls
    if(proximity > Constants.INTAKE_PROXIMITY){
      if(DriverStation.getAlliance() == Alliance.Blue){
        return blue - red >= Constants.BLUE_LEVEL;
      }
      else if(DriverStation.getAlliance() == Alliance.Red){
        return red - blue >= Constants.RED_LEVEL;
      }
      else{
        return true;
      }
    }
    return true;
  }
  
  public void setState(State state){
    this.state = state;
  }

  public State getState(){
    return state;
  }
}
