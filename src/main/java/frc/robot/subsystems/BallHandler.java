// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class BallHandler extends SubsystemBase {

  DoubleSolenoid harvesterTilt;
  VictorSPX harvesterMotor; 
  CANSparkMax handlerMotors[];
  // RelativeEncoder handleEncoders[];//not using handler encoder

  DigitalInput ballSensor0;//switch for detecting Ball0, the one near the intake
  DigitalInput ballSensor1;//switch for detecting Ball1, the one near the shooter
  ColorSensorV3 colorSensor;//sensor for detecting ball color

  Timer selectorTimer = new Timer();//timer for reversing the Ball0 motor, for kicking out wrong balls
  Timer harvesterOutTimer = new Timer();//timer to delay moving Ball0 motor when harvestter is going down

  /**
   * the states for the BallHandler state machine
   */
  public enum State {
   kOff, kFillTo1, kFillTo0, kShoot1, kShoot0, kSpitLow1, kSpitMid1, kSpitHigh, kSpitMid0, kSpitLow0
  }
  
  double[] speeds = new double[4];//speeds to set the motors to
  double[] currentSpeeds = new double[] { 0.0, 0.0, 0.0, 0.0};//speeds stored from previous loop, to compare to current ones

  private State state = State.kOff;//storage for the current state of the state machine
  private State prevState = State.kOff;//storing the previous state for determining if state has changed

  private boolean rejectOppColor = true;//storage for whether we are runing SBM
  private boolean hasHarvesterBeenOut;//a check to see if the harvester has been out, used to avoid handlerMotors[0] burn out

  //pull preset speeds of the motors from constants
  private static final double HARV_IN = Constants.HARVESTER_INTAKE_SPEED;
  private static final double HARV_OUT = Constants.HARVESTER_EXTAKE_SPEED;
  private static final double BALL0_IN = Constants.BALL_HANDLER_0_INTAKE_SPEED;
  private static final double BALL0_OUT = Constants.BALL_HANDLER_0_EXTAKE_SPEED;
  private static final double BALL0_SHOOT = Constants.BALL_HANDLER_0_SHOOT_SPEED;
  private static final double BALL1_IN = Constants.BALL_HANDLER_1_INTAKE_SPEED;
  private static final double BALL1_OUT = Constants.BALL_HANDLER_1_EXTAKE_SPEED;
  private static final double BALL1_SHOOT = Constants.BALL_HANDLER_1_SHOOT_SPEED;
  private static final double BALL2_OUT = Constants.BALL_HANDLER_2_EXTAKE_SPEED;
  private static final double BALL2_SHOOT = Constants.BALL_HANDLER_2_SHOOT_SPEED;

  private static final double HARVESTER_OUT_DELAY = 1.0;
  private static final double SBM_KICKOUT_TIME = .15;
  
  /** Creates a new Intake. */
  public BallHandler() {
    harvesterTilt = new DoubleSolenoid(PneumaticsModuleType.REVPH, 
      Constants.HARVESTER_TILT_IN, Constants.HARVESTER_TILT_OUT);
    
    harvesterMotor = new VictorSPX(Constants.HARVESTER_MOTOR);
    harvesterMotor.setInverted(true);
    harvesterMotor.setNeutralMode(NeutralMode.Coast);
    harvesterMotor.configVoltageCompSaturation(10.5);
    
    harvesterMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
    harvesterMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 251);
    harvesterMotor.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 241);
    harvesterMotor.setStatusFramePeriod(StatusFrame.Status_6_Misc, 239);
    harvesterMotor.setStatusFramePeriod(StatusFrame.Status_7_CommStatus, 233);
    harvesterMotor.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 229);
    harvesterMotor.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 255);
    harvesterMotor.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 251);
    harvesterMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 241);
    harvesterMotor.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 239);
    harvesterMotor.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 233);
    harvesterMotor.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 229);

    handlerMotors = new CANSparkMax[]{
      new CANSparkMax(Constants.BALL_HANDLER_MOTOR_0, MotorType.kBrushless),//axleWheels,
      new CANSparkMax(Constants.BALL_HANDLER_MOTOR_1, MotorType.kBrushless),
      new CANSparkMax(Constants.BALL_HANDLER_MOTOR_2, MotorType.kBrushless)
    };

    for (int i = 0; i <= 2; i++) {
      handlerMotors[i].restoreFactoryDefaults();
      handlerMotors[i].setIdleMode(IdleMode.kBrake);// set brake mode, so motors stop on a dime
      handlerMotors[i].enableVoltageCompensation(10.50);// enable volatge compensation mode
      handlerMotors[i].setInverted(i == 1);// only the second NEO550 in the ballHandler needs to be inverted.
      handlerMotors[i].setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);//applied output, faults, sticky faults //TODO: can this be higher???
      handlerMotors[i].setPeriodicFramePeriod(PeriodicFrame.kStatus1, 60000);//Velocity, Temperature, Voltage, Current
      handlerMotors[i].setPeriodicFramePeriod(PeriodicFrame.kStatus2, 60000);//Position(not used at all)
      handlerMotors[i].setPeriodicFramePeriod(PeriodicFrame.kStatus3, 60000);//analog sensor(not used at all)
      
      // handleEncoders[i] = handlerMotors[i].getEncoder();//We don't use encoder, but this was also causing the code to crash

      handlerMotors[i].burnFlash();//this saves settings, BUT MUST BE DONE LAST, SparkMAX won't accept commands for a moment after this call
    }

    ballSensor0 = new DigitalInput(Constants.BALL_SENSOR_0);
    ballSensor1 = new DigitalInput(Constants.BALL_SENSOR_1);
    
    colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

    selectorTimer.reset();
    selectorTimer.start();

    harvesterOutTimer.reset();
    harvesterOutTimer.start();

    rejectOppColor = true;
    hasHarvesterBeenOut = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //BallHandler State Machine
    switch(state){
      case kShoot1:
        //shooting just the first ball
        speeds = new double[] { 0, 0, BALL1_SHOOT, BALL2_SHOOT };
        break;
      case kShoot0:
        //shooting both balls
        speeds = new double[] { 0, BALL0_SHOOT, BALL1_SHOOT, BALL2_SHOOT };
        break;
      case kSpitHigh:
        //spitting out the balls, with harvester down and intaking
        speeds = new double[] { HARV_IN, BALL0_OUT, BALL1_OUT, BALL2_OUT};
        break;
      case kSpitMid0:
        //spitting balls, with harvester up and off
        speeds = new double[] { 0, BALL0_OUT, 0, 0};
        //check if harvester has ever been out
        if(!hasHarvesterBeenOut()){
          //if harvester hasn't been out switch to the kSpitLow0 to avoid buring out handlerMotors[0]
          state = State.kSpitLow0;
        }else{
          break;
        }
      case kSpitLow0:
        //spitting balls, with harvester down and running out
        speeds = new double[] { HARV_OUT, BALL0_OUT, 0, 0 };
        break;
      case kSpitMid1:
        //spitting balls, with harvester up and off
        speeds = new double[] { 0, BALL0_OUT, BALL1_OUT, BALL2_OUT};
        //check if harvester has ever been out
        if(!hasHarvesterBeenOut()){
          //if harvester hasn't been out switch to the kSpitLow1 to avoid buring out handlerMotors[0]
          state = State.kSpitLow1;
        }else{
          break;
        }
      case kSpitLow1:
        //spitting balls, with harvester down and running out
        speeds = new double[] { HARV_OUT, BALL0_OUT, BALL1_OUT, BALL2_OUT };
        break;
      case kFillTo1:
        //filling the robot until the Ball1 sensor is pressed
        speeds = new double[] { HARV_IN, BALL0_IN, BALL1_IN, 0 };
        if(isBall1()){
          //if we see a Ball change state and fall to next case
          state = State.kFillTo0;
        }
        else{
          break;
        }
      case kFillTo0:
        //filling the robot until the Ball0 sensor is pressed
        speeds = new double[] { HARV_IN, BALL0_IN, 0, 0 };
        if(isBall0() && selectorTimer.hasElapsed(SBM_KICKOUT_TIME)){
          //if we see a Ball change state and fall to next case
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

    //testing the SBM, this is data intensive, enable only for testing
    // if((state == State.kFillTo1 || state == State.kFillTo0)){
    //   //Smartdashboard pushes for testing
      // SmartDashboard.putNumber("Red Color", colorSensor.getRed());
      // SmartDashboard.putNumber("Blue Color", colorSensor.getBlue());
      // SmartDashboard.putNumber("Red - Blue", colorSensor.getRed() - colorSensor.getBlue());
      // SmartDashboard.putNumber("Blue - Red", colorSensor.getBlue() - colorSensor.getRed());
      // SmartDashboard.putBoolean("SelectorTime", !selectorTimer.hasElapsed(.5));
      // SmartDashboard.putNumber("Proximity", colorSensor.getProximity());
    // }

    //if state has changed, check to move harvester in or out
    if(state != prevState){
      if(state == State.kFillTo1 || state == State.kFillTo0) {
        if(!hasHarvesterBeenOut()){
          //if the harvester has never been out, reset the timer(which pauses the floppy arm motor)
          harvesterOutTimer.reset();
        }
        harvesterOut();
      }else if(state == State.kSpitHigh || state == State.kSpitLow1 || state== State.kSpitLow0){
        if(!hasHarvesterBeenOut()){
          //if the harvester has never been out, reset the timer(which pauses the floppy arm motor)
          harvesterOutTimer.reset();
        }
        harvesterOut();
      }else if(state == State.kOff){
        //if the state is off, pull in the harvester
        harvesterIn();
      }else if(state == State.kShoot0){
        if(!hasHarvesterBeenOut()){
          //if the harvester has never been out, drop the harvester and reset the timer(which pauses the floppy arm motor)
          harvesterOutTimer.reset();
          harvesterOut();
        }
      }
    }

    //If the wrong ball color is detected, reset spitout timer
    if((state == State.kFillTo1 || state == State.kFillTo0) && !shouldIntakeBall()){
      // System.out.println("Reset selector timer");
      selectorTimer.reset();
    }

    if( !harvesterOutTimer.hasElapsed(HARVESTER_OUT_DELAY) ){
      //if we think the floppy roller is in, pause the handlerMotors[0] for a minute
      speeds[1] = 0.0;
    }else if( !selectorTimer.hasElapsed(SBM_KICKOUT_TIME) ){
      //if timer reset, run spit out timer for half second
      speeds[1] *= -1;  
    }

    //to avoid too much CAN uses, only change values when speeds change.
    if(!DriverStation.isDisabled() && 
      !Arrays.equals(speeds,currentSpeeds)){
      //If speed have changed, update the motor output speeds
      harvesterMotor.set(VictorSPXControlMode.PercentOutput, speeds[0]);
      for (int i = 1; i <= 3; i++) {
          handlerMotors[i-1].set(speeds[i]);
      }
      //store the speeds sent to the motors as current speeds
      currentSpeeds = speeds;
    }
    // System.out.println("speeds"+speeds[0]+":"+speeds[1]+":"+speeds[2]+":"+speeds[3]+":");

    SmartDashboard.putBoolean("ball0", isBall0());
    SmartDashboard.putBoolean("ball1", isBall1());

    //store current state as prevState for next loop
    prevState = state;
  }
  
  /**
   * sets harvester pneumatic in. this is the place 
   * the harvester should be when starting the 
   * match, where it is safely in the robot.
   */
  public void harvesterIn(){
    harvesterTilt.set(Value.kForward);
  }

  /**
   * sets harvester pneumatic out. Out is where the 
   * harvester must be in order to intake balls.
   */
  public void harvesterOut(){
    harvesterTilt.set(Value.kReverse);
    hasHarvesterBeenOut = true;
  }

  /**
   * reset the hasHarvesterBeenOut boolean to false
   * this shoould be done in disabled, as the 
   * hasHarvesterBeenOut boolean is key to stopping 
   * the handlerMotors[0] from burning out because 
   * it is caught in the upright position while 
   * being held in by the harvester.
   */
  public void resetHasHarvesterBeenOut(){
    hasHarvesterBeenOut = false;
  }

  /**
   * accesor foor the hasHarvesterBeenOut boolean, 
   * which is used as a test to see if the floppy 
   * roller, which is connected to the 
   * handlerMorors[0], is stuck up by the harvester 
   * which is part of starting confingureation.
   * 
   * @return true if harvesterOut() has been called since last disable (that has lasted more than 5 seconds)
   */
  public boolean hasHarvesterBeenOut(){
    return hasHarvesterBeenOut;
  }

  /**
   * An accessor that sees whether the harvester is 
   * currently set to out. This is function is not 
   * used given we use hasHarvesterBeenOut and 
   * changed logic accordingly.
   * 
   * @return false if we know harvester is out
   */
  public boolean isHarvesterNotOut(){
    return !(harvesterTilt.get() == Value.kReverse);
  }

  //commented out because haverster is handled in periodic state machine
  // public void intake(){
  //   harvesterMotor.set(TalonSRXControlMode.PercentOutput, Constants.HARVESTER_INTAKE_SPEED);
  // }

  // public void extake(){
  //   harvesterMotor.set(TalonSRXControlMode.PercentOutput, Constants.HARVESTER_EXTAKE_SPEED);
  // }

  // public void stopHarvester(){
  //   harvesterMotor.set(TalonSRXControlMode.PercentOutput, 0);
  // }

  /**
   * checks the first ball switch
   * the one close to the intake
   * 
   * @return true if ball is present
   */
  public boolean isBall0(){
    return !ballSensor0.get();
  }
  
  /**
   * checks the second ball switch,
   * the one close to the shooter
   * 
   * @return true if ball is present
   */
  public boolean isBall1(){
    return !ballSensor1.get();
  }

  public boolean shouldIntakeBall(){

    //check if sensor is connected(fault could cause not connected), or not rejecting wrong color
    if(!colorSensor.isConnected() || !rejectOppColor ){
      //return true in fault condition, best to just intake on failure
      return true;
    }

    //poll sensor once for the needed values
    int proximity = colorSensor.getProximity();
    int blue = colorSensor.getBlue();
    int red = colorSensor.getRed();

    //in connected but not working situation, reconstruct and return.
    if((proximity == 0 && blue == 0 && red == 0)){
      colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
      return true;
    }
    
    //Checking Alliance then test if the ball is incorrect
    if(DriverStation.getAlliance() == Alliance.Blue){
      // System.out.print( red-blue + "      ");
      // System.out.print((red - blue >= Constants.BLUE_LEVEL));
      // System.out.println("blue Aliance!!!!");
      return blue - red >= Constants.BLUE_LEVEL;
    }else if(DriverStation.getAlliance() == Alliance.Red){
      // System.out.println(red - blue);
      // System.out.println(blue - red >= Constants.BLUE_LEVEL);
      // System.out.println("RED Alliance.");
      return red - blue >= Constants.RED_LEVEL;
    }
    else{
      return true;
    }
  
  }
  
  /**
   * Set the state of the ballhandler. The ballhandler 
   * is managed by the periodic function in the 
   * ballhandler. This is used to change the state of 
   * the ballhandler
   * 
   * @param state uses enum State(BallHandler.State)
   */
  public void setState(State state){
    this.state = state;
  }

  /**
   * returns the current state of the ballhandler.
   * 
   * @return enum State (BallHandler.State)
   */
  public State getState(){
    return state;
  }
  
  /**
   * used to turn on or off the opponant ball 
   * rejection system. SBM
   * 
   * @param reject set true to turn on rejection
   */
  public void rejectOppColor(boolean reject){
    this.rejectOppColor = reject;
  }
}
