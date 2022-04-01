// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ballhandler;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BallHandler.State;

public class BallHandlerShootProgT extends CommandBase {
  private Timer ballTimer = new Timer();
  private Timer isFinishedTimer = new Timer();
  private double timeBetweenBalls = .5;
  private int shootingBall = 1;
  private int ballsToShoot = 2;
  private boolean wasBall1Pressed = false;
  private boolean timeBypass = false;

  /** 
    * This command shoots all the balls, but 
  * starts with one ball and then starts moving 
  * each progessive ball forward delayed by 0.25 
  * seconds after that previous ball.
  * 
  * @requires SnekLoader, Shooter 
  */
  public BallHandlerShootProgT() { 
    this(0.25);
  }

  /** 
   * This command shoots all the balls, but 
   * starts with one ball and then starts moving 
   * each progessive ball forward delayed by 
   * timeBetweenBalls seconds after that previous ball. 
   * 
   * @param timeBetweenBalls time between when oone ball starts moving and the next
   * @requires SnekLoader, Shooter 
   */
  public BallHandlerShootProgT(double timeBetweenBalls) { 
    if(timeBetweenBalls < 0.0){
      timeBetweenBalls = 0.0;
    }
    this.timeBetweenBalls = timeBetweenBalls;
    addRequirements(RobotContainer.ballHandler);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //how many balls are we shooting?
    ballsToShoot = (RobotContainer.ballHandler.isBall0()?1:0) 
      + (RobotContainer.ballHandler.isBall1()?1:0); 

    if(!RobotContainer.ballHandler.hasHarvesterBeenOut()){
      timeBypass = true;
    }else{
      timeBypass = ballsToShoot == 0;
    }

    //check the state of the first sensor
    wasBall1Pressed = RobotContainer.ballHandler.isBall1();

    if(timeBetweenBalls < Robot.kDefaultPeriod){
      RobotContainer.ballHandler.setState(State.kShoot0);
    }else{
      //set the first ball to shoot
      RobotContainer.ballHandler.setState(State.kShoot1);
    }

    //start a timer to delay the launch of Ball0
    ballTimer.reset();
    ballTimer.start();
    
    //Reset the timer that allows balls time to leave the shooter
    isFinishedTimer.reset();
    isFinishedTimer.start();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //Check if a ball has left the top ball position
    if( !RobotContainer.ballHandler.isBall1() && wasBall1Pressed){
      //if the ball has left the top switch, it is one less ball to shoot
      ballsToShoot--;
    } 

    //Wait to start the Ball0 moving
    if(ballTimer.hasElapsed(timeBetweenBalls)){
      RobotContainer.ballHandler.setState(State.kShoot0);
    }

    //for as long as there aare balls in the shooter, keep resetting the timer
    if(ballsToShoot > 0 || (!RobotContainer.ballHandler.isBall1() && wasBall1Pressed) ){
      isFinishedTimer.reset();
    }
    wasBall1Pressed = RobotContainer.ballHandler.isBall1();
    // System.out.println("BallShoot Timer:"+isFinishedTimer.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.ballHandler.setState(State.kOff);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinishedTimer.hasElapsed(timeBypass?5:.5);//TODO: find out how long it takes a ball to leave the shooter from the top position 
  }
}
