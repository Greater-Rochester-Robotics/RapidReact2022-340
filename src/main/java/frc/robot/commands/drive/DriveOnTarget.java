// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController.Axis;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class DriveOnTarget extends CommandBase {
  Timer startTimer = new Timer();
  Timer onTargetTimer = new Timer();
  boolean hasHadTarget;
  double setPointAngle;
  double offsetDistance;

  public DriveOnTarget(){
    this(0.0);
  }

  public DriveOnTarget(double offsetDistance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.offsetDistance = offsetDistance;
    addRequirements(RobotContainer.swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.limeLight.setLightState(true, RobotContainer.swerveDrive);
    //reset start timer for the Pose2d guess where robot angle is
    startTimer.start();
    startTimer.reset();
    //reset onTargetTimer
    onTargetTimer.start();
    onTargetTimer.reset();
    hasHadTarget = false;
    // Default to the current angle of the robot
    setPointAngle = RobotContainer.swerveDrive.getGyroInRad();    
  }

  // Called every time the scheduler runs while the command is scheduled.
  
  @Override
  public void execute() {
    boolean hasTarget = RobotContainer.limeLight.hasTarget();
    hasHadTarget |= hasTarget;

    
    // Set the rotate to angle to the target if limelight sees it
    if(hasTarget) {
      setPointAngle = RobotContainer.swerveDrive.getGyroInRad() - Math.toRadians(RobotContainer.limeLight.angleToTarget(offsetDistance));
    }
    // else if(RobotContainer.swerveDrive.hasPoseBeenSet() && !hasHadTarget && startTimer.hasElapsed(.2)){
    //   // Point robot in the general direction of the target if the limelight doesn't see the target
    //   // Finds where we are relative to the center of the field, set as setPoint
    //   setPointAngle = RobotContainer.swerveDrive.getAngleOfTarget();
    //   hasHadTarget = true;
    // }

    // Sets away and lateral speeds using driver axis
    double  awaySpeed = Robot.robotContainer.getDriverAxis(Axis.kLeftY);
    double lateralSpeed = Robot.robotContainer.getDriverAxis(Axis.kLeftX);
    // Check if secondary sticks are being used
    if(Math.abs(Robot.robotContainer.getDriverAxis(Axis.kRightY))>.1 ||
       Math.abs(Robot.robotContainer.getDriverAxis(Axis.kRightX))>.1){
      // If secondary sticks used, replace with secondary sticks witha slow factor
      awaySpeed = Robot.robotContainer.getDriverAxis(Axis.kRightY)*.5;
      lateralSpeed = Robot.robotContainer.getDriverAxis(Axis.kRightX)*.5;
    }
    double rotSpeed = Robot.robotContainer.getDriverAxis(Axis.kRightTrigger) - Robot.robotContainer.getDriverAxis(Axis.kLeftTrigger);
    double output = RobotContainer.swerveDrive.getRobotRotationPIDOut(setPointAngle);

    if (Math.abs(rotSpeed) > .1){
      output = rotSpeed;
    }
    
    if(!(Math.abs(output) < (Constants.MINIMUM_ROTATIONAL_OUTPUT * 2.0) )){
      onTargetTimer.reset();
    }

    if(Math.abs(awaySpeed) < 0.05 && Math.abs(lateralSpeed) < 0.05) {
      //
      if(onTargetTimer.hasElapsed(.5)){
        output = 0.0;  
      }
      RobotContainer.swerveDrive.driveRobotCentric(
        0.0,
        0.0,
        output, 
        false,
        true
      );
    }else {
      // Drive robot using driver axis and rotateToAngle
      RobotContainer.swerveDrive.driveFieldRelative(
        awaySpeed*-Constants.DRIVER_SPEED_SCALE_LINEAR,
        lateralSpeed*-Constants.DRIVER_SPEED_SCALE_LINEAR,
        output,
        false
      );
      onTargetTimer.reset();//reset timer used to stop jitters
    }

    //If we don't have target, don't worry about the timer
    if(!hasTarget){
      onTargetTimer.reset();//reset timer used to stop jitters
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.limeLight.setLightState(false, RobotContainer.swerveDrive);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
