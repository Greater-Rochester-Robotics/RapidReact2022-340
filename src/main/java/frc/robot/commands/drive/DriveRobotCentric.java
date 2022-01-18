/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.Axis;

/**
 * This command is designed so that a driver can drive 
 * the swerve drive based around the robot's orientation.
 * Forward on the stick will cause the robot to drive 
 * forward. left and right on the stick will cause the 
 * robot to move to its left or right. This command does
 * not end of its own accord so it must be interupted to 
 * end.
 */
public class DriveRobotCentric extends CommandBase {
  /**
   * Creates a new DriveRobotCentric.
   */
  public DriveRobotCentric() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //pull primary stick values, and put to awaySpeed and lateralSpeed doubles
    double  forwardSpeed = Robot.robotContainer.getDriverAxis(Axis.LEFT_Y);
    double strafeSpeed = Robot.robotContainer.getDriverAxis(Axis.LEFT_X);
    //check if secondary sticks are being used
    if(Math.abs(Robot.robotContainer.getDriverAxis(Axis.RIGHT_Y))>.1 ||
      Math.abs(Robot.robotContainer.getDriverAxis(Axis.RIGHT_X))>.1){
      //if secondary sticks used, replace with secondary sticks witha slow factor
      forwardSpeed = Robot.robotContainer.getDriverAxis(Axis.RIGHT_Y)*.5;
      strafeSpeed = Robot.robotContainer.getDriverAxis(Axis.RIGHT_X)*.5;
    }
    //create rotation speed from gamepad triggers
    double rotSpeed = Robot.robotContainer.getDriverAxis(Axis.RIGHT_TRIGGER) - Robot.robotContainer.getDriverAxis(Axis.LEFT_TRIGGER);

    RobotContainer.swerveDrive.driveRobotCentric(
      forwardSpeed *-Constants.DRIVER_SPEED_SCALE_LINEAR ,
      strafeSpeed *-Constants.DRIVER_SPEED_SCALE_LINEAR ,
      rotSpeed*-Constants.DRIVER_SPEED_SCALE_ROTATIONAL,
      false
      );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
