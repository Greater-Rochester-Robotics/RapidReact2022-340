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
 * the swerve drive based around a fixed orientation.
 * Forward on the stick should cause the robot to away 
 * from the driver. If this is true, then left and right 
 * on the stick will cause the robot to move to the 
 * driver's left and right, respectively. This command 
 * does not end of its own accord so it must be interupted 
 * to end.
 */
public class DriveFieldCentricVelocity extends CommandBase {
  /**
   * Creates a new DriveFieldCentric.
   */
  @Deprecated
  public DriveFieldCentricVelocity() {
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
    double  awaySpeed = Robot.robotContainer.getDriverAxis(Axis.LEFT_Y);
    double lateralSpeed = Robot.robotContainer.getDriverAxis(Axis.LEFT_X);
    double rotSpeed = Robot.robotContainer.getDriverAxis(Axis.RIGHT_TRIGGER) - Robot.robotContainer.getDriverAxis(Axis.LEFT_TRIGGER);


    RobotContainer.swerveDrive.driveFieldCentric(
      awaySpeed*-Constants.DRIVER_SPEED_SCALE_LINEAR*Constants.MAXIMUM_VELOCITY,
      lateralSpeed*-Constants.DRIVER_SPEED_SCALE_LINEAR*Constants.MAXIMUM_VELOCITY,
      rotSpeed*-Constants.DRIVER_SPEED_SCALE_ROTATIONAL*Constants.MAX_ROBOT_ROT_VELOCITY, 
      true
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
