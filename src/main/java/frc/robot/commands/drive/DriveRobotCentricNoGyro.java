/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController.Axis;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ADIS.IMUAxis;

/**
 * This command is designed for when the climber is being 
 * run. It is very much programmed the same as 
 * {@link DriveRobotCentric}. The difference is that it 
 * disables the gyro reading in the Y plane and turn it to 
 * reading it in the Z. It re engages reading the gyro in 
 * Y when the command is ended. 
 *  
 * Drive control is based around the robot's orientation.
 * Forward on the stick will cause the robot to drive 
 * forward. left and right on the stick will cause the 
 * robot to move to its left or right. This command does
 * not end of its own accord so it must be interupted to 
 * end.
 */
public class DriveRobotCentricNoGyro extends CommandBase {
  /**
   * Creates a new DriveRobotCentricNoGyro.
   */
  public DriveRobotCentricNoGyro() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.swerveDrive.setGyroAxis(IMUAxis.kZ);
    //TODO: figure a way for the reset to only happen when we are on the ground, 
    RobotContainer.swerveDrive.resetGyroZ();
    RobotContainer.swerveDrive.setIsOdometry(false);
    RobotContainer.setDriverRumble(0.25, 0.25);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.setDriverRumble(0.25, 0.25);
    //pull primary stick values, and put to awaySpeed and lateralSpeed doubles
    double forwardSpeed = Robot.robotContainer.getDriverAxis(Axis.kLeftY);
    double strafeSpeed = Robot.robotContainer.getDriverAxis(Axis.kLeftX);
    //check if secondary sticks are being used
    if(Math.abs(Robot.robotContainer.getDriverAxis(Axis.kRightY))>.1 ||
      Math.abs(Robot.robotContainer.getDriverAxis(Axis.kRightX))>.1){
      //if secondary sticks used, replace with secondary sticks witha slow factor
      forwardSpeed = Robot.robotContainer.getDriverAxis(Axis.kRightY)*.5;
      strafeSpeed = Robot.robotContainer.getDriverAxis(Axis.kRightX)*.5;
    }
    //create rotation speed from gamepad triggers
    double rotSpeed = Robot.robotContainer.getDriverAxis(Axis.kRightTrigger) - Robot.robotContainer.getDriverAxis(Axis.kLeftTrigger);

    RobotContainer.swerveDrive.driveRobotCentric(
      forwardSpeed *-Constants.DRIVER_SPEED_SCALE_LINEAR ,
      strafeSpeed *-Constants.DRIVER_SPEED_SCALE_LINEAR ,
      rotSpeed*-Constants.DRIVER_SPEED_SCALE_ROTATIONAL,
      false,
      false
      );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.swerveDrive.setGyroAxis(IMUAxis.kY);
    RobotContainer.swerveDrive.resetGyro();
    RobotContainer.setDriverRumble(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
