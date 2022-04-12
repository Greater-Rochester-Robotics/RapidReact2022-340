// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController.Axis;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ADIS.IMUAxis;

/**
 * This command is designed so that a driver can drive 
 * the swerve drive based around a fixed orientation.
 * Forward on the stick should cause the robot to away 
 * from the driver. If this is true, then left and right 
 * on the stick will cause the robot to move to the 
 * driver's left and right, respectively. This command 
 * does not end of its own accord so it must be interrupted 
 * to end.
 * 
 * UNLIKE DriveFieldCentric this command uses a PIDController 
 * to maintain the robot's rotational orientation when the 
 * robot is not instructed to rotate by the rotational 
 * input. 
 */

public class DriveFieldRelativeAdvanced extends CommandBase {
  private double currentAngle = 0;
  private boolean wasDriverControl;

  /** Creates a new DriveFieldCentricAdvanced. */
  public DriveFieldRelativeAdvanced() {
    addRequirements(RobotContainer.swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.swerveDrive.setGyroAxis(IMUAxis.kY);
    // RobotContainer.swerveDrive.setIsOdometry(false);
    currentAngle = RobotContainer.swerveDrive.getGyroInRad();
    wasDriverControl = false;
    RobotContainer.swerveDrive.setIsOdometry(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //pull primary stick values, and put to awaySpeed and lateralSpeed doubles
    double awaySpeed = Robot.robotContainer.getDriverAxis(Axis.kLeftY);
    double lateralSpeed = Robot.robotContainer.getDriverAxis(Axis.kLeftX);
    //check if secondary sticks are being used
    if(Math.abs(Robot.robotContainer.getDriverAxis(Axis.kRightY))>.1 ||
      Math.abs(Robot.robotContainer.getDriverAxis(Axis.kRightX))>.1){
      //if secondary sticks used, replace with secondary sticks witha slow factor
      awaySpeed = Robot.robotContainer.getDriverAxis(Axis.kRightY)*.5;
      lateralSpeed = Robot.robotContainer.getDriverAxis(Axis.kRightX)*.5;
    }
    //create rotation speed from gamepad triggers
    double rotSpeed = Robot.robotContainer.getDriverAxis(Axis.kRightTrigger) - Robot.robotContainer.getDriverAxis(Axis.kLeftTrigger);

    //use DPad to turn to specific angles.
    // if(Robot.robotContainer.getDriverDPad() == 0){
    //   currentAngle = Math.round(RobotContainer.swerveDrive.getGyroInRad()/Constants.TWO_PI) * Constants.TWO_PI;
    // }
    // else if(Robot.robotContainer.getDriverDPad() == 90){
    //   currentAngle = Math.round(RobotContainer.swerveDrive.getGyroInRad()/Constants.TWO_PI) * Constants.TWO_PI - 1.178;
    // }

    //test if the absolute rotational input is greater than .1
    if (Math.abs(rotSpeed) > .1){
      //if the test is true, just copy the DriveFieldCentric execute method
      RobotContainer.swerveDrive.driveFieldRelative(
        awaySpeed*-Constants.DRIVER_SPEED_SCALE_LINEAR,
        lateralSpeed*-Constants.DRIVER_SPEED_SCALE_LINEAR,
        rotSpeed*-Constants.DRIVER_SPEED_SCALE_ROTATIONAL ,
        false
      );
      //for when rotation speed is zero, update the current angle
      currentAngle = RobotContainer.swerveDrive.getGyroInRad();
      wasDriverControl = true;

    }
    else {
      if(wasDriverControl && Math.abs(RobotContainer.swerveDrive.getRotationalVelocity()) > 90.0){
        RobotContainer.swerveDrive.driveFieldRelative(
          awaySpeed*-Constants.DRIVER_SPEED_SCALE_LINEAR,
          lateralSpeed*-Constants.DRIVER_SPEED_SCALE_LINEAR,
          0,
          false
        );
        currentAngle = RobotContainer.swerveDrive.getGyroInRad();
      }else{
        //if the test is false, still use driveFieldCentric(), but for last parameter use PIDController accessor function
        RobotContainer.swerveDrive.driveFieldRelative(
          awaySpeed*-Constants.DRIVER_SPEED_SCALE_LINEAR,
          lateralSpeed*-Constants.DRIVER_SPEED_SCALE_LINEAR,
          RobotContainer.swerveDrive.getCounterRotationPIDOut(currentAngle),
          false
        );
        wasDriverControl = false;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.swerveDrive.setIsOdometry(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
