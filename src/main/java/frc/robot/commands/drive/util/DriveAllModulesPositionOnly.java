// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController.Axis;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive.SwerveModNum;

/**
 * This command is for tuning the PIDF loop on the modules' 
 * rotation motors, and other such test. It rotates the 
 * modules to a direction based on the movement of the 
 * joystick. Move the joystick in a direction to make all 
 * modules should go to that direction.
 */
public class DriveAllModulesPositionOnly extends CommandBase {
  //create a value for the modules to rotate to, we will update this later
  private double rotatePos = 0;
  /** Creates a new DriveAllModulesPositionOnly. */
  public DriveAllModulesPositionOnly() {

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Call the stop all command onto all the modules so they are not moving forward or backward.
    RobotContainer.swerveDrive.stopAllModules();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //check to see that the joystick is far enough from centered
    if ((Math.abs(Robot.robotContainer.getDriverAxis(Axis.kLeftY)) > .1)
        || (Math.abs(Robot.robotContainer.getDriverAxis(Axis.kLeftX)) > .1)){
      //take the angle the joystick is moved to and make that the rotation target position
      rotatePos = Math.atan2(Robot.robotContainer.getDriverAxis(Axis.kLeftY),
        Robot.robotContainer.getDriverAxis(Axis.kLeftX));
    }

    for (int i=0 ; i<4 ; i++){
      //assign the rotational direction to each module
      RobotContainer.swerveDrive.driveOneModule(i, 0, rotatePos, false);
    }

    //test smartdashboard outputs, Module Angles and Encoder Counts
    double[] modAngles = RobotContainer.swerveDrive.getAllAbsModuleAngles();
    double[] modRelEnc = RobotContainer.swerveDrive.getAllModuleRelEnc();
    SmartDashboard.putNumber("frontLeftAngle", modAngles[SwerveModNum.frontLeft.getNumber()]);
    SmartDashboard.putNumber("frontLeftEnc", modRelEnc[SwerveModNum.frontLeft.getNumber()]);
    SmartDashboard.putNumber("frontRightAngle", modAngles[SwerveModNum.frontRight.getNumber()]);
    SmartDashboard.putNumber("frontRightEnc", modRelEnc[SwerveModNum.frontRight.getNumber()]);
    SmartDashboard.putNumber("rearLeftAngle", modAngles[SwerveModNum.rearLeft.getNumber()]);
    SmartDashboard.putNumber("rearLeftEnc", modRelEnc[SwerveModNum.rearLeft.getNumber()]);
    SmartDashboard.putNumber("rearRightAngle", modAngles[SwerveModNum.rearRight.getNumber()]);
    SmartDashboard.putNumber("rearRightEnc", modRelEnc[SwerveModNum.rearRight.getNumber()]);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Call the stop all command onto all the modules so they can freely spin.
    RobotContainer.swerveDrive.stopAllModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
