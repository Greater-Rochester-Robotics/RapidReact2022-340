// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.util;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;

public class DriveTuneVelocityPID extends CommandBase {
  /** Creates a new DriveVelocityPIDTune. */
  public DriveTuneVelocityPID() {
    addRequirements(RobotContainer.swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = RobotContainer.swerveDrive.getAwaySpeedPIDFFOut(4.0, 0.0);
    RobotContainer.swerveDrive.driveFieldCentric(output,0,0,false);
    System.out.println("Speed: "+RobotContainer.swerveDrive.getCurrentVelocity().getX()+" m/s");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.swerveDrive.stopAllModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
