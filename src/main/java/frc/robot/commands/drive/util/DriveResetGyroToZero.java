/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive.util;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.RobotContainer;

public class DriveResetGyroToZero extends InstantCommand {
  /**
   * Resets the Gyro to zero, instand command, ends immediately
   */ 
  public DriveResetGyroToZero() {
    addRequirements(RobotContainer.swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.swerveDrive.resetGyro();
  }

  public boolean runsWhenDisabled(){
    return true;
  }
}
