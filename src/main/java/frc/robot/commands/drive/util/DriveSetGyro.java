// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.util;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;


public class DriveSetGyro extends InstantCommand {
  private double angle;

  /**
   * Set the gyro's current angle to the input 
   * param. CCW is positive.
   * 
   * @param angle an angle in DEGREES!!!
   */
  public DriveSetGyro(double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.swerveDrive);
    this.angle = angle;
   
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.swerveDrive.setGyro(angle);
  }

  public boolean runsWhenDisabled(){
    return true;
  }
}
