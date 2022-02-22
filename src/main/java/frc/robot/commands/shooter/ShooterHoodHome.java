// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ShooterHoodHome extends CommandBase {
  /** Creates a new ShooterHoodHome. */
  public ShooterHoodHome() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooter);
  }

  @Override
  public void end(boolean interrupted){
    RobotContainer.shooter.stopHoodMotor();
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.shooter.homeHoodPosition();
  }
}
