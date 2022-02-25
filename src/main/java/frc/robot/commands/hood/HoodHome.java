// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hood;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class HoodHome extends CommandBase {
  boolean overrideBeenHomed = false;
  /** Creates a new HoodHome. */
  public HoodHome() {
    this(false);
  }

  public HoodHome(boolean overrideBeenHomed){
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.hood);
    this.overrideBeenHomed = overrideBeenHomed;
    
  }

  @Override
  public void end(boolean interrupted){
    RobotContainer.hood.stopMotor();
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.hood.homePosition() || (!overrideBeenHomed && RobotContainer.hood.hasBeenHomed());
  }
}
