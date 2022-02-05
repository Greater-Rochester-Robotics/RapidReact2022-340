// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ClimberExtendoToPosition extends CommandBase {
  double position;
  
  public ClimberExtendoToPosition(double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.climber);
    this.position = position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.climber.extendoRightSetPos(position);
    RobotContainer.climber.extendoLeftSetPos(position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.climber.stopExtendoRightArm();
    RobotContainer.climber.stopExtendoLeftArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(RobotContainer.climber.getExtendoRightEncPos() - position) < Constants.EXTENDO_ALLOWABLE_ERROR) &&
       (Math.abs(RobotContainer.climber.getExtendoLeftEncPos() - position) < Constants.EXTENDO_ALLOWABLE_ERROR);
  }
}
