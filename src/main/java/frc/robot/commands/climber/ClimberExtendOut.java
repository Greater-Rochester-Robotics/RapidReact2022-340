// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

/**
 * This is a testing command to drive the Arms Out. it should be bound to a whileHeld() button
 */
public class ClimberExtendOut extends CommandBase {
  /** Creates a new ClimberExtendOut. */
  public ClimberExtendOut() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.climber.extendoArmLeftOut();
    RobotContainer.climber.extendoArmRightOut();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.climber.stopExtendoLeftArm();
    RobotContainer.climber.stopExtendoRightArm();;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
