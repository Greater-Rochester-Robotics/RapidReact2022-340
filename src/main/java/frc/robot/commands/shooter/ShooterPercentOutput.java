// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;


public class ShooterPercentOutput extends CommandBase {
  public double output;
  /**
   * A testing command to run the shooter at a percentVoltage
   * output. This command does not by itself, and stops the motor
   * when it is interupted.
   * 
   * @param output The percent voltage to give the shooter [-1,1]
   */
  public ShooterPercentOutput(double output) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooter);
    this.output = output;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.shooter.setOutput(output);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //turn off motor
    RobotContainer.shooter.setOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
