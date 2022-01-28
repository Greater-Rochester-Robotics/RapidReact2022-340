// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class ClimberFixedHome extends CommandBase {
  Timer timer = new Timer();
  /** Creates a new ClimberFixedHome. */
  public ClimberFixedHome() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.climber.fixedArmIn();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.climber.stopFixedArm();
    RobotContainer.climber.setFixedEnc(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.climber.getFixedSwitch() ||
    (timer.hasElapsed(0.5) && RobotContainer.climber.getFixedCurrent() > Constants.FIXED_HOMING_CURRENT) ||
    (timer.hasElapsed(0.5) && Math.abs(RobotContainer.climber.getFixedEncVel()) < 0.1);
  }
}
