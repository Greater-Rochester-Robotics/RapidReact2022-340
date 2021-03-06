// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ballhandler;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class BallHandlerRejectOppColor extends InstantCommand {
  boolean rejectOppColor = true;

  public BallHandlerRejectOppColor() {
    this(true);
  }

  public BallHandlerRejectOppColor(boolean rejectOppColor) {
    addRequirements(RobotContainer.ballHandler);
    this.rejectOppColor = rejectOppColor;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.ballHandler.rejectOppColor(rejectOppColor);
  }

  public boolean runsWhenDisabled(){
    return true;
  }
}
