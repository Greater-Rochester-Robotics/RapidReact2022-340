// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ballhandler;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BallHandler.State;

public class BallHandlerSetState extends InstantCommand {

  State state;
  boolean sbmAutoDelay = false;


  /** Creates a new BallHandlerSetState. */

  public BallHandlerSetState(State state) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.ballHandler);
    this.state = state;

  }

  public BallHandlerSetState(State state, boolean sbmDelay){
    this(state);
    sbmAutoDelay = sbmDelay;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.ballHandler.setState(state, sbmAutoDelay);
  }

  public boolean runsWhenDisabled(){
    return true;
  }
}
