// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.ballhandler.BallHandlerSetState;
import frc.robot.commands.hood.HoodStop;
import frc.robot.commands.shooter.ShooterStop;
import frc.robot.subsystems.BallHandler.State;


public class StopShooterHandlerHood extends ParallelCommandGroup {
  /** 
   * A commandgroup that stops the hood, then the shooter, 
   * then the BallHandler, each is an InstantCommand.
   */
  public StopShooterHandlerHood() {
    addCommands(
      new HoodStop(),
      new ShooterStop(),
      new BallHandlerSetState(State.kOff)
    );
  }
}
