// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.ballhandler.BallHandlerSetState;
import frc.robot.commands.shooter.ShooterPercentOutput;
import frc.robot.subsystems.BallHandler.State;


public class SpitBalls extends ParallelCommandGroup {
  /** 
   * A ParrallelCommadnGroup that runs both the shooter 
   * and the BallHandler backwards, thus ejecting balls. 
   * The state of the handler causes the harvester to be 
   * up and not run.
   */
  public SpitBalls() {
    addCommands(
      new ShooterPercentOutput( -.05),
      new BallHandlerSetState(State.kSpitMid1)
    );
  }
}
