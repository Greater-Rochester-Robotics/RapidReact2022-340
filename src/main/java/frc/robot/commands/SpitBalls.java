// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.ballhandler.BallHandlerSetState;
import frc.robot.commands.shooter.ShooterPercentOutput;
import frc.robot.subsystems.BallHandler.State;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SpitBalls extends ParallelCommandGroup {
  /** Creates a new SpitBalls. */
  public SpitBalls() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ShooterPercentOutput( -.1),
      new BallHandlerSetState(State.kSpitMid)
    );
  }
}