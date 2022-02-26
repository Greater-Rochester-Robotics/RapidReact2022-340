// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.ballhandler.BallHandlerShootProgT;
import frc.robot.commands.hood.HoodHome;
import frc.robot.commands.hood.HoodToPosition;
import frc.robot.commands.shooter.ShooterSetSpeed;
import frc.robot.commands.shooter.ShooterStop;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootHighGoalFender extends SequentialCommandGroup {
  /** Creates a new ShootHighGoal. */
  public ShootHighGoalFender(double timeBewtweenBalls) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new HoodHome(),
      parallel(
        new ShooterSetSpeed(Constants.SHOOTER_HIGH_SPEEDS_TABLE.lookup(0)).withTimeout(2),
        new HoodToPosition(0.0)
      ),
      // new WaitUntilCommand(RobotContainer.shooter::isAtSpeed),//this should fall through, left for options
      new BallHandlerShootProgT(timeBewtweenBalls),
      new ShooterStop()
    );
  }
}