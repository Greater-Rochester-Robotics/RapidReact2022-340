// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.hood.HoodHome;
import frc.robot.commands.hood.HoodToPosition;
import frc.robot.commands.shooter.ShooterSetSpeed;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PrepLowFender extends SequentialCommandGroup {
  /** Creates a new PrepLowFender. */
  public PrepLowFender() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new HoodHome(),
      parallel(
        new ShooterSetSpeed(Constants.SHOOTER_LOW_GOAL_FENDER_SPEED).withTimeout(2),
        new HoodToPosition(22.0)
      )
    );
  }
}
