// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.hood.HoodHome;
import frc.robot.commands.hood.HoodToPosition;
import frc.robot.commands.shooter.ShooterSetSpeed;

public class PrepLowFender extends SequentialCommandGroup {
  /** Creates a new PrepLowFender. */
  public PrepLowFender() {
    addCommands(
      new HoodHome(),//Home the hood, but should fall through if hood has been homed
      parallel(
        new ShooterSetSpeed(Constants.SHOOTER_LOW_GOAL_FENDER_SPEED).withTimeout(2),//Set the Shooter to FenderSpeed
        new HoodToPosition(22.0)//Set the hood to maximum position
      )
    );
  }
}
