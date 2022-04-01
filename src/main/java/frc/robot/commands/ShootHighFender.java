// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import frc.robot.Constants;
import frc.robot.commands.ballhandler.BallHandlerShootProgT;
import frc.robot.commands.hood.HoodHome;
import frc.robot.commands.hood.HoodToPosition;
import frc.robot.commands.shooter.ShooterSetSpeed;
import frc.robot.commands.shooter.ShooterStop;


public class ShootHighFender extends SequentialCommandGroup {
  /**
   * replaced by ShootHighFenderWithDriveBack
   * @param timeBewtweenBalls
   */
  public ShootHighFender(double timeBewtweenBalls) {
    addCommands(
      new HoodHome(),
      parallel(
        new ShooterSetSpeed(Constants.SHOOTER_FENDER_SHOT_SPEED,true).withTimeout(2),
        new HoodToPosition(0.0)
      ),
      new BallHandlerShootProgT(timeBewtweenBalls),
      new ShooterStop()
    );
  }
}
