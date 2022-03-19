// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.ballhandler.BallHandlerShootProgT;
import frc.robot.commands.drive.auto.DriveFollowTrajectory;
import frc.robot.commands.drive.auto.DriveStraightBack;
import frc.robot.commands.hood.HoodHome;
import frc.robot.commands.hood.HoodToPosition;
import frc.robot.commands.shooter.ShooterSetSpeed;
import frc.robot.commands.shooter.ShooterStop;

public class ShootHighFenderWithDriveBack extends SequentialCommandGroup {
  /** Creates a new ShootHighGoal. */
  public ShootHighFenderWithDriveBack(double timeBewtweenBalls) {
    addCommands(
      new HoodHome(),
      parallel(
        new ShooterSetSpeed(Constants.SHOOTER_FENDER_SHOT_SPEED).withTimeout(2),
        new HoodToPosition(0.0),
        new DriveStraightBack(0.305)
      ),
      // new WaitUntilCommand(RobotContainer.shooter::isAtSpeed),//this should fall through, left for options
      new BallHandlerShootProgT(timeBewtweenBalls),
      new ShooterStop()
    );
  }
}
