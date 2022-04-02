// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShootHighGoal;
import frc.robot.commands.ballhandler.BallHandlerIntakeOut;
import frc.robot.commands.ballhandler.BallHandlerSetState;
import frc.robot.commands.drive.auto.DriveFollowTrajectory;
import frc.robot.commands.drive.auto.DriveTurnToTarget;
import frc.robot.commands.shooter.ShooterPrepShot;

/**
 * Starts at tape of left zone.
 * Back left corner of robot is at center of tape.
 * Moves back and shoots one ball.
 */
public class AutoLeftBackOutOfWay extends SequentialCommandGroup {
  /** Creates a new AutoLeftBackOutOfWay. not tested, replaced with {@link AutoDriveBackAndShoot}*/
  @Deprecated
  public AutoLeftBackOutOfWay() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new BallHandlerIntakeOut(),
      new DriveFollowTrajectory("DriveLeftBackOutOfWay"),
      new ShooterPrepShot(),
      new DriveTurnToTarget(),
      new ShootHighGoal(1.0) 
    );
  }
}
