// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.StopShooterHandlerHood;
import frc.robot.commands.ballhandler.BallHandlerSetState;
import frc.robot.commands.ballhandler.BallHandlerShootProgT;
import frc.robot.commands.drive.auto.DriveFollowTrajectory;
import frc.robot.commands.drive.auto.DriveStraightBack;
import frc.robot.commands.drive.auto.DriveTurnToTarget;
import frc.robot.commands.drive.util.DriveSetGyro;
import frc.robot.commands.drive.util.DriveTurnToAngle;
import frc.robot.commands.hood.HoodHome;
import frc.robot.commands.hood.HoodToPosition;
import frc.robot.commands.shooter.ShooterSetSpeed;
import frc.robot.commands.shooter.ShooterStop;
import frc.robot.subsystems.BallHandler.State;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPartnerPickupLeftBall extends SequentialCommandGroup {
  /** Creates a new AutoPartnerPickupLeftBall. */
  public AutoPartnerPickupLeftBall() {
    addCommands(
      new HoodHome(),
      new DriveSetGyro(45.0),
      parallel(
        new HoodToPosition(6.2),//TODO:get value
        new ShooterSetSpeed(7900.0),//TODO:get value
        new WaitCommand(2.0)
      ),
      new BallHandlerSetState(State.kFillTo0),
      new WaitUntilCommand(RobotContainer.ballHandler::isBall0).withTimeout(5.0),
      new BallHandlerSetState(State.kOff),
      new WaitCommand(1.0),
      new DriveTurnToAngle(Math.toRadians(-47.42)).withTimeout(2.0),//this angle should be correct
      new BallHandlerShootProgT(0.0),
      parallel(
        new BallHandlerSetState(State.kFillTo1),
        new HoodToPosition(10.3),
        new ShooterSetSpeed(9300.0)
      ),
      new DriveFollowTrajectory("DriveToLeftBallFromPartner"),
      new WaitUntilCommand(RobotContainer.ballHandler::isBall1).withTimeout(2.0),
      new BallHandlerShootProgT(0.0),
      new StopShooterHandlerHood(),
      new DriveStraightBack(0.0)
    );
  }
}
