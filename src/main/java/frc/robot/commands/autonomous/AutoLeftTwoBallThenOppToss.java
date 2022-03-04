// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.ShootHighGoal;
import frc.robot.commands.ShootHighGoalWithoutLL;
import frc.robot.commands.ballhandler.BallHandlerSetState;
import frc.robot.commands.drive.DriveOnTarget;
import frc.robot.commands.drive.auto.DriveFollowTrajectory;
import frc.robot.commands.drive.auto.DriveTurnToTarget;
import frc.robot.commands.drive.util.DriveTurnToAngle;
import frc.robot.commands.shooter.ShooterPrepShot;
import frc.robot.subsystems.BallHandler.State;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
/**
 * TODO: Needs to fixed
 */
@Deprecated
public class AutoLeftTwoBallThenOppToss extends SequentialCommandGroup {
  /** Creates a new AutoLefftTwoBallThenOppToss. not tested*/
  @Deprecated
  public AutoLeftTwoBallThenOppToss() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new BallHandlerSetState(State.kFillTo1),
      new DriveFollowTrajectory("DriveToLeftBallFromHub"),
      new ShooterPrepShot(),
      new WaitUntilCommand(RobotContainer.ballHandler::isBall0).withTimeout(2.0),
      new DriveTurnToTarget(),
      new BallHandlerSetState(State.kOff),
      new ShootHighGoal(1.0),
      new BallHandlerSetState(State.kFillTo1),
      new DriveFollowTrajectory("DriveLeftBallToOpponentBall"),
      new ShooterPrepShot(),
      new WaitUntilCommand(RobotContainer.ballHandler::isBall1).withTimeout(2.0),
      new DriveTurnToAngle(15.0 * Math.PI / 180),
      new BallHandlerSetState(State.kOff),
      new ShootHighGoalWithoutLL(1.0)
    );
  }
}
