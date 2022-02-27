// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.ShootHighGoal;
import frc.robot.commands.ballhandler.BallHandlerIntakeIn;
import frc.robot.commands.ballhandler.BallHandlerIntakeOut;
import frc.robot.commands.ballhandler.BallHandlerSetState;
import frc.robot.commands.drive.auto.DriveFollowTrajectory;
import frc.robot.commands.drive.auto.DriveTurnToTarget;
import frc.robot.commands.shooter.ShooterPrepShot;
import frc.robot.subsystems.BallHandler;
import frc.robot.subsystems.BallHandler.State;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
/**
 * Starting at hub, corner at leftmost hub corner.
 * Pick up left ball and shoot both balls.
 */
public class AutoLeftSideTwoBallFromHub extends SequentialCommandGroup {
  /** Creates a new AutoTwoBall. */
  public AutoLeftSideTwoBallFromHub() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new BallHandlerSetState(State.kFillTo1),
      new DriveFollowTrajectory("DriveToLeftBall"),
      new WaitUntilCommand(RobotContainer.ballHandler::isBall0).withTimeout(2.0),
      new BallHandlerSetState(State.kOff),
      new ShooterPrepShot(),
      new DriveTurnToTarget(),
      new ShootHighGoal(1.0)
    );
  }
}
