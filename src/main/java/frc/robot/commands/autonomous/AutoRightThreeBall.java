// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.ShootHighGoal;
import frc.robot.commands.StopShooterHandlerHood;
import frc.robot.commands.ballhandler.BallHandlerIntakeOut;
import frc.robot.commands.ballhandler.BallHandlerSetState;
import frc.robot.commands.ballhandler.BallHandlerShootProgT;
import frc.robot.commands.drive.auto.DriveFollowTrajectory;
import frc.robot.commands.drive.auto.DriveTurnToTarget;
import frc.robot.commands.drive.util.DriveSetGyro;
import frc.robot.commands.drive.util.DriveTurnToAngle;
import frc.robot.commands.hood.HoodHome;
import frc.robot.commands.hood.HoodToPosition;
import frc.robot.commands.shooter.ShooterPrepShot;
import frc.robot.commands.shooter.ShooterSetSpeed;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.BallHandler.State;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
/**
 * Starts at right zone, with left bumper touching center corner.
 * Moves back to pick up right ball and shoots both.
 * Moves to mid ball and shoots.
 */
public class AutoRightThreeBall extends SequentialCommandGroup {
  /** Creates a new AutoRightThreeBall. */
  public AutoRightThreeBall() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new HoodHome(),
      new BallHandlerIntakeOut(),///Make sure intake out for shooting
      parallel(
        new DriveSetGyro(90.0),
        new HoodToPosition(6.4),
        new ShooterSetSpeed(7900)//7700
        
      ),
      new BallHandlerShootProgT(0.0).withTimeout(1.0),
      new BallHandlerSetState(State.kFillTo1),
      new ShooterSetSpeed(9600),
      new DriveFollowTrajectory("DriveToRightBall"),
      new WaitUntilCommand(RobotContainer.ballHandler::isBall1).withTimeout(2.0),
      parallel(
        new DriveFollowTrajectory("DriveFromRightBallToMidBall"),
        new HoodToPosition(23.0)//TODO:This is the problem. the hood can not be set at values greater than 22. this number is not what we had
      ),
      new WaitUntilCommand(RobotContainer.ballHandler::isBall0).withTimeout(2.0),
      new DriveTurnToAngle(Math.toDegrees(42.16)).withTimeout(2.5),//TODO: perhaps replace this with a Trajectory
      new BallHandlerShootProgT(0.0),
      new StopShooterHandlerHood()

      // new BallHandlerSetState(State.kFillTo1),
      // new DriveFollowTrajectory("DriveFromRightBallToMidBall"),
      // new WaitUntilCommand(RobotContainer.ballHandler::isBall1).withTimeout(2.0),
      // new BallHandlerSetState(State.kOff),
      // new ShooterPrepShot(),
      // new DriveTurnToTarget(),
      // new ShootHighGoal(0.0)
    );
  }
}
