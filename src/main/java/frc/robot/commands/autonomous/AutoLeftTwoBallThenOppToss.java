// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.ShootHighGoal;
import frc.robot.commands.ShootHighGoalWithoutLL;
import frc.robot.commands.ballhandler.BallHandlerSetState;
import frc.robot.commands.ballhandler.BallHandlerShootProgT;
import frc.robot.commands.drive.DriveOnTarget;
import frc.robot.commands.drive.auto.DriveFollowTrajectory;
import frc.robot.commands.drive.auto.DriveTurnToTarget;
import frc.robot.commands.drive.util.DriveSetGyro;
import frc.robot.commands.drive.util.DriveTurnToAngle;
import frc.robot.commands.hood.HoodHome;
import frc.robot.commands.hood.HoodToPosition;
import frc.robot.commands.shooter.ShooterPrepShot;
import frc.robot.commands.shooter.ShooterSetSpeed;
import frc.robot.subsystems.BallHandler.State;

/**
 * TODO: Needs to fixed
 */
@Deprecated
public class AutoLeftTwoBallThenOppToss extends SequentialCommandGroup {
  /** Creates a new AutoLeftTwoBallThenOppToss. not tested*/
  @Deprecated
  public AutoLeftTwoBallThenOppToss() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new HoodHome(),
      new BallHandlerSetState(State.kFillTo0),
      parallel(
        new DriveSetGyro(-45.0),
        new WaitCommand(.75)//We need to wait for two reasons, the gyro takes time too set the value, and the harvester needs to come down
      ),
      new ShooterSetSpeed(9200),
      parallel(
        new DriveFollowTrajectory("DriveToLeftBall"),
        new HoodToPosition(9.9)//while driving to the ball, set the hood
      ),
      race(
        new WaitUntilCommand(RobotContainer.ballHandler::isBall0).withTimeout(4.0),//Wait for ball0 switch, race with a wiggle
        sequence(
          new DriveTurnToAngle(Math.toRadians(-26.43)).withTimeout(1.0),//wiggle 5 degrees clockwise
          new WaitCommand(.5),//wait for a moment 
          new DriveTurnToAngle(Math.toRadians(-36.43)).withTimeout(2.0),//wiggle 5 degrees counter-clockwise(total of 10 deg)
          new WaitCommand(.5)//wait for a moment
        )
      ),
      new DriveTurnToAngle(Math.toRadians(-31.43)).withTimeout(1.5),//make sure we return to start rotation
      new BallHandlerShootProgT(0.0),
      //TODO: set intake to not reject Opp color(need to make a new command)
      new BallHandlerSetState(State.kFillTo1),
      new DriveFollowTrajectory("DriveLeftBallToOpponentBall"),//TODO: parallel this with hood and shooter to lowfender settings
      new WaitUntilCommand(RobotContainer.ballHandler::isBall1).withTimeout(2.0),//TODO: use this a race with previous parallel, also test against Ball0 sensor
      //TODO: re-enable the color sensing.
      new DriveTurnToAngle(15.0 * Math.PI / 180),//TODO: adjust this angle, we want to shoot into the hanger
      new BallHandlerSetState(State.kOff)
      //TODO: shoot the ball
      //TODO: stop the shooter
    );
  }
}
