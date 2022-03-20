// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.ShootHighGoal;
import frc.robot.commands.StopShooterHandlerHood;
import frc.robot.commands.ballhandler.BallHandlerSetState;
import frc.robot.commands.ballhandler.BallHandlerShootProgT;
import frc.robot.commands.drive.auto.DriveFollowTrajectory;
import frc.robot.commands.drive.auto.DriveTurnToTarget;
import frc.robot.commands.drive.util.DriveSetGyro;
import frc.robot.commands.hood.HoodHome;
import frc.robot.commands.hood.HoodToPosition;
import frc.robot.commands.shooter.ShooterPrepShot;
import frc.robot.commands.shooter.ShooterSetSpeed;
import frc.robot.commands.shooter.ShooterStop;
import frc.robot.subsystems.BallHandler.State;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
/**
 * Starts at right zone, back is on line, front left corner touching left line.
 * Moves to pick up the middle ball and shoots both.
 * Moves back to human ball while human player is loading other ball
 * Shoots both balls at the high goal
 */
public class AutoMidFourBall extends SequentialCommandGroup {
  /** Creates a new AutoMidFourBall. */
  public AutoMidFourBall() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new HoodHome(),
      new BallHandlerSetState(State.kFillTo0),
      parallel(
        new DriveSetGyro(46.04),
        new WaitCommand(.75)//We need to wait for two reasons, the gyro takes time too set the value, and the harvester needs to come down
      ),
      new ShooterSetSpeed(Constants.SHOOTER_HIGH_SPEEDS_TABLE.lookup(118)),//need 8950ish so going with 9200 bc bad pid, no need to wait to get to speed
      parallel(
        new DriveFollowTrajectory("DriveToMidBall", 4.5, 1.5),
        new HoodToPosition(Constants.HOOD_HIGH_POSITION_TABLE.lookup(118))//while driving to the ball, set the hood
      ),
      new WaitUntilCommand(RobotContainer.ballHandler::isBall0).withTimeout(2.0),
      new BallHandlerShootProgT(0.0).withTimeout(2.0),
      new BallHandlerSetState(State.kFillTo1),
      new DriveFollowTrajectory("DriveMidBallToHuman", 4.5, 1.5),
      new WaitUntilCommand(RobotContainer.ballHandler::isBall1).withTimeout(2.0),//wait to get the first ball
      new WaitUntilCommand(RobotContainer.ballHandler::isBall0).withTimeout(2.0),//now wait for the second ball
      // new BallHandlerSetState(State.kOff),
      new ShooterSetSpeed(Constants.SHOOTER_HIGH_SPEEDS_TABLE.lookup(144)),//need 9400 so use 9700 bc bad pid, no need to wait to get to speed
      parallel(
        new DriveFollowTrajectory("DriveFromHumanStraight", 4.5, 1.5),
        new HoodToPosition(Constants.HOOD_HIGH_POSITION_TABLE.lookup(144))//while driving to the shooting point, set the hood
      ),
      new BallHandlerShootProgT(0.0),
      new StopShooterHandlerHood()
    );
  }
}
