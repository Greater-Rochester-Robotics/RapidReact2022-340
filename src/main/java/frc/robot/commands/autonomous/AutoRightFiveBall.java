// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import javax.swing.text.Position;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import frc.robot.RobotContainer;
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
import frc.robot.commands.shooter.ShooterSetSpeed;
import frc.robot.subsystems.BallHandler.State;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoRightFiveBall extends SequentialCommandGroup {
  /** Creates a new AutoRightFiveBall. Doesn't finish yet*/
  @Deprecated
  public AutoRightFiveBall() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new HoodHome(),
      new BallHandlerIntakeOut(),
      parallel(
        new DriveSetGyro(90.00),
        new WaitCommand(0.5),//This must stay here
        new HoodToPosition(6.4),
        new ShooterSetSpeed(7900,true)
      ),
      new BallHandlerShootProgT(0.0).withTimeout(1.0),
      new BallHandlerSetState(State.kFillTo1),
      new ShooterSetSpeed(9500),
      new DriveFollowTrajectory("DriveToRightBall"),
      race(
        new WaitUntilCommand(RobotContainer.ballHandler::isBall1).withTimeout(2.0),//rather than
        new WaitUntilCommand(RobotContainer.ballHandler::isBall0).withTimeout(2.0)

      ),
      parallel(
        new DriveFollowTrajectory("DriveFromRightBallToMidBall"),
        new HoodToPosition(12.5)
      ),
      new WaitUntilCommand(RobotContainer.ballHandler::isBall0).withTimeout(2.0),
      new BallHandlerSetState(State.kOff),
      new DriveTurnToAngle(Math.toRadians(45.28)),
      new BallHandlerShootProgT(0.0),
      new BallHandlerSetState(State.kFillTo1),
      new DriveFollowTrajectory("DriveRightMidBallToHuman"),
      new WaitUntilCommand(RobotContainer.ballHandler::isBall1).withTimeout(2.0),
      new WaitUntilCommand(RobotContainer.ballHandler::isBall0).withTimeout(2.0),
      new BallHandlerSetState(State.kOff),
      parallel(
        new DriveFollowTrajectory("DriveRightFromHumanStraight"),
        new ShooterSetSpeed(9800),
        new HoodToPosition(15.0)
      ),
      // new DriveTurnToAngle(Math.toRadians(37.54)),//not needed, might need turn to target
      new BallHandlerShootProgT(0.0),
      new StopShooterHandlerHood()
      //FenderHighgoal
      // new ShootHighFender(),
      //start intaking
      //drive shortpath to first(Right) ball
      // new DriveFollowTrajectory(trajName),
      //run wait until with Ball1 sensor,with timeout
      //drive another path to second(middle) ball
      //shooter prep shot
      //run wait until on Ball0, with timeout
      //turn to angle(maybe)
      //turn to target
      //shoot
      //intake
      //counter by turning to angle
      //drive back to loading station
      //run wait until on Ball0, with timeout
      //drive forward to within range
      //turn to target 
      //shoot
    );
  }
}
