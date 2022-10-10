// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.ShootHighGoal;
import frc.robot.commands.StopShooterHandlerHood;
import frc.robot.commands.ballhandler.BallHandlerSetState;
import frc.robot.commands.ballhandler.BallHandlerShootProgT;
import frc.robot.commands.drive.auto.DriveFollowTrajectory;
import frc.robot.commands.drive.auto.DriveTurnToTarget;
import frc.robot.commands.drive.util.DriveSetGyro;
import frc.robot.commands.drive.util.DriveTurnToAngleInRad;
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
 */
public class AutoMidTwoBall extends SequentialCommandGroup {
  /** Creates a new AutoMidTwoBall. */
  public AutoMidTwoBall() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new HoodHome(),
      new BallHandlerSetState(State.kFillTo0),
      parallel(
        new DriveSetGyro(46.04),
        new WaitCommand(.75)//We need to wait for two reasons, the gyro takes time too set the value, and the harvester needs to come down
      ),
      new ShooterSetSpeed(9200),//need 8950ish so going with 9200 bc bad pid, no need to wait to get to speed
      parallel(
        new DriveFollowTrajectory("DriveToMidBall"),
        new HoodToPosition(9.9)//while driving to the ball, set the hood
      ),
      race(
        new WaitUntilCommand(RobotContainer.ballHandler::isBall0).withTimeout(4.0),//Wait for ball0 switch, race with a wiggle
        sequence(
          new DriveTurnToAngleInRad(Math.toRadians(31.9)).withTimeout(1.0),
          new WaitCommand(.5),
          new DriveTurnToAngleInRad(Math.toRadians(41.9)).withTimeout(2.0),
          new WaitCommand(.5)
        )
      ),
      new DriveTurnToAngleInRad(Math.toRadians(36.9)).withTimeout(1.5),
      parallel(
        new ShooterSetSpeed(RobotContainer.limeLight::getShooterHighSpeed,true).withTimeout(1.0),
        new HoodToPosition(RobotContainer.limeLight::getHoodHighAngle,true).withTimeout(1.0)
      ),
      new BallHandlerShootProgT(0.0),
      new StopShooterHandlerHood()
    );
  }
}
