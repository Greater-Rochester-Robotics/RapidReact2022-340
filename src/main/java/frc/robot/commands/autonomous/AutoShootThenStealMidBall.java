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
import frc.robot.commands.ballhandler.BallHandlerIntakeOut;
import frc.robot.commands.ballhandler.BallHandlerRejectOppColor;
import frc.robot.commands.ballhandler.BallHandlerSetState;
import frc.robot.commands.ballhandler.BallHandlerShootProgT;
import frc.robot.commands.drive.auto.DriveFollowTrajectory;
import frc.robot.commands.drive.auto.DriveTurnToTarget;
import frc.robot.commands.drive.util.DriveResetGyroToZero;
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
public class AutoShootThenStealMidBall extends SequentialCommandGroup {
  /** Creates a new AutoShootThenStealMidBall. */
  public AutoShootThenStealMidBall() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new HoodHome(),
      new BallHandlerIntakeOut(),
      parallel(
        new DriveSetGyro(0.0),
        new ShooterSetSpeed(Constants.SHOOTER_FENDER_SHOT_SPEED + 100,true).withTimeout(2.0),
        new HoodToPosition(0.0).withTimeout(2.0)
      ),
      new BallHandlerShootProgT(0.0).withTimeout(2.0),
      new ShooterStop(),
      new BallHandlerRejectOppColor(false),
      new BallHandlerSetState(State.kFillTo0),
      parallel(
        new WaitCommand(3.0),//Wait For 5 ball robot to get its third ball
        new DriveTurnToAngleInRad(Math.toRadians(29.0)).withTimeout(3.0)
      ),
      new DriveFollowTrajectory("DriveToMidOppBall"),
      race(
        new WaitUntilCommand(RobotContainer.ballHandler::isBall0).withTimeout(4.0),//Wait for ball0 switch, race with a wiggle
        sequence(
          new DriveTurnToAngleInRad(Math.toRadians(24.0)).withTimeout(1.0),
          new WaitCommand(.5),
          new DriveTurnToAngleInRad(Math.toRadians(34.0)).withTimeout(2.0),
          new WaitCommand(.5)
        )
      ),
      new BallHandlerIntakeOut(),
      new BallHandlerRejectOppColor(),
      new DriveTurnToAngleInRad(Math.toRadians(-17.0)).withTimeout(1.5),
      new BallHandlerSetState(State.kSpitLow0),
      new WaitCommand(3.0),
      new StopShooterHandlerHood()
    );
  }
}
