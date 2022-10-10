// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.StopShooterHandlerHood;
import frc.robot.commands.ballhandler.BallHandlerIntakeOut;
import frc.robot.commands.ballhandler.BallHandlerSetState;
import frc.robot.commands.ballhandler.BallHandlerShootProgT;
import frc.robot.commands.drive.auto.DriveFollowTrajectory;
import frc.robot.commands.drive.util.DriveSetGyro;
import frc.robot.commands.drive.util.DriveTurnToAngleInRad;
import frc.robot.commands.hood.HoodHome;
import frc.robot.commands.hood.HoodToPosition;
import frc.robot.commands.shooter.ShooterPrepShot;
import frc.robot.commands.shooter.ShooterSetSpeed;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.BallHandler.State;

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
        new ShooterSetSpeed(7900, true)//7700
      ),
      parallel(
        new ShooterSetSpeed(RobotContainer.limeLight::getShooterHighSpeed,true).withTimeout(2.0),
        new HoodToPosition(RobotContainer.limeLight::getHoodHighAngle,true).withTimeout(2.0)
      ),
      new BallHandlerShootProgT(0.0).withTimeout(1.0),
      new StopShooterHandlerHood(),
      new BallHandlerSetState(State.kFillTo1),
      new DriveFollowTrajectory("DriveToRightBall"),
      new WaitUntilCommand(RobotContainer.ballHandler::isBall1).withTimeout(2.0),
      parallel(
        new DriveFollowTrajectory("DriveFromRightBallToMidBall"),
        new HoodToPosition(12.5),
        sequence(
          new WaitCommand(1.0),
          new ShooterSetSpeed(9500)//9300 is real, rescale to 9600
        )
      ),
      new WaitUntilCommand(RobotContainer.ballHandler::isBall0).withTimeout(2.0),
      new DriveTurnToAngleInRad(Math.toRadians(45.28)),
      parallel(
        new ShooterSetSpeed(RobotContainer.limeLight::getShooterHighSpeed,true).withTimeout(2.0),
        new HoodToPosition(RobotContainer.limeLight::getHoodHighAngle,true).withTimeout(2.0)
      ),
      new BallHandlerShootProgT(0.0),
      new StopShooterHandlerHood()
    );
  }
}
