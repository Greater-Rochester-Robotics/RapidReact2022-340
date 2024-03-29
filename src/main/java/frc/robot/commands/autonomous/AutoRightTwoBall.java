// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import frc.robot.commands.StopShooterHandlerHood;
import frc.robot.commands.ballhandler.BallHandlerIntakeOut;
import frc.robot.commands.ballhandler.BallHandlerSetState;
import frc.robot.commands.ballhandler.BallHandlerShootProgT;
import frc.robot.commands.drive.auto.DriveFollowTrajectory;
import frc.robot.commands.drive.auto.DriveStraightBack;
import frc.robot.commands.drive.util.DriveSetGyro;
import frc.robot.commands.drive.util.DriveTurnToAngleInRad;
import frc.robot.commands.hood.HoodHome;
import frc.robot.commands.hood.HoodToPosition;
import frc.robot.commands.shooter.ShooterSetSpeed;

import frc.robot.RobotContainer;
import frc.robot.subsystems.BallHandler.State;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoRightTwoBall extends SequentialCommandGroup {
  /** Creates a new AutoRightTwoBall. */
  public AutoRightTwoBall() {
    addCommands(
      new HoodHome(),
      new BallHandlerSetState(State.kFillTo0),
      parallel(
        new DriveSetGyro(90.0),
        new WaitCommand(.75)//We need to wait for two reasons, the gyro takes time too set the value, and the harvester needs to come down
      ),
      new ShooterSetSpeed(9100),
      parallel(
        new DriveFollowTrajectory("DriveToRightBall"),
        new HoodToPosition(9.9)//while driving to the ball, set the hood
      ),
      race(
        new WaitUntilCommand(RobotContainer.ballHandler::isBall0).withTimeout(4.0),
        sequence(
          new DriveTurnToAngleInRad(Math.toRadians(75.31)).withTimeout(1.0),
          new WaitCommand(.5),
          new DriveTurnToAngleInRad(Math.toRadians(85.31)).withTimeout(2.0),
          new WaitCommand(.5)
        )
      ),
      new DriveTurnToAngleInRad(Math.toRadians(80.31)).withTimeout(1.5),
      parallel(
        new ShooterSetSpeed(RobotContainer.limeLight::getShooterHighSpeed,true).withTimeout(1.0),
        new HoodToPosition(RobotContainer.limeLight::getHoodHighAngle,true).withTimeout(1.0)
      ),
      new BallHandlerShootProgT(0.0),
      new StopShooterHandlerHood(),
      new BallHandlerSetState(State.kOff),
      new WaitCommand(1.0),
      new DriveStraightBack(0.25)
    );
  }
}
