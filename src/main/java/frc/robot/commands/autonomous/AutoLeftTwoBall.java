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
import frc.robot.commands.drive.util.DriveTurnToAngle;
import frc.robot.commands.hood.HoodHome;
import frc.robot.commands.hood.HoodToPosition;
import frc.robot.commands.shooter.ShooterSetSpeed;

import frc.robot.RobotContainer;
import frc.robot.subsystems.BallHandler.State;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoLeftTwoBall extends SequentialCommandGroup {
  /** Creates a new AutoLeftTwoBall. */
  public AutoLeftTwoBall() {
    addCommands(
      new HoodHome(),
      new BallHandlerSetState(State.kFillTo0),
      parallel(
        new DriveSetGyro(-45.0),//set the gyro to a specific angle that we start the robot in
        new WaitCommand(.75)//We need to wait for two reasons, the gyro takes time too set the value, and the harvester needs to come down
      ),
      new ShooterSetSpeed(9200),//set the shooter wheel speed, but dont wait to get to speed
      parallel(
        new DriveFollowTrajectory("DriveToLeftBall"),
        new HoodToPosition(9.9)//while driving to the ball, set the hood
      ),
      race(
        deadline(
          new WaitUntilCommand(RobotContainer.ballHandler::isBall0).withTimeout(4.0),//Wait for ball0 switch, race with a wiggle
          new ShooterSetSpeed(RobotContainer.limeLight::getShooterHighSpeed,true),
          new HoodToPosition(RobotContainer.limeLight::getHoodHighAngle,true)
        ),
        sequence(
          new DriveTurnToAngle(Math.toRadians(-26.43)).withTimeout(1.0),//wiggle 5 degrees clockwise
          new WaitCommand(.5),//wait for a moment 
          new DriveTurnToAngle(Math.toRadians(-36.43)).withTimeout(2.0),//wiggle 5 degrees counter-clockwise(total of 10 deg)
          new WaitCommand(.5)//wait for a moment
        )
      ),
      new DriveTurnToAngle(Math.toRadians(-31.43)).withTimeout(1.5),//make sure we return to start rotation
      new BallHandlerShootProgT(0.0),
      new StopShooterHandlerHood(),
      new DriveStraightBack(0.20)
    );
  }
}
