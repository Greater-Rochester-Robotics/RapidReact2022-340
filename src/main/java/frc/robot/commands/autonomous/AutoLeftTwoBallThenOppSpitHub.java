// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.StopShooterHandlerHood;
import frc.robot.commands.ballhandler.BallHandlerIntakeOut;
import frc.robot.commands.ballhandler.BallHandlerRejectOppColor;
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

public class AutoLeftTwoBallThenOppSpitHub extends SequentialCommandGroup {
  /** Creates a new AutoLeftTwoBallThenOppToss. tested*/
  public AutoLeftTwoBallThenOppSpitHub(){
    addCommands(
      new HoodHome(),
      new BallHandlerSetState(State.kFillTo1),
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
      new DriveTurnToTarget(),
      parallel(
        new ShooterSetSpeed(RobotContainer.limeLight::getShooterHighSpeed,true).withTimeout(1.5),
        new HoodToPosition(RobotContainer.limeLight::getHoodHighAngle,true).withTimeout(1.5)
      ),
      new BallHandlerShootProgT(0.0),
      new BallHandlerRejectOppColor(false),//set intake to not reject Opp color
      new StopShooterHandlerHood(),
      new BallHandlerSetState(State.kFillTo0),
      sequence(
        new DriveTurnToAngle(Math.toRadians(-135)).withTimeout(2.5),
        new WaitCommand(.5),
        new DriveFollowTrajectory("DriveLeftToOppBallShoot")//drive to opponent's ball
      ),
      new WaitUntilCommand(() -> RobotContainer.ballHandler.isBall0()).withTimeout(2.0),//when ball is in robot stop prep and shoot
      new BallHandlerRejectOppColor(),//set intake to reject Opp color
      new BallHandlerIntakeOut(),
      new DriveTurnToAngle(Math.toRadians(-235)),
      new BallHandlerSetState(State.kSpitLow0)
      // new BallHandlerSetState(State.kOff),
      // new StopShooterHandlerHood()//stop shooter (and ballhandler and hood if they aren't already)
    );
  }
}
