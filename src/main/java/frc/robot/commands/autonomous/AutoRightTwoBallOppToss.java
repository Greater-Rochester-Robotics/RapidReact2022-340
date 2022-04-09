// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import frc.robot.commands.StopShooterHandlerHood;
import frc.robot.commands.ballhandler.BallHandlerIntakeOut;
import frc.robot.commands.ballhandler.BallHandlerRejectOppColor;
import frc.robot.commands.ballhandler.BallHandlerSetState;
import frc.robot.commands.ballhandler.BallHandlerShootProgT;
import frc.robot.commands.drive.auto.DriveFollowTrajectory;
import frc.robot.commands.drive.auto.DriveStraightBack;
import frc.robot.commands.drive.util.DriveSetGyro;
import frc.robot.commands.drive.util.DriveTurnToAngle;
import frc.robot.commands.hood.HoodHome;
import frc.robot.commands.hood.HoodToPosition;
import frc.robot.commands.shooter.ShooterSetSpeed;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BallHandler.State;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoRightTwoBallOppToss extends SequentialCommandGroup {
  /** Creates a new AutoRightTwoBall. this won't work because mechanical intake limitations.*/
  @Deprecated
  public AutoRightTwoBallOppToss() {
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
          new DriveTurnToAngle(Math.toRadians(75.31)).withTimeout(1.0),
          new WaitCommand(.5),
          new DriveTurnToAngle(Math.toRadians(85.31)).withTimeout(2.0),
          new WaitCommand(.5)
        )
      ),
      new DriveTurnToAngle(Math.toRadians(80.31)).withTimeout(1.5),
      parallel(
        new ShooterSetSpeed(RobotContainer.limeLight::getShooterHighSpeed,true).withTimeout(1.0),
        new HoodToPosition(RobotContainer.limeLight::getHoodHighAngle,true).withTimeout(1.0)
      ),
      new BallHandlerShootProgT(0.0),
      new BallHandlerRejectOppColor(false),//set intake to not reject Opp color
      parallel(
        sequence(
          // new DriveTurnToAngle(Math.toRadians(170.31)).withTimeout(2.5),
          new BallHandlerSetState(State.kFillTo1),
          // new WaitCommand(.5),
          new DriveFollowTrajectory("DriveRightBallToOppBall")//drive to opponent's ball
        ),
        new ShooterSetSpeed(8000, true).withTimeout(2),//set the shooter to fender speed
        new HoodToPosition(22.0)//set the hood to maximum position
      ),
      new WaitUntilCommand(() -> RobotContainer.ballHandler.isBall0() || RobotContainer.ballHandler.isBall1()).withTimeout(2.0),//when ball is in robot stop prep and shoot
      new DriveTurnToAngle(Math.toRadians(135)).withTimeout(1.5),
      new BallHandlerRejectOppColor(),//set intake to reject Opp color
      new BallHandlerSetState(State.kOff),
      new BallHandlerShootProgT(0.0),//shoot the ball
      new StopShooterHandlerHood()//stop shooter (and ballhandler and hood if they aren't already)
    );
  }
}
