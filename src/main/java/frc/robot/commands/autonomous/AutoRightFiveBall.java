// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.ShootHighFender;
import frc.robot.commands.ShootHighFenderWithDriveBack;
import frc.robot.commands.ShootHighGoal;
import frc.robot.commands.ballhandler.BallHandlerIntakeOut;
import frc.robot.commands.ballhandler.BallHandlerSetState;
import frc.robot.commands.ballhandler.BallHandlerShootProgT;
import frc.robot.commands.drive.auto.DriveFollowTrajectory;
import frc.robot.commands.drive.auto.DriveTurnToTarget;
import frc.robot.commands.drive.util.DriveSetGyro;
import frc.robot.commands.hood.HoodHome;
import frc.robot.commands.hood.HoodToPosition;
import frc.robot.commands.shooter.ShooterPrepShot;
import frc.robot.commands.shooter.ShooterSetSpeed;
import frc.robot.subsystems.BallHandler.State;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoRightFiveBall extends SequentialCommandGroup {
  /** Creates a new AutoRightFiveBall. not written*/
  @Deprecated
  public AutoRightFiveBall() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //FenderHighgoal
      // new ShootHighFender(),
      //start intaking
      new HoodHome(),
      new BallHandlerIntakeOut(),
      parallel(
        new DriveSetGyro(0.0)//,
      ),
      new DriveTurnToTarget(),
      new ShooterPrepShot(),
      new ShootHighGoal(0.0).withTimeout(1.0),
      new BallHandlerSetState(State.kFillTo1),
      new DriveFollowTrajectory("DriveToRightBall"),
      new WaitUntilCommand(RobotContainer.ballHandler::isBall1).withTimeout(2.0),
      new DriveFollowTrajectory("DriveFromRightBallToMidBall"),
      new WaitUntilCommand(RobotContainer.ballHandler::isBall0).withTimeout(2.0),
      new BallHandlerSetState(State.kOff),
      new DriveFollowTrajectory("DriveMidBallToRotate"),
      new ShooterPrepShot(),
      new DriveTurnToTarget()      
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
