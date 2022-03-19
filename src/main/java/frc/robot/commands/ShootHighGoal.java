// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.ballhandler.BallHandlerShootProgT;
import frc.robot.commands.hood.HoodHome;
import frc.robot.commands.hood.HoodToPosition;
import frc.robot.commands.shooter.ShooterPrepShot;
import frc.robot.commands.shooter.ShooterSetSpeed;
import frc.robot.commands.shooter.ShooterStop;

public class ShootHighGoal extends SequentialCommandGroup {
  /**
   * 
   */
  public ShootHighGoal(){
    this(0.0);
  }
  /** Creates a new ShootHighGoal. */
  public ShootHighGoal(double timeBewtweenBalls) {
    addCommands(
      new HoodHome(),
      new ShooterPrepShot(),
      parallel(
        new ShooterSetSpeed(RobotContainer.limeLight::getShooterHighSpeed,true).withTimeout(2.0),
        new HoodToPosition(RobotContainer.limeLight::getHoodHighAngle,true).withTimeout(2.0)//,
        // new WaitCommand(2.0)
      ),
      // new WaitUntilCommand(RobotContainer.shooter::isAtSpeed),//this should fall through, left for options
      new BallHandlerShootProgT(timeBewtweenBalls),
      new ShooterStop()
    );
  }
}
