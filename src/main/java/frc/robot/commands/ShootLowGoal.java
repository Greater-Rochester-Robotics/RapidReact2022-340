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
import frc.robot.commands.shooter.ShooterSetSpeed;
import frc.robot.commands.shooter.ShooterStop;

public class ShootLowGoal extends SequentialCommandGroup {
  /** Creates a new ShootHighGoal. */
  public ShootLowGoal(double timeBewtweenBalls) {
    addCommands(
      new HoodHome(),//Home the hood, but should fall through if hood has been homed
      parallel(
        new ShooterSetSpeed(RobotContainer.limeLight::getShooterLowSpeed,true).withTimeout(2),//Set the Shooter to low goal speed given the limelight distance
        new HoodToPosition(RobotContainer.limeLight::getHoodLowAngle,true)//set the hood to the angle for the low goal given the limelight distance
      ),
      new BallHandlerShootProgT(timeBewtweenBalls),//shoot the balls
      new ShooterStop()//stop the shooter if we finish
    );
  }
}
