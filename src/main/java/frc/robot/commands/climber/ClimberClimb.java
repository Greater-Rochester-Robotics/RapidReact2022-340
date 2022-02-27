// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimberClimb extends SequentialCommandGroup {
  /** Creates a new ClimberClimb. */
  public ClimberClimb() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ClimberExtendoHome().withName("StartAndHomeClimber"),
      new ClimberExtendoToPosition(Constants.CLIMBER_TOP_POSITION).withName("ExtendToSecondBar"),
      new WaitUntilCommand(RobotContainer.climberButton::get).withName("WaitToPullUp"),
      sequence(
        new ClimberExtendoToPosition(Constants.CLIMBER_BOTTOM_POSITION,true),
        new ClimberExtendoForceToBottom(.5)
      ).withName("PullUpToSecondBar"),
      new WaitUntilCommand(RobotContainer.climberButton::get).withName("WaitToTiltRobot"),
      new ClimberTiltOut().withName("TiltToThirdBar"),
      new ClimberExtendoToPosition(Constants.CLIMBER_RELEASE_POSITION).withName("ReleaseFromSecondBar"),
      new WaitUntilCommand(RobotContainer.climberButton::get).withName("WaitToExtendToThirdBar"),
      new ClimberExtendoToPosition(Constants.CLIMBER_TOP_POSITION).withName("ExtendToThirdBar"),
      new WaitUntilCommand(RobotContainer.climberButton::get).withName("WaitUntilThirdBarReached"),
      new ClimberTiltIn().withName("TiltToNormal"),
      new WaitCommand(1.0).withName("PauseForTilt"),
      new WaitUntilCommand(RobotContainer.climberButton::get).withName("WaitToStartClimbToThirdBar"),
      new ClimberExtendoToPosition(Constants.CLIMBER_LIFT_POSITION,true),
      new WaitUntilCommand(RobotContainer.climberButton::get).withName("WaitToClimbToThirdBar"),
      sequence(
        new ClimberExtendoToPosition(Constants.CLIMBER_BOTTOM_POSITION,true),
        new ClimberExtendoForceToBottom(.5)
      ).withName("ClimbToThirdBar"),
      new WaitUntilCommand(RobotContainer.climberButton::get).withName("WaitToTiltRobot"),
      new ClimberTiltOut().withName("TiltToFourthBar"),
      new ClimberExtendoToPosition(Constants.CLIMBER_RELEASE_POSITION).withName("ReleaseFromThirdBar"),
      new WaitUntilCommand(RobotContainer.climberButton::get).withName("WaitToExtendToFourthBar"),
      new ClimberExtendoToPosition(Constants.CLIMBER_TOP_POSITION).withName("ReachToFourthBar"),
      new WaitUntilCommand(RobotContainer.climberButton::get).withName("WaitToTiltToNormal"),
      new ClimberTiltIn().withName("TiltToNormal"),
      new WaitCommand(1.0).withName("PauseToTilt"),
      new WaitUntilCommand(RobotContainer.climberButton::get).withName("WaitToPullUp"),
      new ClimberExtendoToPosition(Constants.CLIMBER_LIFT_POSITION).withName("PullUpToFourthPosition")
    );
  }

}
