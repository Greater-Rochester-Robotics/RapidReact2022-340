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
      new ClimberExtendoHome().withName("StartClimber"),
      new ClimberExtendoToPosition(Constants.CLIMBER_TOP_POSITION).withName("ExtendToSecondBar"),
      new WaitUntilCommand(Robot.robotContainer.getClimberButton()::get).withName("WaitToPullUp"),
      new ClimberExtendoToPosition(Constants.CLIMBER_BOTTOM_POSITION).withName("PullUpToSecondBar"),
      new WaitUntilCommand(Robot.robotContainer.getClimberButton()::get).withName("WaitToTiltRobot"),
      parallel(
        new ClimberTiltOut(),
        new ClimberExtendoToPosition(Constants.CLIMBER_TOP_POSITION)
      ).withName("TiltAndExtendToThirdBar"),
      new WaitUntilCommand(Robot.robotContainer.getClimberButton()::get).withName("WaitUntilThirdBarReached"),
      new ClimberTiltIn().withName("TiltToNormal"),
      new WaitCommand(2.0).withName("PauseToStopSwinging"),
      new WaitUntilCommand(Robot.robotContainer.getClimberButton()::get).withName("WaitToStartClimbToFourthBar"),
      new ClimberExtendoToPosition(Constants.CLIMBER_BOTTOM_POSITION).withName("ExtendToFourthBar"),
      new WaitUntilCommand(Robot.robotContainer.getClimberButton()::get).withName("WaitToTiltRobot"),
      parallel(
        new ClimberTiltOut(),
        new ClimberExtendoToPosition(Constants.CLIMBER_TOP_POSITION)
      ).withName("TiltAndSwingToFourthBar"),
      new WaitUntilCommand(Robot.robotContainer.getClimberButton()::get).withName("WaitToTiltToNormal"),
      new ClimberTiltIn().withName("TiltToNormal"),
      new WaitCommand(2.0).withName("PauseToStopSwinging"),
      new WaitUntilCommand(Robot.robotContainer.getClimberButton()::get).withName("WaitToPullUp"),
      new ClimberExtendoToPosition(Constants.CLIMBER_MIDDLE_POSITION).withName("PullUpToFinalPosition")
    );
  }

}
