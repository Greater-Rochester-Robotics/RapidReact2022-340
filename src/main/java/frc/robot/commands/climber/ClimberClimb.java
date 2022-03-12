// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.SendableCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimberClimb extends SendableCommandGroup {
  /** Creates a new ClimberClimb. */
  public ClimberClimb() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ClimberExtendoHome().withName("StartAndHomeClimber"),
      new ClimberExtendOutSlow(Constants.CLIMBER_TOP_POSITION).withName("ExtendToSecondBar"),
      new WaitUntilCommand(RobotContainer.climberButton::get).withName("WAIT-ToPullUp"),
      sequence(
        // race(
          new ClimberExtendoToPosition(Constants.CLIMBER_BOTTOM_POSITION,true).withTimeout(0.8),
          //   new WaitUntilCommand(RobotContainer.climber::getExtendoBothSwitch)
          // ),
          new ClimberExtendoForceToBottom(.5)
      ).withName("PullUpToSecondBar"),
      new WaitUntilCommand(RobotContainer.climberButton.negate()::get).withName("WAIT-ReleaseButton"),
      new WaitUntilCommand(RobotContainer.climberButton::get).withName("WAIT-ReleaseFromSecondBar"),
      new ClimberExtendoToPosition(Constants.CLIMBER_RELEASE_POSITION).withName("ReleaseFromSecondBar"),
      new WaitUntilCommand(RobotContainer.climberButton.negate()::get).withName("WAIT-ReleaseButton"),
      new WaitUntilCommand(RobotContainer.climberButton::get).withName("WAIT-ToReleaseTiltRobot"),
      new ClimberTiltOut().withName("TiltToThirdBar"),
      new WaitUntilCommand(RobotContainer.climberButton.negate()::get).withName("WAIT-ReleaseButton"),
      new WaitUntilCommand(RobotContainer.climberButton::get).withName("WAIT-ToExtendToThirdBar"),
      new WaitUntilCommand(RobotContainer.climberButton.negate()::get).withName("WAIT-ReleaseButton"),
      new WaitUntilCommand(RobotContainer.climberButton::get).withName("WAIT-ToExtendToThirdBar"),
      new ClimberExtendOutSlow(Constants.CLIMBER_TOP_POSITION).withName("ExtendToThirdBar"),
      new WaitUntilCommand(RobotContainer.climberButton::get).withName("WAIT-UntilTiltInThirdBar"),
      new ClimberTiltIn().withName("TiltToNormal"),
      new WaitUntilCommand(RobotContainer.climberButton.negate()::get).withName("WATI-ReleaseButton"),
      new WaitUntilCommand(RobotContainer.climberButton::get).withName("WAIT-ToLiftOffSecondBar"),
      new ClimberExtendoToPosition(Constants.CLIMBER_LIFT_POSITION,true).withName("LiftOffSecondBar"),
      new WaitUntilCommand(RobotContainer.climberButton.negate()::get).withName("WATI-ReleaseButton"),
      new WaitUntilCommand(RobotContainer.climberButton::get).withName("WAIT-ToClimbToThirdBar"),
      sequence(
       //  race(
          new ClimberExtendoToPosition(Constants.CLIMBER_BOTTOM_POSITION,true).withTimeout(0.5),
       //   new WaitUntilCommand(RobotContainer.climber::getExtendoBothSwitch)
       // ),
       new ClimberExtendoForceToBottom(.5)
      ).withName("ClimbToThirdBar"),
      new WaitUntilCommand(RobotContainer.climberButton::get).withName("WAIT-ToReleaseFromThirdBar"),
      new ClimberExtendOutSlow(Constants.CLIMBER_TOP_POSITION).withName("ReleaseFromThirdBar"),
      new WaitUntilCommand(RobotContainer.climberButton.negate()::get).withName("WATI-ReleaseButton"),
      new WaitUntilCommand(RobotContainer.climberButton::get).withName("WAIT-ToTiltToFourth"),
      new ClimberTiltOut().withName("TiltToFourthBar"),
      new WaitCommand(1.0).withName("PauseToTilt"),
      new WaitUntilCommand(RobotContainer.climberButton.negate()::get).withName("WATI-ReleaseButton"),
      // new WaitUntilCommand(RobotContainer.climberButton::get).withName("WAIT-ToExtendToFouthBar"),
      // new ClimberExtendoToPosition(Constants.CLIMBER_TOP_POSITION).withName("ExtendToFouthBar"),
      // new WaitUntilCommand(RobotContainer.climberButton::get).withName("WAIT-ToTiltToNormal"),
      // new ClimberTiltIn().withName("TiltToNormal"),
      // new WaitCommand(1.0).withName("PauseToTilt"),
      new WaitUntilCommand(RobotContainer.climberButton::get).withName("WAIT-ToLiftOffThirdbar"),
      new ClimberExtendoToPosition(Constants.CLIMBER_LIFT_POSITION).withName("LiftOffThirdBar"),
      new WaitUntilCommand(RobotContainer.climberButton.negate()::get).withName("WATI-ReleaseButton"),
      new WaitUntilCommand(RobotContainer.climberButton::get).withName("WAIT-ToClimbToFourthBar"),
      new ClimberExtendoToPosition(Constants.CLIMBER_BOTTOM_POSITION,true).withTimeout(0.5),
      new WaitUntilCommand(RobotContainer.climberButton.negate()::get).withName("WATI-ReleaseButton"),
      new WaitUntilCommand(RobotContainer.climberButton::get).withName("WAIT-ToTiltToNormal"),
      new ClimberTiltIn().withName("TiltToNormal"),
      new WaitCommand(1.0).withName("PauseToTilt"),
      new ClimberExtendoStop() //,
      // new WaitUntilCommand(RobotContainer.climberButton::get).withName("WAIT-ToClimbToFourthBar"),
      // // race(
      //   new ClimberExtendoToPosition(Constants.CLIMBER_BOTTOM_POSITION,true),
      //   // new WaitUntilCommand(RobotContainer.climber::getExtendoBothSwitch)
      // // ),
      // new WaitUntilCommand(RobotContainer.climberButton::get).withName("WAIT-ToClimbToFourthBar"),
      // // race(
      //   new ClimberExtendoToPosition(Constants.CLIMBER_BOTTOM_POSITION,true) //,
        // new WaitUntilCommand(RobotContainer.climber::getExtendoBothSwitch)
      // )
    );
  }

}
