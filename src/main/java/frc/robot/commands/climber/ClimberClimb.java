// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SendableCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ClimberClimb extends SendableCommandGroup {
  /** Creates a new ClimberClimb. */
  public ClimberClimb() {
    addCommands(
      /* Home Climber */
      new ClimberExtendoHome().withName("StartAndHomeClimber"),

      /* Climb To Second Bar */
      new ClimberDampeningOut().withName("DampeningBarOut"),
      new ClimberExtendOutSlow(Constants.CLIMBER_TOP_POSITION).withName("ExtendToSecondBar"),
      new WaitUntilCommand(RobotContainer.climberManButton.negate()::get).withName("WAIT-ReleaseAButton"),
      new WaitUntilCommand(RobotContainer.climberManButton::get).withName("WAIT-ToPullUp"),
      sequence(
          new ClimberExtendoToPosition(Constants.CLIMBER_BOTTOM_POSITION,true).withTimeout(0.8),
          new ClimberExtendoForceToBottom(.5)
      ).withName("PullUpToSecondBar"),
      new WaitUntilCommand(RobotContainer.climberManButton.negate()::get).withName("WAIT-ReleaseAButton"),

      /* Release Second Bar */
      new WaitUntilCommand(RobotContainer.climberManButton::get).withName("WAIT-ReleaseFromSecondBar"),
      new ClimberExtendoToPosition(Constants.CLIMBER_RELEASE_POSITION).withName("ReleaseFromSecondBar"),
      new WaitUntilCommand(RobotContainer.climberManButton.negate()::get).withName("WAIT-ReleaseAButton"),
      new WaitUntilCommand(RobotContainer.climberManButton::get).withName("WAIT-ToReleaseTiltRobot"),
      new ClimberTiltOut().withName("TiltToThirdBar"),
      new WaitCommand(0.5).withName("PauseToTilt"),
      new WaitUntilCommand(RobotContainer.climberManButton.negate()::get).withName("WAIT-ReleaseAButton"),

      /* Climb To Third Bar */
      race(
        new WaitUntilCommand(RobotContainer.climberManButton::get),
        new WaitUntilCommand(() -> RobotContainer.swerveDrive.isTiltedBack() || RobotContainer.climberAutoButton.get())
      ).withName("WAIT-ToExtendToThirdBar"),
      new ClimberExtendOutSlow(Constants.CLIMBER_TOP_POSITION).withName("ExtendToThirdBar"),
      new WaitUntilCommand(RobotContainer.climberManButton.negate()::get).withName("WAIT-ReleaseAButton"),
      new WaitUntilCommand(RobotContainer.climberManButton::get).withName("WAIT-UntilTiltInThirdBar"),
      new ClimberTiltIn().withName("TiltToNormal"),
      
      new WaitCommand(1.0).withName("PauseToTilt"),
      
      new WaitUntilCommand(RobotContainer.climberManButton.negate()::get).withName("WAIT-ReleaseAButton"),
      race(
        new WaitUntilCommand(RobotContainer.climberManButton::get),
        new WaitUntilCommand(()-> RobotContainer.swerveDrive.isTiltedUp() || RobotContainer.climberAutoButton.get())
      ).withName("WAIT-ToLiftOffSecondBar"),
      new ClimberExtendoToPosition(Constants.CLIMBER_LIFT_POSITION,true).withName("LiftOffSecondBar"),
      new WaitUntilCommand(RobotContainer.climberManButton.negate()::get).withName("WAIT-ReleaseAButton"),
      race(
        new WaitUntilCommand(RobotContainer.climberManButton::get),
        new WaitUntilCommand(()-> RobotContainer.swerveDrive.isLiftToThirdReady() || RobotContainer.climberAutoButton.get())
      ).withName("WAIT-ToClimbToThirdBar"),
      sequence(
        new ClimberExtendoToPosition(Constants.CLIMBER_BOTTOM_POSITION,true).withTimeout(0.5),
        new ClimberExtendoForceToBottom(.5)
      ).withName("ClimbToThirdBar"),
      new WaitUntilCommand(RobotContainer.climberManButton.negate()::get).withName("WAIT-ReleaseAButton"),

      /* Release Third Bar */  
      new WaitUntilCommand(RobotContainer.climberManButton::get).withName("WAIT-ToReleaseFromThirdBar"),
      new ClimberExtendOutSlow(Constants.CLIMBER_TOP_POSITION).withName("ReleaseFromThirdBar"),
      new ClimberDampeningIn().withName("DampeningBarIn"),
      new WaitUntilCommand(RobotContainer.climberManButton.negate()::get).withName("WAIT-ReleaseAButton"),
      new WaitUntilCommand(RobotContainer.climberManButton::get).withName("WAIT-ToTiltToFourth"),
      new ClimberTiltOut().withName("TiltToFourthBar"),
      // race(
        new WaitCommand(1.0).withName("PauseToTilt"),//TODO: Race? with a detection of a tilt of 38ish degrees
        // new WaitUntilCommand(RobotContainer.swerveDrive::isTiltedBack)
      // ),
      new WaitUntilCommand(RobotContainer.climberManButton.negate()::get).withName("WAIT-ReleaseAButton"),

      /* Climb To Fourth Bar */
      new WaitUntilCommand(RobotContainer.climberManButton::get).withName("WAIT-ToLiftOffThirdbar"),
      new ClimberExtendoToPosition(Constants.CLIMBER_LIFT_POSITION).withName("LiftOffThirdBar"),
      new WaitUntilCommand(RobotContainer.climberManButton.negate()::get).withName("WAIT-ReleaseAButton"),
      new WaitUntilCommand(RobotContainer.climberManButton::get).withName("WAIT-ToClimbToFourthBar"),
      new ClimberExtendoToPosition(Constants.CLIMBER_BOTTOM_POSITION,true).withTimeout(0.5),
      new ClimberTiltIn().withName("TiltToNormal"),
      new WaitCommand(1.0).withName("PauseToTilt"),
      new ClimberExtendoStop()
    );
  }

}
