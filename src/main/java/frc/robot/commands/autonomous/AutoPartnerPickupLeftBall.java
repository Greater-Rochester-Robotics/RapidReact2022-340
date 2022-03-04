// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPartnerPickupLeftBall extends SequentialCommandGroup {
  /** Creates a new AutoPartnerPickupLeftBall. */
  public AutoPartnerPickupLeftBall() {
    addCommands(
      //Home the hood
      //set the hood and shooter.(parallel with next too)
      //have a simple wait command, we may need to wait for the ball to be place
      //use kFill0
      //wait for the ball0 button(with timeout)
      //run the kOff, don't want to rip off the harvester if ball not found
      //WAIT???? for harvester
      //turn to the target, with a timeout
      //Shoot(this is a ball handler command, already at speed and hood pos)
      //kFill1 and shooterspeed and hoodset(paral)
      //run path
      //wait for the ball in 1
      //shoot the ball
      //stop the shooter, handler, hood
      //back up some more
    );
  }
}
