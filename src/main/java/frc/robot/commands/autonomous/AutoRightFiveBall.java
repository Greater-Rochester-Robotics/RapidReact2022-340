// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoRightFiveBall extends SequentialCommandGroup {
  /** Creates a new AutoRightFiveBall. */
  public AutoRightFiveBall() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //FenderHighgoal
      //start intaking
      //drive shortpath to first(Right) ball
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
