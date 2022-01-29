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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimberClimb extends SequentialCommandGroup {
  /** Creates a new ClimberClimb. */
  public ClimberClimb() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ClimberExtendoHome(),
      new ClimberExtendoToPosition(Constants.CLIMBER_TOP_POSITION),
      new WaitUntilCommand(new ContinueButton()),
      new ClimberExtendoToPosition(Constants.CLIMBER_BOTTOM_POSITION),
      new WaitUntilCommand(new ContinueButton()),
      parallel(
        new ClimberTiltOut(),
        new ClimberExtendoToPosition(Constants.CLIMBER_TOP_POSITION)
      ),
      new WaitUntilCommand(new ContinueButton()),
      new ClimberTiltIn(),
      new WaitCommand(2.0),
      new WaitUntilCommand(new ContinueButton()),
      new ClimberExtendoToPosition(Constants.CLIMBER_BOTTOM_POSITION),
      new WaitUntilCommand(new ContinueButton()),
      parallel(
        new ClimberTiltOut(),
        new ClimberExtendoToPosition(Constants.CLIMBER_TOP_POSITION)
      ),
      new WaitUntilCommand(new ContinueButton()),
      new ClimberTiltIn(),
      new WaitCommand(2.0),
      new WaitUntilCommand(new ContinueButton()),
      new ClimberExtendoToPosition(Constants.CLIMBER_MIDDLE_POSITION)
    );
  }

  public class ContinueButton implements BooleanSupplier{
    public boolean getAsBoolean(){
      return Robot.robotContainer.getCoDriverButton(1);
    }
  }
}
