// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LimeLightPowerCycle extends CommandBase {
  Timer time;
  public LimeLightPowerCycle() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.limeLight);
    time = new Timer();
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.limeLight.setLimeLightPower(false);
    time.reset();
    time.start();
  }

  public void end(boolean interrupted){
    RobotContainer.limeLight.setLimeLightPower(true);
  }

  public boolean isFinnished(){
    return time.hasElapsed(.5);
  }
}
