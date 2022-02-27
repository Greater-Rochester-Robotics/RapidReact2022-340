// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hood;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class HoodHome extends CommandBase {
  boolean overrideBeenHomed = false;//Used to skip homing if hood has been homed since robot boot
  /** Creates a new HoodHome. */
  public HoodHome() {
    this(false);
  }

  public HoodHome(boolean overrideBeenHomed){
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.hood);
    this.overrideBeenHomed = overrideBeenHomed;
  }

  public void initialize(){
    //double check that the hood motor hasn't been power cycled
    // RobotContainer.hood.checkForPowerCycle();//TODO:Test if this works before enabling code
  }

  @Override
  public void end(boolean interrupted){
    //stop the motor when we finish
    RobotContainer.hood.stopMotor();
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.hood.homePosition() 
      || (!overrideBeenHomed && RobotContainer.hood.hasBeenHomed());
  }
}
