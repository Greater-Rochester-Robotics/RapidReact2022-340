// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hood;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class HoodHome extends CommandBase {
  boolean overrideBeenHomed = false;//Used to skip homing if hood has been homed since robot boot
  
  /**
   * A command that homes the hood by slowly driving 
   * the hood toward the limit switch. It stops once 
   * the limit switch is pressed. If the hood has been 
   * homed since the robot has been booted, it will 
   * not home again.
   */
  public HoodHome() {
    this(false);
  }

  /**
   *  A command that homes the hood by slowly driving 
   * the hood toward the limit switch. It stops once 
   * the limit switch is pressed. If the hood has been 
   * homed since the robot has been booted, it will 
   * not home if overrideBeenHomed is false. If true 
   * is passed, then the hood will be homed again.
   * 
   * @param overrideBeenHomed
   */
  public HoodHome(boolean overrideBeenHomed){
    addRequirements(RobotContainer.hood);
    this.overrideBeenHomed = overrideBeenHomed;
  }

  public void initialize(){
    //double check that the hood motor hasn't been power cycled- on hold
    // RobotContainer.hood.checkForPowerCycle();//This doesn't seem to work, don't use
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
