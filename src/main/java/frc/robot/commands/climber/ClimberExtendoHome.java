// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/**
 * Command to return the extending arms to the bottom position and zeroizes
 * the encoders at the 
 */
public class ClimberExtendoHome extends CommandBase {
  Timer timer = new Timer();
  
  public ClimberExtendoHome() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(isRightAtBottom()){
      RobotContainer.climber.stopExtendoRightArm();
    }
    else{
      RobotContainer.climber.extendoArmRightIn();
    }

    if(isLeftAtBottom()){
      RobotContainer.climber.stopExtendoLeftArm();
    }else{
      RobotContainer.climber.extendoArmLeftIn();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.climber.stopExtendoRightArm();
    RobotContainer.climber.stopExtendoLeftArm();

    if(!interrupted){
      RobotContainer.climber.setExtendoRightEnc(0.0);
      RobotContainer.climber.setExtendoLeftEnc(0.0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isRightAtBottom() && isLeftAtBottom();
  }

  boolean isRightAtBottom(){
    return RobotContainer.climber.getExtendoRightSwitch() ||
    (timer.hasElapsed(0.5) && RobotContainer.climber.getExtendoRightCurrent() > Constants.EXTENDO_HOMING_CURRENT) ||
    (timer.hasElapsed(0.5) && Math.abs(RobotContainer.climber.getExtendoRightEncVel()) < 0.1);
  }
  boolean isLeftAtBottom(){
    return RobotContainer.climber.getExtendoLeftSwitch() ||
    (timer.hasElapsed(0.5) && RobotContainer.climber.getExtendoLeftCurrent() > Constants.EXTENDO_HOMING_CURRENT) ||
    (timer.hasElapsed(0.5) && Math.abs(RobotContainer.climber.getExtendoLeftEncVel()) < 0.1);
  }
}
