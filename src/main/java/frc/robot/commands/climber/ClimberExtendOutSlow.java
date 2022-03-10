// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ClimberExtendOutSlow extends CommandBase {
  double distance;
  /** Creates a new ClimberExtendOutSlow. */
  public ClimberExtendOutSlow(double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.climber);
    this.distance = distance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(distance-RobotContainer.climber.getExtendoLeftEncPos() <= 0.0){
      RobotContainer.climber.stopExtendoLeftArm();
    }else{
      RobotContainer.climber.extendoArmLeftOut();
    }
    if(distance-RobotContainer.climber.getExtendoRightEncPos() <= 0.0){
      RobotContainer.climber.stopExtendoRightArm();
    }else{
      RobotContainer.climber.extendoArmRightOut();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.climber.stopBothMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (distance-RobotContainer.climber.getExtendoLeftEncPos() <= 0.0)&&(distance-RobotContainer.climber.getExtendoRightEncPos() <= 0.0);
  }
}
