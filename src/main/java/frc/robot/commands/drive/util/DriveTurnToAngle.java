// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.util;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;
import frc.robot.Constants;

public class DriveTurnToAngle extends CommandBase {
  private double angle = 0;
  private int onTargetCount;

  /** Creates a new DriveTurnToAngle. This command 
   * is used in tuning PID for rotation and in 
   * autonomous to make a specific turn. 
   * 
   * @param angle angle in radians 
   */
  public DriveTurnToAngle(double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.swerveDrive);

    this.angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    onTargetCount = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = RobotContainer.swerveDrive.getRobotRotationPIDOut(angle);
    // System.out.println("pid output"+output);
    RobotContainer.swerveDrive.driveRobotCentric(0, 0, output, false, true);
    if(Math.abs(angle - RobotContainer.swerveDrive.getGyroInRad()) < .03){
      onTargetCount++;
    }else{
      onTargetCount = 0;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.swerveDrive.stopAllModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return onTargetCount >= 10; //TODO:fix the end condition
  }
}
