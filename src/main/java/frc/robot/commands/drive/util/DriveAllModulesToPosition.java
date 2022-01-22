// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.util;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;

/**
 * This command allows all the modules to be rotated to a specific position.
 */
public class DriveAllModulesToPosition extends CommandBase {
  double angle;
  boolean[] finishedArray = new boolean[4];
  /** Creates a new DriveAllModulesToPosition. */
  public DriveAllModulesToPosition(double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.angle = angle;
    addRequirements(RobotContainer.swerveDrive);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] angles = RobotContainer.swerveDrive.getAllAbsModuleAngles();
    double targetAngle = angle - RobotContainer.swerveDrive.getGyroInRad();
    
    for(int i=0;i<4;i++ ){
      RobotContainer.swerveDrive.driveOneModule(i, 0, targetAngle, false);
      finishedArray[i]= .17 > Math.abs(angles[i] -  Math.toDegrees(targetAngle));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finishedArray[0]&&finishedArray[1]&&finishedArray[2]&&finishedArray[3];
  }
}
