// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

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
    RobotContainer.swerveDrive.driveRobotCentric(0, 0, RobotContainer.swerveDrive.getRobotRotationPIDOut(angle), false);
    boolean isOnTarget = false;//TODO: we need to set this in a method
    Timer timer = new Timer();
    double rotateToAngle = 0.0;
    if(timer.hasElapsed(.2) && !isOnTarget){
      Translation2d target = RobotContainer.swerveDrive.driveOdometry.getPoseMeters().getTranslation().minus(Constants.FIELD_CENTER);
      double desiredAngle = Math.atan2(target.getY(), target.getX());
      double currentAngle = RobotContainer.swerveDrive.getGyroInRad();
      double absoluteCurrentAngle = currentAngle%Math.PI;
      if(absoluteCurrentAngle > Math.PI){
        absoluteCurrentAngle -= 2*(Math.PI);
      }
      else if(absoluteCurrentAngle < -Math.PI){
        absoluteCurrentAngle += 2 * Math.PI;
      }
      rotateToAngle = currentAngle - absoluteCurrentAngle + desiredAngle;
      }
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
    return false;//onTargetCount >= 10;
  }
}
