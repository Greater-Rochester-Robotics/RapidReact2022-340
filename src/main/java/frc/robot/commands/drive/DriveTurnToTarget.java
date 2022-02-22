// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveTurnToTarget extends CommandBase {
  Timer timer = new Timer();
  boolean hasHadTarget;
  double setPointAngle;
  double offsetDistance;//TODO: impliment so we can shoot off to the side of the goal

  public DriveTurnToTarget(){
    this(0.0);
  }

  public DriveTurnToTarget(double offsetDistance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.offsetDistance = offsetDistance;
    addRequirements(RobotContainer.swerveDrive, RobotContainer.limeLight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.limeLight.setLightState(3);
    timer.start();
    hasHadTarget = false;
    // Default to the current angle of the robot
    setPointAngle = RobotContainer.swerveDrive.getGyroInRad();
  }

  // Called every time the scheduler runs while the command is scheduled.
  
  @Override
  public void execute() {
    boolean isOnTarget = RobotContainer.limeLight.haveTarget();
    hasHadTarget |= isOnTarget;

    
    // Point robot in the general direction of the target if the limelight doesn't see the target
    if(isOnTarget) {
      setPointAngle = RobotContainer.swerveDrive.getGyroInRad() - Math.toRadians(RobotContainer.limeLight.angleToTarget());
    }else if(RobotContainer.swerveDrive.hasPoseBeenSet() && !hasHadTarget && timer.hasElapsed(.2)){
      // Finds where we are relative to the center of the field
      Translation2d target = RobotContainer.swerveDrive.driveOdometry.getPoseMeters().getTranslation().minus(Constants.FIELD_CENTER);
      double desiredAngle = Math.atan2(target.getY(), target.getX());

      // Calculating current angle between -pi and pi
      double currentAngle = RobotContainer.swerveDrive.getGyroInRad();
      double absoluteCurrentAngle = currentAngle%Constants.TWO_PI;
      if(absoluteCurrentAngle > Math.PI){
        absoluteCurrentAngle -= 2*(Math.PI);
      }
      else if(absoluteCurrentAngle < -Math.PI){
        absoluteCurrentAngle += 2 * Math.PI;
      }

      // Calculate the robot's target angle given the continuous angle of the gyroscope
      setPointAngle = currentAngle - absoluteCurrentAngle + desiredAngle;
    }
    // Set the rotate to angle to the target if limelight sees it
    

    // Drive robot using driver axis and rotateToAngle
    RobotContainer.swerveDrive.driveFieldRelative(
      0.0,
      0.0,
      RobotContainer.swerveDrive.getRobotRotationPIDOut(setPointAngle), 
      false
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.swerveDrive.stopAllModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.limeLight.haveTarget() 
          && Math.abs(RobotContainer.swerveDrive.getRotationalVelocity()) < Constants.ROTATIONAL_VELOCITY_TOLERANCE 
          && Math.abs(RobotContainer.limeLight.angleToTarget()) < Constants.LL_ANGLE_TOLERANCE;
  }
}
