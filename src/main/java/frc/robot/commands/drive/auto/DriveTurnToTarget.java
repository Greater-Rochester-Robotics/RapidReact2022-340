// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ADIS.IMUAxis;

public class DriveTurnToTarget extends CommandBase {
  Timer timer = new Timer();
  boolean hasHadTarget;
  double setPointAngle;
  double offsetDistance;

  public DriveTurnToTarget(){
    this(0.0);
  }

  public DriveTurnToTarget(double offsetDistance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.offsetDistance = offsetDistance;
    addRequirements(RobotContainer.swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.swerveDrive.setGyroAxis(IMUAxis.kY);
    RobotContainer.limeLight.setLightState(true, RobotContainer.swerveDrive);
    timer.start();
    hasHadTarget = false;
    // Default to the current angle of the robot
    setPointAngle = RobotContainer.swerveDrive.getGyroInRad();
  }

  // Called every time the scheduler runs while the command is scheduled.
  
  @Override
  public void execute() {
    boolean hasTarget = RobotContainer.limeLight.hasTarget();
    hasHadTarget |= hasTarget;

    // Set the rotate to angle to the target if limelight sees it
    if(hasTarget) {
      setPointAngle = RobotContainer.swerveDrive.getGyroInRad() - Math.toRadians(RobotContainer.limeLight.angleToTarget(offsetDistance));
    }else if(RobotContainer.swerveDrive.hasPoseBeenSet() && !hasHadTarget && timer.hasElapsed(.2)){
      // Point robot in the general direction of the target if the limelight doesn't see the target
      // Finds where we are relative to the center of the field, set as setPoint
      setPointAngle = RobotContainer.swerveDrive.getAngleOfTarget();
      hasHadTarget = true;
    }

    // Drive robot using driver axis and rotateToAngle
    RobotContainer.swerveDrive.driveRobotCentric(
      0.0,
      0.0,
      RobotContainer.swerveDrive.getRobotRotationPIDOut(setPointAngle), 
      false,
      true
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.swerveDrive.stopAllModules();
    RobotContainer.limeLight.setLightState(false, RobotContainer.swerveDrive);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.limeLight.hasTarget() 
          && Math.abs(RobotContainer.swerveDrive.getRotationalVelocity()) < Constants.ROTATIONAL_VELOCITY_TOLERANCE 
          && Math.abs(RobotContainer.limeLight.angleToTarget()) < Constants.LL_ANGLE_TOLERANCE;
  }
}
