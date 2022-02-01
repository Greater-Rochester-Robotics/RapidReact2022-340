// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swervelib.SwervePathController;
import frc.robot.subsystems.swervelib.SwerveTrajectory;

/**
 * This helper class takes in a path and performs all necessary setup and tells the robot how to follow the provided path.
 * The majority of this code came from this file: 
 * https://github.com/3015RangerRobotics/RobotCode2021/blob/main/src/main/java/frc/robot/commands/DriveFollowPath.java
 */
public class DriveFollowTrajectory extends CommandBase {
  Timer timer;
  SwerveTrajectory traj;
  SwervePathController pathController;
  double lastTime;
  boolean ignoreHeading;

  public DriveFollowTrajectory(String trajName) {
    this(trajName, false);
  }

  public DriveFollowTrajectory(String trajName, boolean ignoreHeading) {
    addRequirements(RobotContainer.swerveDrive);
    this.timer = new Timer();
    this.traj = SwerveTrajectory.fromCSV(trajName);

    PIDController posController = new PIDController(Constants.DRIVE_POS_ERROR_CONTROLLER_P, Constants.DRIVE_POS_ERROR_CONTROLLER_I, Constants.DRIVE_POS_ERROR_CONTROLLER_D);
    PIDController headingController = new PIDController(Constants.DRIVE_HEADING_ERROR_CONTROLLER_P, Constants.DRIVE_HEADING_ERROR_CONTROLLER_I, Constants.DRIVE_HEADING_ERROR_CONTROLLER_D);
    ProfiledPIDController rotationController = new ProfiledPIDController(Constants.DRIVE_ROTATION_CONTROLLER_P, Constants.DRIVE_ROTATION_CONTROLLER_I, Constants.DRIVE_ROTATION_CONTROLLER_D,
            new TrapezoidProfile.Constraints(Constants.DRIVE_MAX_ANGULAR_VELOCITY, Constants.DRIVE_MAX_ANGULAR_ACCEL));
    this.pathController = new SwervePathController(posController, headingController, rotationController);
    this.ignoreHeading = ignoreHeading;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    SwerveTrajectory.State initialState = traj.getInitialState();
    RobotContainer.swerveDrive.setCurPose2d(new Pose2d(RobotContainer.swerveDrive.getCurPose2d().getTranslation(), initialState.getRotation()));
    pathController.reset(RobotContainer.swerveDrive.getCurPose2d());
    lastTime = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double time = timer.get();
    SwerveTrajectory.State desiredState = traj.sample(time);

    if(ignoreHeading) desiredState.rotation = new Rotation2d(0);

    ChassisSpeeds targetSpeeds = pathController.calculate(RobotContainer.swerveDrive.getCurPose2d(), desiredState, time - lastTime, timer.hasElapsed(0.1));
    RobotContainer.swerveDrive.driveRobotCentric(targetSpeeds, true);

    lastTime = time;

    // Position Graph
    // SmartDashboard.putNumber("PIDTarget", desiredState.getPos());
    // SmartDashboard.putNumber("PIDActual", pathController.getTotalDistance());

    // Heading Graph
    // SmartDashboard.putNumber("PIDTarget", desiredState.getHeading().getDegrees());
    // SmartDashboard.putNumber("PIDActual", pathController.getCurrentHeading().getDegrees());

    // Rotation Graph
    // SmartDashboard.putNumber("PIDTarget", desiredState.getRotation().getDegrees());
    // SmartDashboard.putNumber("PIDActual", RobotContainer.swerveDrive.getCurPose2d().getRotation().getDegrees());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // System.out.println(timer.get());
    timer.stop();
    RobotContainer.swerveDrive.driveRobotCentric(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(traj.getRuntime());
  }
}
