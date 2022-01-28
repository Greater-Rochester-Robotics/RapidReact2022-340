// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swervelib.SwervePathController;
import frc.robot.subsystems.swervelib.SwerveTrajectory;
/*
a good 85% of this code came from this database:
https://github.com/3015RangerRobotics/RobotCode2021/blob/main/src/main/java/frc/robot/commands/DriveFollowPath.java 
*/
public class DriveFollowTrajectory extends CommandBase {
  /** Creates a new DriveFollowTrajectory. */
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
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
