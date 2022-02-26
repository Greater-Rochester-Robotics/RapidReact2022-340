// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/**
 * This helper class takes in a path and performs all necessary setup and tells the robot how to follow the provided path.
 * The majority of this code came from this file: 
 * https://github.com/3015RangerRobotics/RobotCode2021/blob/main/src/main/java/frc/robot/commands/DriveFollowPath.java
 */
public class DriveFollowTrajectory extends CommandBase {
  Timer timer;
  PathPlannerTrajectory trajectory;
  HolonomicDriveController pathController;
  boolean resetOdometry;

  public DriveFollowTrajectory(String trajName) {
    this(trajName, Constants.PATH_MAXIMUM_VELOCITY, Constants.MAXIMUM_ACCELERATION);
  }

  public DriveFollowTrajectory(String trajName, double maxVel, double maxAccel) {
    this(trajName, maxVel, maxAccel, true);
  }

  public DriveFollowTrajectory(String trajName, double maxVel, double maxAccel, boolean resetOdometry) {
    addRequirements(RobotContainer.swerveDrive);
    this.timer = new Timer();
    this.trajectory = PathPlanner.loadPath(trajName, maxVel, maxAccel);

    PIDController xController = new PIDController(Constants.DRIVE_POS_ERROR_CONTROLLER_P, Constants.DRIVE_POS_ERROR_CONTROLLER_I, Constants.DRIVE_POS_ERROR_CONTROLLER_D);
    PIDController yController = new PIDController(Constants.DRIVE_POS_ERROR_CONTROLLER_P, Constants.DRIVE_POS_ERROR_CONTROLLER_I, Constants.DRIVE_POS_ERROR_CONTROLLER_D);
    ProfiledPIDController rotationController = new ProfiledPIDController(Constants.DRIVE_ROTATION_CONTROLLER_P, Constants.DRIVE_ROTATION_CONTROLLER_I, Constants.DRIVE_ROTATION_CONTROLLER_D,
            new TrapezoidProfile.Constraints(Constants.DRIVE_MAX_ANGULAR_VELOCITY, Constants.DRIVE_MAX_ANGULAR_ACCEL));
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    this.pathController = new HolonomicDriveController(xController, yController, rotationController);
    this.resetOdometry = resetOdometry;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    Pose2d initialState = trajectory.getInitialPose();
    if(resetOdometry) {
      RobotContainer.swerveDrive.setCurPose2d(new Pose2d(initialState.getTranslation(),new Rotation2d(0)));
      RobotContainer.swerveDrive.setGyro(initialState.getRotation().getDegrees());
      
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double time = timer.get();
    PathPlannerState desiredState = (PathPlannerState) trajectory.sample(time);

    ChassisSpeeds targetSpeeds = pathController.calculate(RobotContainer.swerveDrive.getCurPose2d(), desiredState, new Rotation2d(desiredState.holonomicRotation.getRadians()));
    RobotContainer.swerveDrive.driveRobotCentric(targetSpeeds, true, false);

    // Position Graph
    // SmartDashboard.putNumber("PIDTarget", desiredState.getPos());
    // SmartDashboard.putNumber("PIDActual", pathController.getTotalDistance());

    // // Heading Graph
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
    RobotContainer.swerveDrive.driveRobotCentric(0, 0, 0, true, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(trajectory.getTotalTimeSeconds());
  }
}
