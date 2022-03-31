// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ADIS.IMUAxis;

public class DriveStraightBack extends CommandBase {
  Timer timer;
  TrapezoidProfile profile;
  double distance;
  PIDController backController;
  Pose2d intpPose2d;
  double curDistance;
  private double currentAngle = 0;

  /**
   *Simple autonomous for backing robot up 
   by the given distance(m)

   @param distance in meters backwards
   */
  public DriveStraightBack(double distance) {
    addRequirements(RobotContainer.swerveDrive);

    this.distance = distance;
    timer = new Timer();
    backController = new PIDController(Constants.DRIVE_POS_ERROR_CONTROLLER_P, Constants.DRIVE_POS_ERROR_CONTROLLER_I, Constants.DRIVE_POS_ERROR_CONTROLLER_D);
    TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(Constants.MOTOR_MAXIMUM_VELOCITY, Constants.MAXIMUM_ACCELERATION);
    TrapezoidProfile.State goal = new TrapezoidProfile.State(distance, 0);
    TrapezoidProfile.State initial = new TrapezoidProfile.State(0, 0);
    profile = new TrapezoidProfile(constraints, goal, initial);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.swerveDrive.setGyroAxis(IMUAxis.kY);
    RobotContainer.swerveDrive.setIsOdometry(true);
    intpPose2d = RobotContainer.swerveDrive.getCurPose2d();
    curDistance = 0.0;
    backController.reset();
    timer.reset();
    timer.start();
    curDistance = distance;
    currentAngle = RobotContainer.swerveDrive.getGyroInRad();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    TrapezoidProfile.State targetState = profile.calculate(timer.get());
    curDistance = intpPose2d.getTranslation().getDistance(RobotContainer.swerveDrive.getCurPose2d().getTranslation());
    double output = targetState.velocity + backController.calculate(curDistance, distance);
    double counterRotation = RobotContainer.swerveDrive.getCounterRotationPIDOut(currentAngle);
    counterRotation *= Constants.MOTOR_MAXIMUM_VELOCITY;
    RobotContainer.swerveDrive.driveRobotCentric(-output,0.0,counterRotation,true,false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //stop the drive when this command ends
    RobotContainer.swerveDrive.stopAllModules();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //stop the command when we are within 5 cm of the target distance 
    return Math.abs(distance - curDistance) <= 0.05;
  }
}
