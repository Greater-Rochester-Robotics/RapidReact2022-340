// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;
import frc.robot.RobotContainer;

/**
 * A testing command meant to spin the motor at a specific 
 * velocity, allowing data from drive motors without need 
 * for a full field. PID and F values can be changed through 
 * the SmartDashboard without need to redeploy code. These 
 * values will need to be placed into constants so that they 
 * are saved for future code deploys.
 * for use with https://docs.ctre-phoenix.com/en/latest/ch16_ClosedLoop.html
 */
public class DriveTuneDriveMotorPID extends CommandBase {
  public double kP = Constants.SWERVE_DRIVE_P_VALUE;
  public double kI = Constants.SWERVE_DRIVE_I_VALUE;
  public double kD = Constants.SWERVE_DRIVE_D_VALUE;
  public double kF = Constants.SWERVE_DRIVE_FF_VALUE;
  public double speed = 1;
  private double[] angle = new double[]{Math.toRadians(135), Math.toRadians(135), Math.toRadians(-45), Math.toRadians(45)};    

  /** Creates a new DriveTuneDriveMotorPIDF. */
  @Deprecated
  public DriveTuneDriveMotorPID(double speed) {
    this.speed = speed;
    addRequirements(RobotContainer.swerveDrive);
    //Push PID Constants(From Constants.java) to SmartDashboard
    SmartDashboard.putNumber("driveP", kP);
    SmartDashboard.putNumber("driveI", kI);
    SmartDashboard.putNumber("driveD", kD);
    SmartDashboard.putNumber("driveF", kF);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //you can look at for reference https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Position%20Closed%20Loop%20Control/src/main/java/frc/robot/Robot.java lines 79-94
    //Pull PIDF values from the SmartDashboard and store in variables
    double sdP = SmartDashboard.getNumber("driveP", 0);
    double sdI = SmartDashboard.getNumber("driveI", 0);
    double sdD = SmartDashboard.getNumber("driveD", 0);
    double sdFF = SmartDashboard.getNumber("driveF", 0);
    
    //Check if any SmartDashboard PID constants are different from field constants
    if((sdP != kP)||(sdI != kI)||(sdD != kD)||(sdFF != kF)) {
      //if PIDf coefficients on SmartDashboard have changed, write new values to controller
      //assign new PIDF coefficients to field PIDF constants
      RobotContainer.swerveDrive.setDrivePIDF(sdP, sdI, sdD, sdFF);
      kP = sdP;
      kI = sdI;
      kD =sdD;
      kF = sdFF;


    }
    
    //use driveOneModule for each motor, position setting m0 to 135, m1 to -135, m2 to -45, and m3 to 45, set speed to constructor param
    for (int i=0; i<4; i++){
      RobotContainer.swerveDrive.driveOneModule(i, speed, angle[i], true);
      //Print all module velocities
      SmartDashboard.putNumber("Module Velocity", RobotContainer.swerveDrive.getAllModuleVelocity()[i]);
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
    return false;
  }
}
