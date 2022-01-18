/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive.SwerveModNum;

/**
 * This command stops all the motors for drive, then 
 * feeds the rotational position of each module to 
 * the smartdashboard along with the gyroscope's angle 
 * and the drive motor encoder's values. This should 
 * be run, allowing manual rotation of the modules, 
 * then a user can call DriveResetAllModulePositionsToZero.
 */
public class DriveAdjustModuleZeroPoint extends CommandBase {
  /**
   * Creates a new DriveAdjustModuleZeroPoint.
   * 
   * Each module has a different 0 angle. To solve this, 
   * physically angle each module straight. 
   * Then, tell all modules its current position is at 0. 
   * The later is done by the Command 
   * DriveResetAllModulePositionsToZero 
   */ 
  public DriveAdjustModuleZeroPoint() {
    addRequirements(RobotContainer.swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Call the stop all command onto all the modules so they can freely spin.
    RobotContainer.swerveDrive.stopAllModules();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //read the position of all modules and print them
    double[] modAngles = RobotContainer.swerveDrive.getAllAbsModuleAngles();
    double[] moduleDistances = RobotContainer.swerveDrive.getAllModuleDistance();
    double[] modRelEnc = RobotContainer.swerveDrive.getAllModuleRelEnc();
    double[] moduleVelocities = RobotContainer.swerveDrive.getAllModuleVelocity();
    
    //test smartdashboard outputs, Module Angles and Encoder Counts
    SmartDashboard.putNumber("frontLeftAngle", modAngles[SwerveModNum.frontLeft.getNumber()]);
    SmartDashboard.putNumber("frontLeftEnc", modRelEnc[SwerveModNum.frontLeft.getNumber()]);
    SmartDashboard.putNumber("frontRightAngle", modAngles[SwerveModNum.frontRight.getNumber()]);
    SmartDashboard.putNumber("frontRightEnc", modRelEnc[SwerveModNum.frontRight.getNumber()]);
    SmartDashboard.putNumber("rearLeftAngle", modAngles[SwerveModNum.rearLeft.getNumber()]);
    SmartDashboard.putNumber("rearLeftEnc", modRelEnc[SwerveModNum.rearLeft.getNumber()]);
    SmartDashboard.putNumber("rearRightAngle", modAngles[SwerveModNum.rearRight.getNumber()]);
    SmartDashboard.putNumber("rearRightEnc", modRelEnc[SwerveModNum.rearRight.getNumber()]);
    // SmartDashboard.putNumber("gyroAngle",RobotContainer.swerveDrive.getGyroInDeg());

    //Push distances pulled from the modules' drive motors
    SmartDashboard.putNumber("frontLeftDistance", moduleDistances[SwerveModNum.frontLeft.getNumber()]);
    SmartDashboard.putNumber("frontRightDistance", moduleDistances[SwerveModNum.frontRight.getNumber()]);
    SmartDashboard.putNumber("rearLeftDistance", moduleDistances[SwerveModNum.rearLeft.getNumber()]);
    SmartDashboard.putNumber("rearRightDistance", moduleDistances[SwerveModNum.rearRight.getNumber()]);

    //push velocities pulled from the drive motors
    SmartDashboard.putNumber("frontLeftVelocity", moduleVelocities[SwerveModNum.frontLeft.getNumber()]);
    SmartDashboard.putNumber("frontRightVelocity", moduleVelocities[SwerveModNum.frontRight.getNumber()]);
    SmartDashboard.putNumber("rearLeftVelocity", moduleVelocities[SwerveModNum.rearLeft.getNumber()]);
    SmartDashboard.putNumber("rearRightVelocity", moduleVelocities[SwerveModNum.rearRight.getNumber()]);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //DriveResetAllModulePositionsToZero should handle this 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public boolean runsWhenDisabled(){
    return true;
  }
}
