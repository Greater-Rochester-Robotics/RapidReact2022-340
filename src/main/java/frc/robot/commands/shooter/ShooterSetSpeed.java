// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ShooterSetSpeed extends CommandBase {
  private double speed;
  private DoubleSupplier speedSupplier;
  private boolean speedSupplierMode;
  private boolean hasHadTarget;
  private boolean withLimelight;

  /** 
   * Sets speed of the shooter to a speed given 
   * the double. this command ends when the shooter 
   * is at speed. This command does not stop the motor.
   */
  public ShooterSetSpeed(double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooter);
    this.speedSupplierMode = false;
    this.speed = speed;
    withLimelight = false;
  }

  public ShooterSetSpeed(DoubleSupplier speedSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooter);
    speedSupplierMode = true;
    this.speedSupplier = speedSupplier;
    this.withLimelight = false;
  }

  /** 
   * Sets speed of the shooter to a speed given by the DoubleSupplier.
   * This will only work when the limelight has target.
   * This command does not stop the motor.
   * 
   */
  public ShooterSetSpeed(DoubleSupplier speedSupplier, boolean withLimelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooter);
    speedSupplierMode = true;
    this.speedSupplier = speedSupplier;
    this.withLimelight = withLimelight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hasHadTarget = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(speedSupplierMode){
      //If speed is coming from the limeLight, check we have a target
      boolean hasTarget = RobotContainer.limeLight.hasTarget();
      //keep track if we have seen the target
      hasHadTarget |= hasTarget;
      if(!withLimelight || hasTarget){ 
        //if there is a target, get the speed
        speed = speedSupplier.getAsDouble();
      }
      if(!withLimelight || hasHadTarget){
        //if we have ever seen the target set the setpoint to the speed
        RobotContainer.shooter.setSpeed(speed); 
      } 
    }else{
      //if speed input hardcoded, just set the speed setpoint 
      RobotContainer.shooter.setSpeed(speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (!speedSupplierMode || hasHadTarget) && RobotContainer.shooter.isAtSpeed();
  }
}
