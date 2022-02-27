// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ShooterSetSpeed extends CommandBase {
  private double speed;//a set speed to drive the shooter
  private DoubleSupplier speedSupplier;//a supplier that give a speed to the shooter
  private boolean speedSupplierMode;//whether we use set speed or the supplier
  private boolean withLimelight;//Are we using the limelight as a supplier
  private boolean hasHadTarget;//if we have had a target with the limelight
  private int atSpeedCount;//a count of how many times we are at speed

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
    atSpeedCount = 0;
    if(withLimelight){
      RobotContainer.limeLight.setLightState(true, RobotContainer.shooter);
    }
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

    if(Math.abs(speed - 200 - RobotContainer.shooter.getSpeed()) < 150){
      //if we're at speed add one to the count
      atSpeedCount++;
    }else{
      //if not at speed discard all previous counts.
      atSpeedCount = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(withLimelight){
      RobotContainer.limeLight.setLightState(false, RobotContainer.shooter);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (!speedSupplierMode || hasHadTarget) && (atSpeedCount >= 15);
  }
}
