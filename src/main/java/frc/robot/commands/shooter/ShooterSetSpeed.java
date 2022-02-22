// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ShooterSetSpeed extends CommandBase {
  public double speed;
  private DoubleSupplier speedSupplier;
  private boolean speedSupplierMode;

  /** 
   * Sets speed of the shooter 
   */
  public ShooterSetSpeed(double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooter);
    this.speedSupplierMode = false;
    this.speed = speed;
  }

  public ShooterSetSpeed(DoubleSupplier speedSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooter);
    speedSupplierMode = true;
    this.speedSupplier = speedSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(speedSupplierMode){
      speed = speedSupplier.getAsDouble();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.shooter.setSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.shooter.isAtSpeed();
  }
}
