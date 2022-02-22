// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ShooterHoodToPosition extends CommandBase {
  private double position;
  private DoubleSupplier positionSupplier;
  private boolean positionSupplierMode;

  /** Creates a new ShooterHoodToPosition. */
  public ShooterHoodToPosition(double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooter);
    positionSupplierMode = false;
    this.position = position;
  }

  /** Creates a new ShooterHoodToPosition. */
  public ShooterHoodToPosition(DoubleSupplier positionSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooter);
    positionSupplierMode = true;
    this.positionSupplier = positionSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(positionSupplierMode){
      position = positionSupplier.getAsDouble();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    RobotContainer.shooter.setHoodPosition(position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooter.stopHoodMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(RobotContainer.shooter.getHoodPosition() - position) < 0.5;
  }
}
