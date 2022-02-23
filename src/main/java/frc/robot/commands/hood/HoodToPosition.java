// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hood;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class HoodToPosition extends CommandBase {
  private double position;
  private DoubleSupplier positionSupplier;
  private boolean positionSupplierMode;
  private boolean hasHadTarget;

  /** Creates a new HoodToPosition. */
  public HoodToPosition(double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.hood);
    positionSupplierMode = false;
    this.position = position;
  }

  /** Creates a new HoodToPosition. */
  public HoodToPosition(DoubleSupplier positionSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.hood);
    positionSupplierMode = true;
    this.positionSupplier = positionSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hasHadTarget = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    if(positionSupplierMode){
      //If speed is coming from the limeLight, check we have a target
      boolean hasTarget = RobotContainer.limeLight.hasTarget();
      //keep track if we have seen the target
      hasHadTarget |= hasTarget;
      if(hasTarget){ 
        //if there is a target, get the position
        position = positionSupplier.getAsDouble();
      }
      if(hasHadTarget){
        //if we have ever seen the target set the setpoint to the position
        RobotContainer.hood.setPosition(position);
      } 
    }else{
      //if position input hardcoded, just set the position setpoint
      RobotContainer.hood.setPosition(position);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.hood.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (!positionSupplierMode || hasHadTarget) 
      && (Math.abs(RobotContainer.hood.getPosition() - position) < 0.5);
  }
}
