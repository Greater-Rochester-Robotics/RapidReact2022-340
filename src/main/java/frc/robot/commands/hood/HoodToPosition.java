// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hood;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class HoodToPosition extends CommandBase {
  private double position;
  private DoubleSupplier positionSupplier;
  private boolean positionSupplierMode;
  private boolean hasHadTarget;
  private boolean withLimelight;

  /** Creates a new HoodToPosition. */
  public HoodToPosition(double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.hood);
    positionSupplierMode = false;
    this.position = position;
    withLimelight = false;
  }

  /** Creates a new HoodToPosition. */
  public HoodToPosition(DoubleSupplier positionSupplier) {
    this(positionSupplier,false);
  }

  /** Creates a new HoodToPosition. */
  public HoodToPosition(DoubleSupplier positionSupplier, boolean withLimelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.hood);
    positionSupplierMode = true;
    this.positionSupplier = positionSupplier;
    this.withLimelight = withLimelight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hasHadTarget = false;
    if(withLimelight){
      RobotContainer.limeLight.setLightState(true, RobotContainer.hood);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    if(positionSupplierMode){
      //If speed is coming from the limeLight, check we have a target
      boolean hasTarget = RobotContainer.limeLight.hasTarget();
      //keep track if we have seen the target
      hasHadTarget |= hasTarget;
      if(!withLimelight || hasTarget){ 
        //if there is a target, get the position
        position = positionSupplier.getAsDouble();
      }
      if(!withLimelight || hasHadTarget){
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
    // RobotContainer.hood.stopMotor();
    if(withLimelight){
      RobotContainer.limeLight.setLightState(false, RobotContainer.hood);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (!positionSupplierMode || hasHadTarget) 
      && (Math.abs(RobotContainer.hood.getPosition() - position) < Constants.HOOD_POS_ALLOWABLE_ERROR);
  }
}
