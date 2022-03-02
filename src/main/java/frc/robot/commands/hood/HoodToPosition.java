// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hood;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class HoodToPosition extends CommandBase {
  private double position;
  private DoubleSupplier positionSupplier;
  private boolean positionSupplierMode;
  private boolean hasHadTarget;
  private boolean withLimelight;

  /**
   * Sets the hood to input position in degrees.
   * @param position should be between 0 and 22 degrees
   */
  public HoodToPosition(double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.hood);
    positionSupplierMode = false;//not supplierMode as set to position
    this.position = position;
    withLimelight = false;//no using the limelight
  }

  /**
   * Sets the hood to the supplied position, assumed that no limelight used.
   * @param positionSupplier
   */
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
    //we haven't seen the target yet
    hasHadTarget = false;
    if(withLimelight){
      //turn on the limelight if we are using it
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
        //TODO: If position is larger than max position, drop to HOOD_FORWARD_LIMIT_DEGREES
        //if we have ever seen the target set the setpoint to the position
        RobotContainer.hood.setPosition(position);
      } 
    }else{
      //TODO: If position is larger than max position, drop to HOOD_FORWARD_LIMIT_DEGREES
      //if position input hardcoded, just set the position setpoint
      RobotContainer.hood.setPosition(position);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // RobotContainer.hood.stopMotor();//we aren't stopping the motor so thatthe PID will hold the position
    if(withLimelight){
      //turn off the limelight if we used it
      RobotContainer.limeLight.setLightState(false, RobotContainer.hood);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //end when we are at the target position (only if we have seen target, when using target though)
    return (!positionSupplierMode || hasHadTarget) 
      && (Math.abs(RobotContainer.hood.getPosition() - position) < Constants.HOOD_POS_ALLOWABLE_ERROR);
  }
}
