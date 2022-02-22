// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShooterPrepShot extends InstantCommand {
  private double speed;
  public ShooterPrepShot() {
    this(Constants.SHOOOTER_PREP_SPEED);
  }
  // public ShooterPrepShot(boolean isHigh){
  //   if(isHigh){
  //     this(Constants.SHOOOTER_PREP_SPEED);
  //   }else{
  //     this(Constants.SHOOOTER_PREP_SPEED);
  //   }
  // }
  public ShooterPrepShot(double speed) {
    addRequirements(RobotContainer.shooter);
    this.speed = speed;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.shooter.setSpeed(speed);
  }
}
