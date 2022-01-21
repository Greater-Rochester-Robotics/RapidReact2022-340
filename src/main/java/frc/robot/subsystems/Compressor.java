// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Compressor extends SubsystemBase {
  PneumaticHub pneumaticHub;
  /** Creates a new Compressor. */
  public Compressor() {
    pneumaticHub = new PneumaticHub();
    pneumaticHub.enableCompressorAnalog(Constants.MIN_PRESSURE, Constants.MAX_PRESSURE);
  }

  @Override
  public void periodic() {
    // push value of getPressure() to SmartDashboard, round the output before pushing to SD. decimal is not needed
    SmartDashboard.putNumber("Pressure", Math.round(getPressure()));
  }

  /**
   * Gets and returns pressure of pneumatic hub.
   * @return Pressure in PSI
   */
  public double getPressure(){
    return pneumaticHub.getPressure(0);
  }
  
  /** Enables compressor. */
  public void enableCompressor(){
    pneumaticHub.enableCompressorAnalog(Constants.MIN_PRESSURE, Constants.MAX_PRESSURE);
  }

  /** Disables compressor. */
  public void disableCompressor(){
    pneumaticHub.disableCompressor();
  }
}