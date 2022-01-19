// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Compressor extends SubsystemBase {
  //TODO: instantiate the PnumaticHub
  /** Creates a new Compressor. */
  public Compressor() {
    //TODO: construct PneumaticHub
    //TODO:run enableCompressorAnalog, max ressure and min pressure from Constants(that haven't yet been made)
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //TODO:push value of getPressure()(the one below) to SmartDashboard, round the output before pushing to SD. decimal is not needed
  }

  //TODO:accessor method for getPressure
  //TODO:modifier methods enableCompressor, disableCompressor
}
