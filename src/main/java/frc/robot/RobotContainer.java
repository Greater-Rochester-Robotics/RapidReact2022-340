// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Compressor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's gamepads are defined here...
  
  final XboxController driver = new XboxController(0);
  final XboxController coDriver = new XboxController(1);

  ////////////////////
  // DRIVER BUTTONS //
  ////////////////////

  final Button driverA = new JoystickButton(driver, 1);
  final Button driverB = new JoystickButton(driver, 2);
  final Button driverX = new JoystickButton(driver, 3);
  final Button driverY = new JoystickButton(driver, 4);
  final Button driverLB = new JoystickButton(driver, 5);
  final Button driverRB = new JoystickButton(driver, 6);
  final Button driverBack = new JoystickButton(driver, 7);
  final Button driverStart = new JoystickButton(driver, 8);
  final Button driverLS = new JoystickButton(driver, 9);
  final Button driverRS = new JoystickButton(driver, 10);
  final Button driverDUp = new POVButton(driver, 0);
  final Button driverDDown = new POVButton(driver, 180);
  final Button driverDLeft = new POVButton(driver, 270);
  final Button driverDRight = new POVButton(driver, 90);
  // final Button driverLTButton = new JoyTriggerButton(driver, .3, Axis.LEFT_TRIGGER);
  // final Button driverRTButton = new JoyTriggerButton(driver, .3, Axis.RIGHT_TRIGGER);

  ///////////////////////
  // CO-DRIVER BUTTONS //
  ///////////////////////

  final Button coDriverA = new JoystickButton(coDriver, 1);
  final Button coDriverB = new JoystickButton(coDriver, 2);
  final Button coDriverX = new JoystickButton(coDriver, 3);
  final Button coDriverY = new JoystickButton(coDriver, 4);
  final Button coDriverLB = new JoystickButton(coDriver, 5);
  final Button coDriverRB = new JoystickButton(coDriver, 6);
  final Button coDriverBack = new JoystickButton(coDriver, 7);
  final Button coDriverStart = new JoystickButton(coDriver, 8);
  final Button coDriverLS = new JoystickButton(coDriver, 9);
  final Button coDriverRS = new JoystickButton(coDriver, 10);
  final Button coDriverDUp = new POVButton(coDriver, 0);
  final Button coDriverDDown = new POVButton(coDriver, 180);
  final Button coDriverDLeft = new POVButton(coDriver, 270);
  final Button coDriverDRight = new POVButton(coDriver, 90);
  // final Button coDriverLTButton = new JoyTriggerButton(coDriver, .7, Axis.LEFT_TRIGGER);
  // final Button coDriverRTButton = new JoyTriggerButton(coDriver, .7, Axis.RIGHT_TRIGGER);


  //The robot's subsystems are instantiated here
  public static SwerveDrive swerveDrive;
  public static Climber climber;
  public static Compressor compressor;
  public static Intake intake;
  public static LimeLight limeLight;
  public static Shooter shooter;

  //The sendable chooser for autonomous is constructed here
  //TODO: construct the sendable chooser 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //create(construct) subsystems
    swerveDrive = new SwerveDrive();
    climber = new Climber();
    compressor = new Compressor();
    intake = new Intake();
    limeLight = new LimeLight();
    shooter = new Shooter();

    //Add all autos to the auto selector
    configureAutoModes();

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Define all autonomous modes here to have them 
   * appear in the autonomous select drop down menu.
   */
  private void configureAutoModes() {
    //TODO:add auto modes to the sendable chooser
  }

  
  /**
   * A method to return the value of a driver joystick axis,
   * which runs from -1.0 to 1.0, with a .1 dead zone(a 0 
   * value returned if the joystick value is between -.1 and 
   * .1)
   * @param axis
   * @return value of the joystick, from -1.0 to 1.0 where 0.0 is centered
   */
  public double getDriverAxis(Axis axis) {
    return (driver.getRawAxis(axis.value) < -.1 || driver.getRawAxis(axis.value) > .1)
        ? driver.getRawAxis(axis.value)
        : 0;
  }

  /**
   * Returns the int position of the DPad/POVhat based
   * on the following table:
   *    input    |return
   * not pressed |  -1
   *     up      |   0
   *   up right  |  45
   *    right    |  90
   *  down right | 135
   *    down     | 180
   *  down left  | 225
   *    left     | 270
   *   up left   | 315
   * @return
   */
  public int getDriverDPad() {
    return (driver.getPOV());
  }

  /**
   * A method to return the value of a codriver joystick axis,
   * which runs from -1.0 to 1.0, with a .1 dead zone(a 0 
   * value returned if the joystick value is between -.1 and 
   * .1) 
   * @param axis
   * @return
   */
  public double getCoDriverAxis(Axis axis) {
    return (coDriver.getRawAxis(axis.value) < -.1 || coDriver.getRawAxis(axis.value) > .1)
        ? coDriver.getRawAxis(axis.value)
        : 0;
  }

  /**
   * Accessor method to set codriver rumble function
   * 
   * @param leftRumble
   * @param rightRumble
   */
  public void setCoDriverRumble(double leftRumble, double rightRumble) {
    coDriver.setRumble(RumbleType.kLeftRumble, leftRumble);
    coDriver.setRumble(RumbleType.kRightRumble, rightRumble);
  }

  /**
   * accessor to get the true/false of the buttonNum 
   * on the coDriver control
   * @param buttonNum
   * @return the value of the button
   */
  public boolean getCoDriverButton(int buttonNum) {
    return coDriver.getRawButton(buttonNum);
  }
}
