// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.ShootHighGoal;
import frc.robot.commands.ShootHighGoalFender;
import frc.robot.commands.SpitBalls;
import frc.robot.commands.StopShooterHandlerHood;
import frc.robot.commands.autonomous.TestPath;
import frc.robot.commands.ballhandler.BallHandlerIntakeIn;
import frc.robot.commands.ballhandler.BallHandlerIntakeOut;
import frc.robot.commands.ballhandler.BallHandlerSetState;
import frc.robot.commands.climber.ClimberClimb;
import frc.robot.commands.climber.ClimberExtendIn;
import frc.robot.commands.climber.ClimberExtendOut;
import frc.robot.commands.climber.ClimberExtendoHome;
import frc.robot.commands.climber.ClimberExtendoToPosition;
import frc.robot.commands.climber.ClimberTiltIn;
import frc.robot.commands.climber.ClimberTiltOut;
import frc.robot.commands.drive.DriveFieldRelative;
import frc.robot.commands.drive.DriveFieldRelativeAdvanced;
import frc.robot.commands.drive.DriveOnTarget;
import frc.robot.commands.drive.DriveRobotCentric;
import frc.robot.commands.drive.DriveStopAllModules;
import frc.robot.commands.drive.auto.DriveFollowTrajectory;
import frc.robot.commands.drive.util.DriveAdjustModuleZeroPoint;
import frc.robot.commands.drive.util.DriveAllModulesPositionOnly;
import frc.robot.commands.drive.util.DriveFindMaxAccel;
import frc.robot.commands.drive.util.DriveOneModule;
import frc.robot.commands.drive.util.DriveResetAllModulePositionsToZero;
import frc.robot.commands.drive.util.DriveResetGyroToZero;
import frc.robot.commands.drive.util.DriveSetGyro;
import frc.robot.commands.drive.util.DriveTuneDriveMotorFeedForward;
import frc.robot.commands.drive.util.DriveTurnToAngle;
import frc.robot.commands.hood.HoodHome;
import frc.robot.commands.hood.HoodToPosition;
import frc.robot.commands.shooter.ShooterPercentOutput;
import frc.robot.commands.shooter.ShooterSetSpeed;
import frc.robot.commands.shooter.ShooterStop;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Compressor;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.BallHandler;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.BallHandler.State;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's gamepads are defined here...
  
  static final XboxController driver = new XboxController(0);
  static final XboxController coDriver = new XboxController(1);

  ////////////////////
  // DRIVER BUTTONS //
  ////////////////////

  static final Button driverA = new JoystickButton(driver, 1);
  static final Button driverB = new JoystickButton(driver, 2);
  static final Button driverX = new JoystickButton(driver, 3);
  static final Button driverY = new JoystickButton(driver, 4);
  static final Button driverLB = new JoystickButton(driver, 5);
  static final Button driverRB = new JoystickButton(driver, 6);
  static final Button driverBack = new JoystickButton(driver, 7);
  static final Button driverStart = new JoystickButton(driver, 8);
  static final Button driverLS = new JoystickButton(driver, 9);
  static final Button driverRS = new JoystickButton(driver, 10);
  static final Button driverDUp = new POVButton(driver, 0);
  static final Button driverDDown = new POVButton(driver, 180);
  static final Button driverDLeft = new POVButton(driver, 270);
  static final Button driverDRight = new POVButton(driver, 90);
  // final Button driverLTButton = new JoyTriggerButton(driver, .3, Axis.LEFT_TRIGGER);
  // final Button driverRTButton = new JoyTriggerButton(driver, .3, Axis.RIGHT_TRIGGER);

  ///////////////////////
  // CO-DRIVER BUTTONS //
  ///////////////////////

  static final Button coDriverA = new JoystickButton(coDriver, 1);
  static final Button coDriverB = new JoystickButton(coDriver, 2);
  static final Button coDriverX = new JoystickButton(coDriver, 3);
  static final Button coDriverY = new JoystickButton(coDriver, 4);
  static final Button coDriverLB = new JoystickButton(coDriver, 5);
  static final Button coDriverRB = new JoystickButton(coDriver, 6);
  static final Button coDriverBack = new JoystickButton(coDriver, 7);
  static final Button coDriverStart = new JoystickButton(coDriver, 8);
  static final Button coDriverLS = new JoystickButton(coDriver, 9);
  static final Button coDriverRS = new JoystickButton(coDriver, 10);
  static final Button coDriverDUp = new POVButton(coDriver, 0);
  static final Button coDriverDDown = new POVButton(coDriver, 180);
  static final Button coDriverDLeft = new POVButton(coDriver, 270);
  static final Button coDriverDRight = new POVButton(coDriver, 90);
  // final Button coDriverLTButton = new JoyTriggerButton(coDriver, .7, Axis.LEFT_TRIGGER);
  // final Button coDriverRTButton = new JoyTriggerButton(coDriver, .7, Axis.RIGHT_TRIGGER);

  //Climber next step button is aliased here.
  public static final Button climberButton = driverStart;

  //The robot's subsystems are instantiated here
  public static Compressor compressor;
  public static SwerveDrive swerveDrive;
  public static Climber climber;
  public static BallHandler ballHandler;
  public static LimeLight limeLight;
  public static Shooter shooter;
  public static Hood hood;

  //The sendable chooser for autonomous is constructed here
  public static SendableChooser<Command> autoChooser = new SendableChooser<Command>();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //create(construct) subsystems
    compressor = new Compressor();//Let's keep compressor first
    swerveDrive = new SwerveDrive();
    swerveDrive.setDefaultCommand(new DriveFieldRelativeAdvanced());
    climber = new Climber();
    ballHandler = new BallHandler();
    limeLight = new LimeLight();
    shooter = new Shooter();
    hood = new Hood();

    //Add all autos to the auto selector
    // configureAutoModes();

    // Configure the button bindings
    configureButtonBindings();

    //add some commands to dashboard for testing/configuring
    SmartDashboard.putData(new DriveResetAllModulePositionsToZero());//For setup of swerve
    SmartDashboard.putData(new DriveAdjustModuleZeroPoint());//For setup of swerve
    SmartDashboard.putData("Drive Module 0", new DriveOneModule(0));//For setup of swerve
    SmartDashboard.putData("Drive Module 1", new DriveOneModule(1));//For setup of swerve
    SmartDashboard.putData("Drive Module 2", new DriveOneModule(2));//For setup of swerve
    SmartDashboard.putData("Drive Module 3", new DriveOneModule(3));//For setup of swerve
    SmartDashboard.putData(new DriveFindMaxAccel());//This is for PathPlanning Parameters
    SmartDashboard.putData(new DriveStopAllModules());//For setup of swerve
    SmartDashboard.putData(new DriveTuneDriveMotorFeedForward(1.0));//this is for Velocity PID parameters, Path Planning by Extention
    SmartDashboard.putData(new DriveAllModulesPositionOnly());
    // SmartDashboard.putData(new BallHandlerIntakeIn());
    // SmartDashboard.putData(new BallHandlerIntakeOut()); 
    SmartDashboard.putData(new ShooterSetSpeed(this::speedIShoot));//Testing Shooter
    SmartDashboard.putData(new HoodToPosition(this::angleIShoot));//Testing Shooter
    SmartDashboard.putData(new ShooterStop());//Testing Shooter
    SmartDashboard.putData(new HoodHome());//For setup
    SmartDashboard.putData(new DriveTurnToAngle(Constants.PI_OVER_TWO));

    SmartDashboard.putNumber("SpeedIShoot",0.0);
    SmartDashboard.putNumber("angleIShoot",0.0);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    driverA.whenPressed(new BallHandlerSetState(State.kFillTo1));
    driverA.whenReleased(new BallHandlerSetState(State.kOff));
    driverB.whenPressed(new SpitBalls());
    driverB.whenReleased(new StopShooterHandlerHood());
    driverX.whenPressed(new ShootHighGoalFender(1));
    driverX.whenReleased(new StopShooterHandlerHood());
    driverY.whenPressed(new ShootHighGoal(1));
    driverY.whenReleased(new StopShooterHandlerHood());
    driverLB.whenPressed(new DriveResetGyroToZero());
    driverRB.whileHeld(new DriveOnTarget());
    // driverStart.whenPressed(new DriveFollowTrajectory("DriveStraight"));
    driverBack.toggleWhenActive(new DriveRobotCentric());
  }

  /**
   * testing double supplier
   */
  public double speedIShoot(){
    return SmartDashboard.getNumber("SpeedIShoot",0.0);
  }
  /**
   * testing double supplier
   */
  public double angleIShoot(){
    return SmartDashboard.getNumber("angleIShoot",0.0);
  }

  /**
   * Define all autonomous modes here to have them 
   * appear in the autonomous select drop down menu.
   */
  private void configureAutoModes() {
    //TODO:add auto modes to the sendable chooser when autos written
    autoChooser.setDefaultOption("Wait 1 sec(do nothing)", new WaitCommand(1));

    // autoChooser.addOption("Barrel Racing 64", new AutoBarrelPath());
  // autoChooser.addOption("TestPath", new TestPath());

    SmartDashboard.putData(RobotContainer.autoChooser);
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
        : 0.0;
  }

  /**
   * Accessor method to set codriver rumble function
   * 
   * @param leftRumble
   * @param rightRumble
   */
  public static void setDriverRumble(double leftRumble, double rightRumble) {
    driver.setRumble(RumbleType.kLeftRumble, leftRumble);
    driver.setRumble(RumbleType.kRightRumble, rightRumble);
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
