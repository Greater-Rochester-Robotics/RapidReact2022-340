// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.sound.midi.Sequence;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SendableCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import frc.robot.commands.LimeLightPowerCycle;
import frc.robot.commands.PrepLowFender;
import frc.robot.commands.ShootHighFenderWithDriveBack;
import frc.robot.commands.ShootHighGoal;
import frc.robot.commands.ShootHighFender;
import frc.robot.commands.ShootLowGoalFender;
import frc.robot.commands.SpitBalls;
import frc.robot.commands.StopShooterHandlerHood;

import frc.robot.commands.autonomous.AutoDriveBackAndShoot;
import frc.robot.commands.autonomous.AutoLeftBackOutOfWay;
import frc.robot.commands.autonomous.AutoLeftFourBall;
import frc.robot.commands.autonomous.AutoLeftTwoBall;
import frc.robot.commands.autonomous.AutoLeftTwoBallFromHub;
import frc.robot.commands.autonomous.AutoLeftTwoBallThenOppToss;
import frc.robot.commands.autonomous.AutoMidFourBall;
import frc.robot.commands.autonomous.AutoMidTwoBall;
import frc.robot.commands.autonomous.AutoPartnerPickupLeftBall;
import frc.robot.commands.autonomous.AutoRightFiveBall;
import frc.robot.commands.autonomous.AutoRightThreeBall;
import frc.robot.commands.autonomous.AutoRightTwoBall;

import frc.robot.commands.ballhandler.BallHandlerIntakeIn;
import frc.robot.commands.ballhandler.BallHandlerIntakeOut;
import frc.robot.commands.ballhandler.BallHandlerSetState;

import frc.robot.commands.climber.ClimberClimb;
import frc.robot.commands.climber.ClimberExtendBothIn;
import frc.robot.commands.climber.ClimberExtendLeftIn;
import frc.robot.commands.climber.ClimberExtendOut;
import frc.robot.commands.climber.ClimberExtendRightIn;
import frc.robot.commands.climber.ClimberExtendoHome;
import frc.robot.commands.climber.ClimberExtendoToPosition;
import frc.robot.commands.climber.ClimberTiltIn;
import frc.robot.commands.climber.ClimberTiltOut;

import frc.robot.commands.drive.DriveFieldRelative;
import frc.robot.commands.drive.DriveFieldRelativeAdvanced;
import frc.robot.commands.drive.DriveOnTarget;
import frc.robot.commands.drive.DriveRobotCentric;
import frc.robot.commands.drive.DriveRobotCentricNoGyro;
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

import frc.robot.commands.shooter.ShooterPrepShot;
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
  // final Button driverLTButton = new JoyTriggerButton(driver, .3, Axis.LEFT_TRIGGER);//This is used in driving, don't enable
  // final Button driverRTButton = new JoyTriggerButton(driver, .3, Axis.RIGHT_TRIGGER);//This is used in driving, don't enable

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
  static final Button coDriverLTButton70 = new JoyTriggerButton(coDriver, .7, Axis.kLeftTrigger);
  static final Button coDriverRTButton70 = new JoyTriggerButton(coDriver, .7, Axis.kRightTrigger);
  static final Button coDriverLTButton25 = new JoyTriggerButton(coDriver, .25, Axis.kLeftTrigger);
  static final Button coDriverRTButton25 = new JoyTriggerButton(coDriver, .25, Axis.kRightTrigger);

  //Climber next step button is aliased here.
  public static final Button climberManButton = coDriverA;
  public static final Button climberAutoButton = coDriverB;
  public static SendableCommandGroup climbCommand;
  
  public class ClimberCommandRunning extends Button{
    public boolean get(){
      return climbCommand.isScheduled();
    }
  };
  public static ClimberCommandRunning climberCommandRunning;

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

    climbCommand = new ClimberClimb();
    climberCommandRunning = new ClimberCommandRunning();

    //Add all autos to the auto selector
    configureAutoModes();

    // Configure the button bindings
    configureButtonBindings();

    //add some commands to dashboard for testing/configuring
    SmartDashboard.putData(new DriveResetAllModulePositionsToZero());//For setup of swerve
    SmartDashboard.putData(new DriveAdjustModuleZeroPoint());//For setup of swerve
    SmartDashboard.putData("Drive Module 0", new DriveOneModule(0));//For setup of swerve
    SmartDashboard.putData("Drive Module 1", new DriveOneModule(1));//For setup of swerve
    SmartDashboard.putData("Drive Module 2", new DriveOneModule(2));//For setup of swerve
    SmartDashboard.putData("Drive Module 3", new DriveOneModule(3));//For setup of swerve
    SmartDashboard.putData(new DriveAllModulesPositionOnly());
    SmartDashboard.putData(new DriveStopAllModules());//For setup of swerve
    SmartDashboard.putData(new HoodHome(true));//For setup, leave for drives to use
    SmartDashboard.putData("Climber Home", new SequentialCommandGroup(
                            new ClimberTiltIn(),
                            new ClimberExtendoHome(),
                            climbCommand.resetCommandCommand()));//for setup
    SmartDashboard.putData(new LimeLightPowerCycle());//allows the drivers to restart the Limelight at will(good for hangups)
    // SmartDashboard.putData(new DriveFindMaxAccel());//This is for tuning acceleration constants
    // SmartDashboard.putData(new DriveTuneDriveMotorFeedForward(1.0));//this is for Velocity PID parameters
    SmartDashboard.putData(new ShooterSetSpeed(this::speedIShoot));//Testing Shooter
    SmartDashboard.putData(new HoodToPosition(this::angleIShoot));//Testing Shooter
    SmartDashboard.putData(new ShooterStop());//Testing Shooter
    // SmartDashboard.putData(new DriveTurnToAngle(Constants.PI_OVER_TWO));//for testing turn to angle function
    // SmartDashboard.putData(new AutoDriveStraightBackAndShootHigh(1.5));//AutoTesting
    // SmartDashboard.putData(new AutoMidTwoBall());//AutoTesting
    // SmartDashboard.putData(new AutoRightThreeBall());//AutoTesting
    // SmartDashboard.putData(new AutoPartnerPickupLeftBall());
    
    SmartDashboard.putData(climbCommand);//this is for the LED to see that this command is running


    SmartDashboard.putNumber("SpeedIShoot",0.0);
    SmartDashboard.putNumber("angleIShoot",0.0);

    // SmartDashboard.putData(new AutoRightFiveBall());

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* ==================== DRIVER BUTTONS ==================== */
    driverA.whenPressed(new BallHandlerSetState(State.kFillTo1));
    driverA.whenReleased(new BallHandlerSetState(State.kOff));
    driverB.whenPressed(new SpitBalls());
    driverB.whenReleased(new StopShooterHandlerHood());
    driverX.whileHeld(new ShootLowGoalFender(.5));
    driverX.whenReleased(new StopShooterHandlerHood());
    driverY.whenPressed(new ShootHighGoal(0.0));
    driverY.whenReleased(new StopShooterHandlerHood());
    driverLB.and(climberCommandRunning.negate()).whenActive(new DriveResetGyroToZero());
    driverRB.and(climberCommandRunning.negate()).whileActiveContinuous(new DriveOnTarget(4));
    driverBack.and(climberCommandRunning.negate()).toggleWhenActive(new DriveRobotCentric()); 
    // driverStart.whenPressed(new AutoMidFourBall());
    driverDUp.whenPressed(new ShootHighFender(0.1));
    driverDDown.whenPressed(new ShootHighFenderWithDriveBack(0.1));
    driverDDown.whenReleased(new StopShooterHandlerHood());
    driverDRight.whenPressed(new BallHandlerIntakeOut());
    driverDLeft.whenPressed(new BallHandlerIntakeIn());
    // driverStart.whileHeld(new SequentialCommandGroup(
    //   new DriveTurnToAngle(Math.toRadians(-135)).withTimeout(2.5),
    //   new WaitCommand(.5),
    //   new DriveFollowTrajectory("DriveLeftToOppBallShoot")
    // ));

    /* =================== CODRIVER BUTTONS =================== */
    coDriverB.and(climberCommandRunning.negate()).whenActive(new BallHandlerSetState(State.kSpitMid0))//spit ball0
    .whenInactive(new BallHandlerSetState(State.kOff));
    coDriverX.whenPressed(new PrepLowFender());//prep low goal, aka hood to position and shooter to speed
    coDriverY.whenPressed(new ShooterPrepShot());//prep high goal, aka shooter to a starter speed

    coDriverRB.and(coDriverDUp).whenActive(climbCommand.iterateFowardCommand());
    coDriverLB.and(coDriverDUp).whenActive(climbCommand.iterateBackwardCommand());

    coDriverBack.and(coDriverStart).whenActive(new ParallelCommandGroup(
      climbCommand,
      new DriveRobotCentricNoGyro())
      );

    coDriverLS.and(coDriverRS).whenActive(new ClimberTiltIn());
    coDriverDDown.whenPressed(new SequentialCommandGroup(
      new ClimberTiltIn(),
      new ClimberExtendoHome(),
      climbCommand.resetCommandCommand())
      );

    coDriverLTButton25.and(coDriverRTButton25).whileActiveContinuous(new ClimberExtendBothIn());
    coDriverRTButton70.and(coDriverLTButton25.negate()).whileActiveContinuous(new ClimberExtendRightIn());
    coDriverLTButton70.and(coDriverRTButton25.negate()).whileActiveContinuous(new ClimberExtendLeftIn());

    
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
   * They will appear in the order entered
   */
  private void configureAutoModes() {
    
    autoChooser.setDefaultOption("Wait 1 sec(do nothing)", new WaitCommand(1));
    autoChooser.addOption("Drives backwards 1.5 robot, shoots", new AutoDriveBackAndShoot(1.5));
    autoChooser.addOption("Drives backwards 2.5 robot, shoots", new AutoDriveBackAndShoot(2.5));
    autoChooser.addOption("Left Two Ball", new AutoLeftTwoBall());
    autoChooser.addOption("Mid Two Ball", new AutoMidTwoBall());
    autoChooser.addOption("Right Two Ball", new AutoRightTwoBall());
    autoChooser.addOption("Right Three Ball", new AutoRightThreeBall());
    autoChooser.addOption("Mid Four Ball", new AutoMidFourBall());
    autoChooser.addOption("Left Four Ball", new AutoLeftFourBall());
    autoChooser.addOption("Hungry Hungry Hippo", new AutoPartnerPickupLeftBall());
    autoChooser.addOption("Drive Left Steal Opp Ball Shoot", new AutoLeftTwoBallThenOppToss());
    // autoChooser.addOption("5 ball", new AutoRightFiveBall());


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
