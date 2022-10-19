// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    /* Factors of PI */
    public static final double PI_OVER_TWO = Math.PI/2;
    public static final double THREE_PI_OVER_TWO = 3*PI_OVER_TWO;
    public static final double TWO_PI = 2*Math.PI;
    public static final Rotation2d ROTATE_BY_PI = Rotation2d.fromDegrees(180);//I only make this once

    /* Swerve Module Positions */
    public static final Translation2d FRONT_LEFT_POSITION = new Translation2d(.3016,.3016);//These are in meters
    public static final Translation2d REAR_LEFT_POSITION = new Translation2d(-.3016,.3016);
    public static final Translation2d REAR_RIGHT_POSITION = new Translation2d(-.3016,-.3016);
    public static final Translation2d FRONT_RIGHT_POSITION = new Translation2d(.3016,-.3016); 

    /* Swerve Module Drive Motor Constants */
    public static final double DRIVE_ENC_TO_METERS_FACTOR = 0.00002153;//7.13:1//the ratio from mechanical specs
    public static final double MINIMUM_DRIVE_SPEED = 0.01;// the slowest the wheels can turn, in m/s
    public static final double MINIMUM_DRIVE_DUTY_CYCLE = 0.05;// the slowest the wheels can turn, in duty cycle
    public static final double MOTOR_MAXIMUM_VELOCITY = 4.62;//4.33 5.19
    public static final double PATH_MAXIMUM_VELOCITY = 3.5;
    public static final double MAXIMUM_ACCELERATION = 1.25;

    public static final double MAX_ROBOT_ROT_VELOCITY = 2;

    // public static final double MAX_ROBOT_ROT_VELOCITY = MAXIMUM_VELOCITY / DISTANCE_TO_MODULE_0;
    public static final double MAXIMUM_VOLTAGE = 12.0;
    public static final double SWERVE_DRIVE_P_VALUE = 1000; // 0.035;
    public static final double SWERVE_DRIVE_I_VALUE = 0.0;
    public static final double SWERVE_DRIVE_D_VALUE = 25;
    public static final double SWERVE_DRIVE_FF_VALUE = 1023 / (MOTOR_MAXIMUM_VELOCITY / DRIVE_ENC_TO_METERS_FACTOR);

    /* Swerve Module Rotation constants */
    public static final double RAD_TO_ENC_CONV_FACTOR = 5029.08073;//swerveX 8T to 24T / 14T to 72T //the radian to enc factor
    public static final double SWERVE_ROT_P_VALUE = 0.1;
    public static final double SWERVE_ROT_I_VALUE = 0.0;
    public static final double SWERVE_ROT_D_VALUE = 0.05; 
    public static final double SWERVE_ROT_I_ZONE_VALUE = 0;
    public static final double SWERVE_ROT_FF_VALUE = 0.0;
    // public static final double SWERVE_ROT_ARB_FF_VOLTAGE = 0.0;//This is left over from NEO550 consider deleting
    // public static final double SWERVE_ROT_PID_VOLTAGE_MINIMUM = -12.0;//This is left over from NEO550 consider deleting
    // public static final double SWERVE_ROT_PID_VOLTAGE_MAXIMUM = 12.0;//This is left over from NEO550 consider deleting
    public static final double SWERVE_MODULE_TOLERANCE = 0.1;
    public static final double ROTATIONAL_VELOCITY_TOLERANCE = 1.0;

    /* Robot Rotation PID controller constants */
    public static final double ROBOT_SPIN_PID_TOLERANCE = Math.toRadians(0.5);
    public static final double MINIMUM_ROTATIONAL_OUTPUT = 0.10;

    public static final double ROBOT_SPIN_P = 1.55;//tuned for drive/climber bot
    public static final double ROBOT_SPIN_I = 0.0;
    public static final double ROBOT_SPIN_D = 0.01;
    
    public static final double ROBOT_COUNTER_SPIN_P = 1.1;
    public static final double ROBOT_COUNTER_SPIN_I = 0.0;
    public static final double ROBOT_COUNTER_SPIN_D = 0.001;

    /* We stole 3015's constants for DriveFollowTrajectory */
    public static final double DRIVE_POS_ERROR_CONTROLLER_P = .33; // 10
    public static final double DRIVE_POS_ERROR_CONTROLLER_I = 0.001;
    public static final double DRIVE_POS_ERROR_CONTROLLER_D = 0.0;//0.05;
    // public static final double DRIVE_HEADING_ERROR_CONTROLLER_P = 0; // 1.05
    // public static final double DRIVE_HEADING_ERROR_CONTROLLER_I = 0;
    // public static final double DRIVE_HEADING_ERROR_CONTROLLER_D = 0; // 0.02
    public static final double DRIVE_ROTATION_CONTROLLER_P = 1.6*MOTOR_MAXIMUM_VELOCITY;//.1396;// 9
    public static final double DRIVE_ROTATION_CONTROLLER_I = 0.0;
    public static final double DRIVE_ROTATION_CONTROLLER_D = 0.01;
    public static final double DRIVE_MAX_ANGULAR_VELOCITY = 13.5;//10.8;//PathFollowing
    public static final double DRIVE_MAX_ANGULAR_ACCEL = 8.5;//7.03;//PathFollowing
    // public static final double DRIVE_ROTATION_MIN_VELOCITY = 25;

    /* Driver Scaling Constants */
    public static final double DRIVER_SPEED_SCALE_LINEAR = 0.65;
    public static final double DRIVER_SPEED_SCALE_ROTATIONAL = .75;

    /* Limelight Values*/
    public static final double LL_ANGLE_TOLERANCE = 1.0;
    public static final double LL_TARGET_HEIGHT = 104.0;
    public static final double LL_MOUNT_HEIGHT = 33.0;
    public static final double LL_TARGET_TO_LL_HEIGHT = LL_TARGET_HEIGHT - LL_MOUNT_HEIGHT;
    public static final double LL_TARGET_RADIUS = 26.69;
    public static final double LL_TARGET_TO_FENDER = 7.2;
    public static final double LL_DISTANCE_TO_FRONT = 4.5;
    public static final double LL_MOUNT_ANGLE = 35.5;
    public static final double LL_DISTANCE_TO_ROBOT_CENTER = 0.0;

    /* Aiming Values*/
    public static final Translation2d FIELD_CENTER = new Translation2d();
    
    /* Harvester Constants */
    public static final double HARVESTER_INTAKE_SPEED = 0.8;
    public static final double HARVESTER_EXTAKE_SPEED = -0.6;

    public static final double BALL_HANDLER_0_INTAKE_SPEED = 0.55;
    public static final double BALL_HANDLER_0_EXTAKE_SPEED = -0.6;
    public static final double BALL_HANDLER_1_INTAKE_SPEED = 0.6;
    public static final double BALL_HANDLER_1_EXTAKE_SPEED = -0.6;
    public static final double BALL_HANDLER_2_EXTAKE_SPEED = -0.6;
    public static final double BALL_HANDLER_2_SHOOT_SPEED = 0.6;
    public static final double BALL_HANDLER_1_SHOOT_SPEED = 0.6;
    public static final double BALL_HANDLER_0_SHOOT_SPEED = 0.5;

    /* Intake Levels */
    public static final int BLUE_LEVEL = -100;
    public static final int RED_LEVEL = -100;
    
    public static final int INTAKE_PROXIMITY = 500;

    /* Shooter Constants */
    public static final int SHOOTER_MOTOR_PULSES_PER_REV = 1;//5000; //No counts per revolution, keep native

    public static final double SHOOTER_MAIN_MOTOR_P = 1.75;//1.8;
    public static final double SHOOTER_MAIN_MOTOR_I = 0.0;
    public static final double SHOOTER_MAIN_MOTOR_D = 2.0;//4.0;
    public static final double SHOOTER_MAIN_MOTOR_F = 0.0;

    public static final double SHOOTER_MOTOR_ALLOWABLE_ERROR = 200.0;//TODO: get a better number here, and this is in pulses, but need to fix PID before changing
    
    public static final double SHOOOTER_PREP_SPEED = 7300;
    public static final double SHOOTER_FENDER_SHOT_SPEED = 7400;
    public static final LookUpTable SHOOTER_HIGH_SPEEDS_TABLE = new LookUpTable();
    static {
        //use put(distance, speed)
        SHOOTER_HIGH_SPEEDS_TABLE.put(260, 12000);
        SHOOTER_HIGH_SPEEDS_TABLE.put(220, 10600);
        SHOOTER_HIGH_SPEEDS_TABLE.put(180, 10200);
        SHOOTER_HIGH_SPEEDS_TABLE.put(153.25, 9600);
        SHOOTER_HIGH_SPEEDS_TABLE.put(130, 9200);
        SHOOTER_HIGH_SPEEDS_TABLE.put(115, 9000);
        SHOOTER_HIGH_SPEEDS_TABLE.put(92, 8500);
        SHOOTER_HIGH_SPEEDS_TABLE.put(75.25, 8150);
        SHOOTER_HIGH_SPEEDS_TABLE.put(65.75, 8000);
        SHOOTER_HIGH_SPEEDS_TABLE.put(43.75, 7800);
        SHOOTER_HIGH_SPEEDS_TABLE.put(24.0, 7700);
    }
    public static final LookUpTable SHOOTER_LOW_SPEEDS_TABLE = new LookUpTable();
    static {
        //use put(distance, speed)
        SHOOTER_LOW_SPEEDS_TABLE.put(5, 10000);
       
    }
    public static final double SHOOTER_LOW_GOAL_FENDER_SPEED = 4000.0;

    /* Hood Constants */
    public static final double HOOD_DEGREE_CONVERSION = 30.0 * 32.0 / 35.0 / 51.0;// gear ratio on the versa, 5:1+7:1 ratio w/ 30T gear, 51T rack, 32 degrees
    public static final double HOOD_FORWARD_LIMIT_DEGREES = 22;
    public static final double HOOD_POS_ALLOWABLE_ERROR = 0.5;
    
    public static final double HOOD_MOTOR_P = 0.5;//This seems to work
    public static final double HOOD_MOTOR_I = 0.0;
    public static final double HOOD_MOTOR_D = 0.0;
    public static final double HOOD_MOTOR_FF = 0.0;

    public static final double HOOD_HOMING_SPEED = -.10;
    public static final LookUpTable HOOD_HIGH_POSITION_TABLE = new LookUpTable();
    static {
        //use put(distance, angle)
        HOOD_HIGH_POSITION_TABLE.put(260, 19);
        HOOD_HIGH_POSITION_TABLE.put(220, 18);
        HOOD_HIGH_POSITION_TABLE.put(180, 17.5);
        HOOD_HIGH_POSITION_TABLE.put(153.25, 16);
        HOOD_HIGH_POSITION_TABLE.put(130, 14);
        HOOD_HIGH_POSITION_TABLE.put(115,14);
        HOOD_HIGH_POSITION_TABLE.put(92,12.5);
        HOOD_HIGH_POSITION_TABLE.put(75.25,11);
        HOOD_HIGH_POSITION_TABLE.put(65.75,11);
        HOOD_HIGH_POSITION_TABLE.put(43.75,4);
        HOOD_HIGH_POSITION_TABLE.put(23.5,0);
    }
    public static final LookUpTable HOOD_LOW_POSITION_TABLE = new LookUpTable();
    static {
        //use put(distance, angle)
        HOOD_LOW_POSITION_TABLE.put(5, 8);
       
    }

    /* Climber Constants */
    public static final double CLIMBER_EXTENDO_SPEED_OUT = 0.4;//This is for testing up 
    public static final double CLIMBER_EXTENDO_SPEED_IN = -0.6;//This is for testing down and homing
    public static final double CLIMBER_EXTENDO_FORCE_IN = -0.7;

    public static final double EXTENDO_MOTOR_P = 1.0;
    public static final double EXTENDO_MOTOR_I = 0.0;
    public static final double EXTENDO_MOTOR_D = 1.5;
    public static final double EXTENDO_MOTOR_F = 0.0;
    public static final double EXTENDO_MOTOR_P_COMPENSATE = 0.1;
    public static final double EXTENDO_MOTOR_I_COMPENSATE = 0.0;
    public static final double EXTENDO_MOTOR_D_COMPENSATE = 0.0;
    public static final double EXTENDO_MOTOR_F_COMPENSATE = 0.0;

    public static final double EXTENDO_CRUISE_VELOCITY = 15.0;
    public static final double EXTENDO_INCHES_PER_PULSE_CONVERSION_FACTOR = 5.105 / 24576;
    public static final double EXTENDO_ACCELERATION = 6.0;
    public static final double EXTENDO_HOMING_CURRENT = 20.0;
    public static final double EXTENDO_ALLOWABLE_ERROR = 0.25;

    public static final double CLIMBER_TOP_POSITION = 23;
    public static final double CLIMBER_LIFT_POSITION = 12;
    public static final double CLIMBER_RELEASE_POSITION = 4;
    public static final double CLIMBER_BOTTOM_POSITION = -.9;

    public static final int SELECTED_FEEDBACK_COEFFICIENT = 1;

    /* Compressor Pressure Constants */
    public static final double MIN_PRESSURE = 100.0;
    public static final double MAX_PRESSURE = 120.0;//TODO: change this on between comp bot and prac , 100 for prac, 120 for comp

    /* IDENTIFICATION NUMBERS FOR DEVICES */

    /* CTRE motor and sensors */
    public static final int CLIMBER_LEFT_ARM = 52;//Climber
    public static final int CLIMBER_RIGHT_ARM = 53;//Climber

    public static final int SHOOTER_SHOOTING_MOTOR = 20;//Shooter

    public static final int HARVESTER_MOTOR = 21;//Handler

    public static final int FRONT_LEFT_MOVE_MOTOR = 3;//drive module 0
    public static final int FRONT_LEFT_ROTATE_MOTOR = 4;//drive module 0
    public static final int FRONT_LEFT_ROTATE_SENSOR = 5;//drive module 0

    public static final int REAR_LEFT_MOVE_MOTOR = 6;//drive module 1
    public static final int REAR_LEFT_ROTATE_MOTOR = 7;//drive module 1
    public static final int REAR_LEFT_ROTATE_SENSOR = 8;//drive module 1

    public static final int REAR_RIGHT_MOVE_MOTOR = 9;//drive module 2
    public static final int REAR_RIGHT_ROTATE_MOTOR = 10;//drive module 2
    public static final int REAR_RIGHT_ROTATE_SENSOR = 11;//drive module 2
    
    public static final int FRONT_RIGHT_MOVE_MOTOR = 12;//drive module 3
    public static final int FRONT_RIGHT_ROTATE_MOTOR = 13;//drive module 3
    public static final int FRONT_RIGHT_ROTATE_SENSOR = 14;//drive module 3

    /* Rev Robotics SparkMAXs */
    public static final int BALL_HANDLER_MOTOR_0 = 15;//Handler
    public static final int BALL_HANDLER_MOTOR_1 = 16;//Handler
    public static final int BALL_HANDLER_MOTOR_2 = 17;
    public static final int SHOOTER_HOOD_MOTOR = 18; //Shooter

    /* Solenoid Channels */
    public static final int HARVESTER_TILT_IN = 8;//Intake
    public static final int HARVESTER_TILT_OUT = 9;//Intake
    public static final int CLIMBER_TILT_IN = 13;//Climber
    public static final int CLIMBER_TILT_OUT = 12;//Climber
    public static final int CLIMBER_DAMPENING_BAR = 15;//Climber
    public static final int LL_LIGHT = 0;//Limelight

    /* Digital Input */
    public static final int BALL_SENSOR_0 = 0;
    public static final int BALL_SENSOR_1 = 1;
    public static final int SHOOTER_HOOD_SWITCH = 2;

}
