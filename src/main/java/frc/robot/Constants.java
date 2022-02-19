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
    public static final Rotation2d ROTATE_BY_PI = Rotation2d.fromDegrees(180);

    /* Swerve Module Positions */
    public static final Translation2d FRONT_LEFT_POSITION = new Translation2d(.3016,.3016);//These are in meters
    public static final Translation2d REAR_LEFT_POSITION = new Translation2d(-.3016,.3016);
    public static final Translation2d REAR_RIGHT_POSITION = new Translation2d(-.3016,-.3016);
    public static final Translation2d FRONT_RIGHT_POSITION = new Translation2d(.3016,-.3016); 

    /* Swerve Module Drive Motor Constants */
    public static final double DRIVE_ENC_TO_METERS_FACTOR = 0.00002153;//7.13:1//the ratio from mechanical specs
    public static final double MINIMUM_DRIVE_SPEED = 0.01;// the slowest the wheels can turn, in m/s
    public static final double MINIMUM_DRIVE_DUTY_CYCLE = 0.05;// the slowest the wheels can turn, in duty cycle
    public static final double MAXIMUM_VELOCITY = 4.5;
    public static final double MAX_ROBOT_ROT_VELOCITY = 2;
    public static final double MAXIMUM_ACCELERATION = 1.0;
    public static final double DRIVE_MAX_ANGULAR_VELOCITY = 600; //stolen from 3015
    public static final double DRIVE_MAX_ANGULAR_ACCEL = 6000; //stolen from 3015
    // public static final double MAX_ROBOT_ROT_VELOCITY = MAXIMUM_VELOCITY / DISTANCE_TO_MODULE_0;
    public static final double MAXIMUM_VOLTAGE = 12.0;
    public static final double SWERVE_DRIVE_P_VALUE = 1000; // 0.035;
    public static final double SWERVE_DRIVE_I_VALUE = 0.0;
    public static final double SWERVE_DRIVE_D_VALUE = 25;
    public static final double SWERVE_DRIVE_FF_VALUE = 1023 / (MAXIMUM_VELOCITY / DRIVE_ENC_TO_METERS_FACTOR);

    /* Swerve Module Rotation constants */
    public static final double RAD_TO_ENC_CONV_FACTOR = 5029.08073;//swerveX 8T to 24T / 14T to 72T //the radian to enc factor
    public static final double SWERVE_ROT_P_VALUE = 0.1;// TODO:find this value for real, .4 was from 2021
    public static final double SWERVE_ROT_I_VALUE = 0.0;
    public static final double SWERVE_ROT_D_VALUE = 0.05; 
    public static final double SWERVE_ROT_I_ZONE_VALUE = 0;
    public static final double SWERVE_ROT_FF_VALUE = 0.0;
    public static final double SWERVE_ROT_ARB_FF_VOLTAGE = 0.0;//This is left over from NEO550
    public static final double SWERVE_ROT_PID_VOLTAGE_MINIMUM = -12.0;//This is left over from NEO550
    public static final double SWERVE_ROT_PID_VOLTAGE_MAXIMUM = 12.0;//This is left over from NEO550
    public static final double SWERVE_MODULE_TOLERANCE = 0.1;//TODO: this is scaled from the NEO550, needs adjust for Falcon500

    /* Robot Motion PID controller constants */
    public static final double ROBOT_SPIN_P = 1.1;//tuned for drive/climber bot
    public static final double ROBOT_SPIN_I = 0;
    public static final double ROBOT_SPIN_D = 0.001;

    
    /* We stole 3015's constants for DriveFollowTrajectory */
    public static final double DRIVE_POS_ERROR_CONTROLLER_P = 12; // 10
    public static final double DRIVE_POS_ERROR_CONTROLLER_I = 0;
    public static final double DRIVE_POS_ERROR_CONTROLLER_D = 0.05;
    public static final double DRIVE_HEADING_ERROR_CONTROLLER_P = 0; // 1.05
    public static final double DRIVE_HEADING_ERROR_CONTROLLER_I = 0;
    public static final double DRIVE_HEADING_ERROR_CONTROLLER_D = 0; // 0.02
    public static final double DRIVE_ROTATION_CONTROLLER_P = 10;// 9
    public static final double DRIVE_ROTATION_CONTROLLER_I = 0;
    public static final double DRIVE_ROTATION_CONTROLLER_D = 0;
    public static final double DRIVE_TARGETING_CONTROLLER_P = 13;// 9
    public static final double DRIVE_TARGETING_CONTROLLER_I = 0;
    public static final double DRIVE_TARGETING_CONTROLLER_D = 0.5;
    public static final double DRIVE_ROTATION_MIN_VELOCITY = 25;
    public static final double DRIVE_TARGETING_I_ZONE = 2;

    /* Driver Scaling Constants */
    public static final double DRIVER_SPEED_SCALE_LINEAR = 0.4;
    public static final double DRIVER_SPEED_SCALE_ROTATIONAL = 1.5;

    
    /* Compressor Pressure Constants */
    public static final double MIN_PRESSURE = 95.0;
    public static final double MAX_PRESSURE = 120.0;

    /* Harvester Constants */
    public static final double HARVESTER_INTAKE_SPEED = 0.6;
    public static final double HARVESTER_EXTAKE_SPEED = -0.6;

    public static final double BALL_HANDLER_0_INTAKE_SPEED = 0.6;
    public static final double BALL_HANDLER_0_EXTAKE_SPEED = -0.6;
    public static final double BALL_HANDLER_1_INTAKE_SPEED = 0.6;
    public static final double BALL_HANDLER_1_EXTAKE_SPEED = -0.6;
    public static final double BALL_HANDLER_2_EXTAKE_SPEED = -0.6;
    public static final double BALL_HANDLER_2_SHOOT_SPEED = 0.6;
    public static final double BALL_HANDLER_1_SHOOT_SPEED = 0.6;
    public static final double BALL_HANDLER_0_SHOOT_SPEED = 0.5;

    /* Intake Levels */
    public static final int BLUE_LEVEL = 500;
    public static final int RED_LEVEL = 500;
    
    public static final int INTAKE_PROXIMITY = 500;

    /* Shooter Constants */
    public static final int SHOOTER_HOOD_MOTOR_COUNTS_PER_REV = 5000; //TODO: get actual counts per revolution
    public static final double SHOOTER_HOOD_DEGREE_CONVERSION = (30/7) * (32/51);// gear ratio on the versa, 7:1 ratio w/ 30T gear, 51T rack, 32 degrees
    
    public static final double SHOOTER_MAIN_MOTOR_P = 1.0;
    public static final double SHOOTER_MAIN_MOTOR_I = 0.0;
    public static final double SHOOTER_MAIN_MOTOR_D = 0.0;
    public static final double SHOOTER_MAIN_MOTOR_F = 0.0;

    public static final double SHOOTER_HOOD_MOTOR_P = 1.0;
    public static final double SHOOTER_HOOD_MOTOR_I = 0.0;
    public static final double SHOOTER_HOOD_MOTOR_D = 0.0;
    public static final double SHOOTER_HOOD_MOTOR_FF = 0.0;

    public static final double SHOOTER_MOTOR_ALLOWABLE_ERROR = 0.0;

    /* Climber Constants */
    public static final double CLIMBER_EXTENDO_SPEED_OUT = 0.3;//This is for testing up 
    public static final double CLIMBER_EXTENDO_SPEED_IN = -0.3;//This is for testing down and homing

    public static final double EXTENDO_MOTOR_P = 1.0;
    public static final double EXTENDO_MOTOR_I = 0.0;
    public static final double EXTENDO_MOTOR_D = 1.5;
    public static final double EXTENDO_MOTOR_F = 0.0;

    public static final double EXTENDO_CRUISE_VELOCITY = 15.0;
    public static final double EXTENDO_INCHES_PER_PULSE_CONVERSION_FACTOR = 5.105 / 24576;
    public static final double EXTENDO_ACCELERATION = 6.0;
    public static final double EXTENDO_HOMING_CURRENT = 20.0;
    public static final double EXTENDO_ALLOWABLE_ERROR = 0.25;

    public static final double CLIMBER_TOP_POSITION = 24;
    public static final double CLIMBER_BOTTOM_POSITION = 0;
    public static final double CLIMBER_MIDDLE_POSITION = 12;

    /* IDENTIFICATION NUMBERS FOR DEVICES */

    /* CTRE motor and sensors */
    public static final int CLIMBER_LEFT_ARM = 52;//Climber
    public static final int CLIMBER_RIGHT_ARM = 53;//Climber

    public static final int MAIN_SHOOTER_MOTOR = 4;//Shooter

    public static final int HARVESTER_MOTOR = 6;//Handler

    // TODO: Change all motor ids to 1-12 in order to increase priority
    public static final int FRONT_LEFT_MOVE_MOTOR = 40;//drive module 0
    public static final int FRONT_LEFT_ROTATE_MOTOR = 41;//drive module 0
    public static final int FRONT_LEFT_ROTATE_SENSOR = 42;//drive module 0

    public static final int REAR_LEFT_MOVE_MOTOR = 43;//drive module 1
    public static final int REAR_LEFT_ROTATE_MOTOR = 44;//drive module 1
    public static final int REAR_LEFT_ROTATE_SENSOR = 45;//drive module 1

    public static final int REAR_RIGHT_MOVE_MOTOR = 46;//drive module 2
    public static final int REAR_RIGHT_ROTATE_MOTOR = 47;//drive module 2
    public static final int REAR_RIGHT_ROTATE_SENSOR = 48;//drive module 2
    
    public static final int FRONT_RIGHT_MOVE_MOTOR = 49;//drive module 3
    public static final int FRONT_RIGHT_ROTATE_MOTOR = 50;//drive module 3
    public static final int FRONT_RIGHT_ROTATE_SENSOR = 51;//drive module 3

    /* Rev Robotics SparkMAXs */
    public static final int BALL_HANDLER_MOTOR_0 = 1;//Handler
    public static final int BALL_HANDLER_MOTOR_1 = 2;//Handler
    public static final int BALL_HANDLER_MOTOR_2 = 3;
    public static final int SHOOTER_HOOD_MOTOR = 4; //Shooter

    /* Solenoid Channels */
    public static final int HARVESTER_TILT_IN = 0;//Intake
    public static final int HARVESTER_TILT_OUT = 1;//Intake
    public static final int CLIMBER_TILT_IN = 2;//Climber
    public static final int CLIMBER_TILT_OUT = 3;//Climber

    /* Digital Input */
    public static final int BALL_SENSOR_0 = 0;
    public static final int BALL_SENSOR_1 = 1;
    public static final int SHOOTER_HOOD_SWITCH = 4;

    /* Aiming Values*/
    public static final Translation2d FIELD_CENTER = new Translation2d();

}
