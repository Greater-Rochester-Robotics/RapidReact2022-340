// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

    public static final double RAD_TO_ENC_CONV_FACTOR = 5029.08073;//swerveX 8T to 24T / 14T to 72T //the radian to enc factor
    public static final double DRIVE_ENC_TO_METERS_FACTOR = 0.00002153;//7.13:1//the ratio from mechanical specs
    public static final double PI_OVER_TWO = Math.PI/2;
    public static final double THREE_PI_OVER_TWO = 3*PI_OVER_TWO;
    public static final double TWO_PI = 2*Math.PI;

    /* Swerve Module Positions */
    public static final Translation2d FRONT_LEFT_POSITION = new Translation2d(.2,.2);
    public static final Translation2d FRONT_RIGHT_POSITION = new Translation2d(-.2,.2);
    public static final Translation2d REAR_LEFT_POSITION = new Translation2d(-.2,-.2);
    public static final Translation2d REAR_RIGHT_POSITION = new Translation2d(.2,-.2); 

    /* Swerve Module Drive Motor Constants */
    public static final double MINIMUM_DRIVE_SPEED = 0.01;// the slowest the wheels can turn, in m/s
    public static final double MINIMUM_DRIVE_DUTY_CYCLE = 0.05;// the slowest the wheels can turn, in duty cycle
    public static final double MAXIMUM_VELOCITY = 4.5;
    public static final double MAX_ROBOT_ROT_VELOCITY = 2;
    public static final double MAXIMUM_ACCELERATION = 1.0;
    // public static final double MAX_ROBOT_ROT_VELOCITY = MAXIMUM_VELOCITY / DISTANCE_TO_MODULE_0;
    public static final double MAXIMUM_VOLTAGE = 12.0;
    public static final double SWERVE_DRIVE_P_VALUE = 1000; // 0.035;
    public static final double SWERVE_DRIVE_I_VALUE = 0.0;
    public static final double SWERVE_DRIVE_D_VALUE = 25;
    public static final double SWERVE_DRIVE_FF_VALUE = 1023 / (MAXIMUM_VELOCITY / DRIVE_ENC_TO_METERS_FACTOR);

    /* Swerve Module Rotation constants */
    public static final double SWERVE_ROT_P_VALUE = 0.4;// TODO:find this value for real, .4 was from 2021
    public static final double SWERVE_ROT_I_VALUE = 0.0;
    public static final double SWERVE_ROT_D_VALUE = 0.0; 
    public static final double SWERVE_ROT_I_ZONE_VALUE = 0;
    public static final double SWERVE_ROT_FF_VALUE = 0.0;
    public static final double SWERVE_ROT_ARB_FF_VOLTAGE = 0.0;//This is left over from NEO550
    public static final double SWERVE_ROT_PID_VOLTAGE_MINIMUM = -12.0;//This is left over from NEO550
    public static final double SWERVE_ROT_PID_VOLTAGE_MAXIMUM = 12.0;//This is left over from NEO550
    public static final double SWERVE_MODULE_TOLERANCE = .051;

    /* Robot Motion PID controller constants */
    public static final double ROBOT_SPIN_P = 1;//TODO:this can't be the number, adjust when have robot
    public static final double ROBOT_SPIN_I = 0;
    public static final double ROBOT_SPIN_D = 0;

    /* Driver Scaling Constants */
    public static final double DRIVER_SPEED_SCALE_LINEAR = -.6;
    public static final double DRIVER_SPEED_SCALE_ROTATIONAL = .6;

    // CTRE motor and sensors
    public static final int FRONT_LEFT_MOVE_MOTOR = 40;// module 0
    public static final int FRONT_LEFT_ROTATE_MOTOR = 41;// module 0
    public static final int FRONT_LEFT_ROTATE_SENSOR = 42;// module 0

    public static final int REAR_LEFT_MOVE_MOTOR = 43;// module 1
    public static final int REAR_LEFT_ROTATE_MOTOR = 44;// module 1
    public static final int REAR_LEFT_ROTATE_SENSOR = 45;// module 1

    public static final int REAR_RIGHT_MOVE_MOTOR = 46;// module 2
    public static final int REAR_RIGHT_ROTATE_MOTOR = 47;// module 2
    public static final int REAR_RIGHT_ROTATE_SENSOR = 48;// module 2
    
    public static final int FRONT_RIGHT_MOVE_MOTOR = 49;// module 3
    public static final int FRONT_RIGHT_ROTATE_MOTOR = 50;// module 3
    public static final int FRONT_RIGHT_ROTATE_SENSOR = 51;// module 3

    public static final double MIN_PRESSURE = 95.0;
    public static final double MAX_PRESSURE = 120.0;

    // Solenoid Channels
    public static final int HARVESTER_TILT_IN = 0;
    public static final int HARVESTER_TILT_OUT = 1;
    public static final int CLIMBER_TILT_IN = 2;
    public static final int CLIMBER_TILT_OUT = 3;

    // Harvester Motors
    public static final int HARVESTER_MOTOR = 0;
    public static final double HARVESTER_INTAKE_SPEED = 0.6;
    public static final double HARVESTER_EXTAKE_SPEED = -0.6;

    // Shooter Motors
    public static final int MAIN_SHOOTER_MOTOR = 0;
    public static final int FOLLOW_SHOOTER_MOTOR = 1;

    // Climber Motors
    public static final int CLIMBER_FIXED_ARM = 2;
    public static final int CLIMBER_EXTENDO_ARM = 3;
    public static final double CLIMBER_EXTENDO_SPEED_OUT = 0.6;
    public static final double CLIMBER_EXTENDO_SPEED_IN = -0.6;
    public static final double CLIMBER_FIXED_SPEED_OUT = 0.6;
    public static final double CLIMBER_FIXED_SPEED_IN = -0.6;

}
