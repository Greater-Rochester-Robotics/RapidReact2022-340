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

    public static final double RAD_TO_ENC_CONV_FACTOR = 10.1859;//TODO: fix for swerveX //the radian to enc factor
    public static final double DRIVE_ENC_TO_METERS_FACTOR = 0.00002226;//TODO:Get correct value from SwerveX//the ratio from mechanical specs
    public static final double PI_OVER_TWO = Math.PI/2;
    public static final double THREE_PI_OVER_TWO = 3*PI_OVER_TWO;
    public static final double TWO_PI = 2*Math.PI;

    public static final Translation2d FRONT_LEFT_POSITION = new Translation2d(.2,.2);
    public static final Translation2d FRONT_RIGHT_POSITION = new Translation2d(-.2,.2);
    public static final Translation2d REAR_LEFT_POSITION = new Translation2d(-.2,-.2);
    public static final Translation2d REAR_RIGHT_POSITION = new Translation2d(.2,-.2); 

    /* Swerve Drive Constants */
    public static final double MINIMUM_DRIVE_SPEED = 0.01;// the slowest the wheels can turn, in m/s
    public static final double MINIMUM_DRIVE_DUTY_CYCLE = 0.05;// the slowest the wheels can turn, in duty cycle
    public static final double MAXIMUM_VELOCITY = 4.5;
    public static final double MAXIMUM_ACCELERATION = 1.0;
    // public static final double MAX_ROBOT_ROT_VELOCITY = MAXIMUM_VELOCITY / DISTANCE_TO_MODULE_0;
    public static final double MAXIMUM_VOLTAGE = 12.0;
    public static final double SWERVE_DRIVE_P_VALUE = 1000; // 0.035;
    public static final double SWERVE_DRIVE_I_VALUE = 0.0;
    public static final double SWERVE_DRIVE_D_VALUE = 25;
    public static final double SWERVE_DRIVE_F_VALUE = 1023 / (MAXIMUM_VELOCITY / DRIVE_ENC_TO_METERS_FACTOR);

    /* Swerve Module Rotation constants */
    public static final double SWERVE_ROT_P_VALUE = 0.4;// 2if sluggish, increase P value
    public static final double SWERVE_ROT_I_VALUE = 0.0;
    public static final double SWERVE_ROT_D_VALUE = 0.0; // 4
    public static final double SWERVE_ROT_I_ZONE_VALUE = 0;
    public static final double SWERVE_ROT_NONARB_FF_VALUE = 0.0;//.0001;//Not arbitrary, this is multiplied by setpoint, must be 0 in position PID
    public static final double SWERVE_ROT_ARB_FF_VOLTAGE = 0.0;//1.1;
    public static final double SWERVE_ROT_PID_VOLTAGE_MINIMUM = -12.0;
    public static final double SWERVE_ROT_PID_VOLTAGE_MAXIMUM = 12.0;
    public static final double SWERVE_MODULE_TOLERANCE = .051;



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

}
