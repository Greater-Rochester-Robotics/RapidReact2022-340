// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import com.analog.adis16470.frc.ADIS16470_IMU.ADIS16470CalibrationTime;
import com.analog.adis16470.frc.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDrive extends SubsystemBase {

  private static SwerveModule swerveModules[];
  private static SwerveModule frontLeft, rearLeft, rearRight, frontRight;
  public ADIS16470_IMU imu;
  private SwerveDriveKinematics driveKinematics;


  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    //adds CANCoder address as third param, already named as rotation sensor in constants
    // Constructs the swerve modules 
    frontLeft = new SwerveModule(Constants.FRONT_LEFT_MOVE_MOTOR, Constants.FRONT_LEFT_ROTATE_MOTOR, Constants.FRONT_LEFT_ROTATE_SENSOR);
    rearLeft = new SwerveModule(Constants.REAR_LEFT_MOVE_MOTOR, Constants.REAR_LEFT_ROTATE_MOTOR, Constants.REAR_LEFT_ROTATE_SENSOR);
    rearRight = new SwerveModule(Constants.REAR_RIGHT_MOVE_MOTOR, Constants.REAR_RIGHT_ROTATE_MOTOR, Constants.REAR_RIGHT_ROTATE_SENSOR);
    frontRight = new SwerveModule(Constants.FRONT_RIGHT_MOVE_MOTOR, Constants.FRONT_RIGHT_ROTATE_MOTOR, Constants.FRONT_RIGHT_ROTATE_SENSOR);
    
     //This may seem repetitive, but it makes clear which module is which.
    swerveModules = new SwerveModule[]{
      frontLeft,
      rearLeft,
      rearRight,
      frontRight
    };
    
    // Constructs IMU object
    imu = new ADIS16470_IMU(IMUAxis.kY, SPI.Port.kOnboardCS0, ADIS16470CalibrationTime._4s);
    driveKinematics = new SwerveDriveKinematics(
      Constants.FRONT_LEFT_POSITION, Constants.REAR_LEFT_POSITION, 
      Constants.REAR_RIGHT_POSITION, Constants.FRONT_RIGHT_POSITION);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


    /**
   * Stops all module motion, then lets all the modules spin freely.
   */
  public void stopAllModules(){
    for (int i=0; i<4; i++){
      swerveModules[i].stopAll();
    }
  }

  /**
   * A function that allows the user to reset the gyro, this 
   * makes the current orientation of the robot 0 degrees on 
   * the gyro.
   */
  public void resetGyro(){
    //Resets the gyro(zero it)
    imu.reset();
  }

  /**
   * This calls the drive Gyro and returns the Rotation2d object.
   * This object contains angle in radians, as well as the sin 
   * and cos of that angle. This is an object that represents the
   * rotation of the robot.
   * @return a Rotation2d object
   */
  public Rotation2d getGyroRotation2d(){
    //return a newly constructed Rotation2d object, it takes the angle in radians as a constructor argument
    return new Rotation2d(getGyroInRad());
    //note that counterclockwise rotation is positive
  }

  /**
   * This polls the onboard gyro, which, when the robot boots,
   * assumes and angle of zero, this needs to be positive when
   * turning left
   * @return the angle of the robot in radians
   */
  public double getGyroInRad(){
    return Math.toRadians(getGyroInDeg());// Pull the gyro in degrees, convert and return in radians
    //note that counterclockwise rotation is positive
  }

  /**
   * This polls the onboard gyro, which, when the robot boots,
   * assumes and angle of zero, this needs to be positive when
   * turning left
   * @return the angle of the robot in degrees
   */
  public double getGyroInDeg(){
    return imu.getAngle();//getYawAxis();//*-1;//Pull gyro in degrees
    //note counterclockwise rotation is positive
  }

  public double[] getAllAbsModuleAngles(){
    double[] moduleAngles = new double[4];
    for(int i=0; i<4; i++){
      moduleAngles[i]=swerveModules[i].getAbsPosInDeg();
    }
    return moduleAngles;
  }

  public double[] getAllModuleRelEnc(){
    double[] moduleRelEnc = new double[4];
    for(int i=0; i<4; i++){
      moduleRelEnc[i]=swerveModules[i].getRelEncCount();
    }
    return moduleRelEnc;
  }

  /**
   * 
   * @return
   */
  public double[] getAllModuleDistance(){
    double[] moduleDistances = new double[4];
    for(int i=0; i<4; i++){
      moduleDistances[i]=swerveModules[i].getDriveDistance();
    }
    return moduleDistances;
  }

  /**
   *  Gets all the drive velocities.
   * 
   * @return An array of velocities.
   */
  public double[] getAllModuleVelocity(){
    double[] moduleVelocities = new double[4];
    for(int i=0; i<4; i++){
      moduleVelocities[i]=swerveModules[i].getDriveVelocity();
    }
    return moduleVelocities;
  }

  /**
   * a method to print all module positions for testing purposes
   */
  public void printAllModuleAngles(){
    //Use a for loop to and print() all modules' angles(degrees) on one line  
    System.out.print("Angle = ");
  
    for(int i=0; i<4; i++){
      System.out.print(swerveModules[i].getAbsPosInDeg()+"\t");
    }
    //make sure to newline "\n" at the end
    System.out.print("\n");
  }
  
  /**
   * Method for taking the current position of all modules,
   * and making that position the absolute zero of each 
   * modules position respectively.
   */
  public void zeroAllModulePosSensors(){
    //a for loop so cycle through all modules
    for (int i=0; i<4; i++){
      //call the zero position method
      swerveModules[i].zeroAbsPositionSensor();
    }
  }
}
