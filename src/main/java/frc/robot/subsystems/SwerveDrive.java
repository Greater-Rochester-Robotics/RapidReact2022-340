// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import frc.robot.Constants;
import frc.robot.subsystems.ADIS.CalibrationTime;
import frc.robot.subsystems.ADIS.IMUAxis;
import frc.robot.subsystems.swervelib.SwerveModule;

/**
 * This subsystem contains all SwerveModule objects, and runs all drive functions.
 * 
 * More on swerve found here:
 * https://docs.google.com/presentation/d/1feVl0L5lgIKSZhKCheWgWhkOydIu-ibgdp7oqA0yqAQ/edit?usp=sharing
 */
public class SwerveDrive extends SubsystemBase {

  private static SwerveModule swerveModules[];
  private static SwerveModule frontLeft, rearLeft, rearRight, frontRight;
  public ADIS imu;
  private SwerveDriveKinematics driveKinematics;
  public SwerveDriveOdometry driveOdometry;
  private PIDController robotSpinController;
  private PIDController robotCounterSpinController;
  private boolean hasPoseBeenSet = false;
  private boolean isOdometry = true;

  /**
   * This enumeration clarifies the numbering of the swerve module for new users.
   * frontLeft  | 0
   * rearLeft   | 1
   * rearRight  | 2
   * frontRight | 3
   */
  public enum SwerveModNum{
    frontLeft(0) , rearLeft(1) , rearRight(2) , frontRight(3);
    public int num;
    private SwerveModNum(int number){
      num = number;
    }
    public int getNumber() {
			return num;
		}
  }
  
  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    //adds CANCoder address as third param, already named as rotation sensor in constants
    // Constructs the swerve modules 
    frontLeft = new SwerveModule(Constants.FRONT_LEFT_MOVE_MOTOR, Constants.FRONT_LEFT_ROTATE_MOTOR, Constants.FRONT_LEFT_ROTATE_SENSOR, false);
    rearLeft = new SwerveModule(Constants.REAR_LEFT_MOVE_MOTOR, Constants.REAR_LEFT_ROTATE_MOTOR, Constants.REAR_LEFT_ROTATE_SENSOR, false);
    rearRight = new SwerveModule(Constants.REAR_RIGHT_MOVE_MOTOR, Constants.REAR_RIGHT_ROTATE_MOTOR, Constants.REAR_RIGHT_ROTATE_SENSOR, false);
    frontRight = new SwerveModule(Constants.FRONT_RIGHT_MOVE_MOTOR, Constants.FRONT_RIGHT_ROTATE_MOTOR, Constants.FRONT_RIGHT_ROTATE_SENSOR, false);
    
     //This may seem repetitive, but it makes clear which module is which.
    swerveModules = new SwerveModule[]{
      frontLeft,
      rearLeft,
      rearRight,
      frontRight
    };
    
    //Create kinematics object, which converts between ChassisSpeeds and ModuleStates
    driveKinematics = new SwerveDriveKinematics(
      Constants.FRONT_LEFT_POSITION, Constants.REAR_LEFT_POSITION, 
      Constants.REAR_RIGHT_POSITION, Constants.FRONT_RIGHT_POSITION);

    // Constructs IMU object
    imu = new ADIS(IMUAxis.kY, SPI.Port.kOnboardCS0, CalibrationTime._4s);//Must use params, won't work without
    
    //construct the odometry class.
    driveOdometry = new SwerveDriveOdometry(driveKinematics, getGyroRotation2d());

    //construct the wpilib PIDcontroller for rotation.
    robotSpinController = new PIDController(Constants.ROBOT_SPIN_P, Constants.ROBOT_SPIN_I, Constants.ROBOT_SPIN_D);
    robotSpinController.setTolerance(Constants.ROBOT_SPIN_PID_TOLERANCE);

    //construct the wpilib PIDcontroller for counter rotation.
    robotCounterSpinController = new PIDController(Constants.ROBOT_COUNTER_SPIN_P, Constants.ROBOT_COUNTER_SPIN_I, Constants.ROBOT_COUNTER_SPIN_D);
    robotCounterSpinController.setTolerance(Constants.ROBOT_SPIN_PID_TOLERANCE);

    hasPoseBeenSet = false;
    isOdometry = true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Gyro", this.getGyroInDeg());
    // SmartDashboard.putNumber("GyroZ", this.getGyroInDegZ());
    //run odometry update on the odometry object
    if(isOdometry) {
      //instatiate and construct a 4 large SwerveModuleState array
      SwerveModuleState[] moduleStates =  new SwerveModuleState[4];
      //get the current SwerveModuleStates from all modules in array
      for (int i = 0; i < moduleStates.length; i++) {
        moduleStates[i] = swerveModules[i].getModuleState();
      }
      driveOdometry.update(getGyroRotation2d(), moduleStates);
      // SmartDashboard.putNumber("GyroRate", this.getRotationalVelocity());
      // SmartDashboard.putNumber("Odometry X", getCurPose2d().getX());
      // SmartDashboard.putNumber("Odometry Y", getCurPose2d().getY());
    }
  }

  public void setIsOdometry(boolean isOdometry){
    this.isOdometry = isOdometry;
  }

  public boolean getIsOdometry(){
    return isOdometry;
  }

  /**
   * Drives the robot based on speeds from the robot's orientation.
   * all speed should be in range of -1.0 to 1.0 with 0.0 being not 
   * moving for percentVoltage mode and between the Max Velocity 
   * and -Max Velocity with 0 not moving in Velocity mode
   * @param chassisSpeeds an object  
   * @param isVeloMode true if velocity mode, false if percent output mode
   */
  public void driveRobotCentric(ChassisSpeeds chassisSpeeds , boolean isVeloMode, boolean rotationOnlyMode){
    //instantiate an array of SwerveModuleStates, set equal to the output of toSwerveModuleStates() 
    SwerveModuleState[] targetStates = driveKinematics.toSwerveModuleStates(chassisSpeeds);
    //use SwerveDriveKinematic.desaturateWheelSpeeds(), max speed is 1 if percentOutput, MaxVelovcity if velocity mode
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, isVeloMode? Constants.PATH_MAXIMUM_VELOCITY : 1.0);
    // if(Math.abs(chassisSpeeds.vxMetersPerSecond) < 0.05 && Math.abs(chassisSpeeds.vyMetersPerSecond) < 0.05 && Math.abs(chassisSpeeds.omegaRadiansPerSecond) > .01){
    if(rotationOnlyMode){
      //pass along SwerveModuleStates to SwerveModules and pass along boolean isVeloMode
      for (int i = 0; i < targetStates.length; i++) {
        swerveModules[i].setModuleStateRot(targetStates[i], isVeloMode);
      } 
    }else{
      //pass along SwerveModuleStates to SwerveModules and pass along boolean isVeloMode
      for (int i = 0; i < targetStates.length; i++) {
          swerveModules[i].setModuleState(targetStates[i], isVeloMode);
      }
    }
  }

  /**
   * Drives the robot based on speeds from the robot's orientation.
   * all speed should be in range of -1.0 to 1.0 with 0.0 being not 
   * moving for percentVoltage mode and between the Max Velocity 
   * and -Max Velocity with 0 not moving in Velocity mode
   * @param forwardSpeed the movement forward and backward
   * @param strafeSpeed the movement side to side
   * @param rotSpeed the speed of rotation
   * @param isVeloMode true if velocity mode, false if percent output mode
   */
  public void driveRobotCentric(double forwardSpeed, double strafeSpeed, double rotSpeed, boolean isVeloMode, boolean rotationOnlyMode){
    //convert forwardSpeed, strafeSpeed and rotSpeed to a chassisSpeeds object, pass to driveRobotCentric
    driveRobotCentric(new ChassisSpeeds(forwardSpeed, strafeSpeed, rotSpeed), isVeloMode, rotationOnlyMode);
  }

  /**
   * Drive the robot so that all directions are independent of the robots orientation (rotation)
   * all speed should be in range of -1.0 to 1.0 with 0.0 being not moving in that direction
   * 
   * @param awaySpeed from field relative, aka a fix direction,
   *                  away from or toward the driver, a speed
   *                  valued between -1.0 and 1.0, where 1.0
   *                  is to away from the driver 
   * @param lateralSpeed from field relative, aka a fix direction
   *                     regardless of robot rotation, a speed
   *                     valued between -1.0 and 1.0, where 1.0
   *                     is to the left 
   * @param rotSpeed rotational speed of the robot
   *                 -1.0 to 1.0 where 0.0 is not rotating
   * @param isVeloMode true if velocity mode, false if percent output mode
   */
  public void driveFieldRelative(double awaySpeed, double lateralSpeed, double rotSpeed, boolean isVeloMode){
    //convert awaySpeed, lateralSpeed and rotSpeed to ChassisSpeeds with fromFieldRelativeSpeeds pass to driveRobotCentric
    driveRobotCentric(ChassisSpeeds.fromFieldRelativeSpeeds(awaySpeed, lateralSpeed, rotSpeed, getGyroRotation2d()), isVeloMode, false);
  }

/**
   * This function is meant to drive one module at a time for testing purposes.
   * @param moduleNumber which of the four modules(0-3) we are using
   * @param moveSpeed move speed -1.0 to 1.0, where 0.0 is stopped
   * @param rotatePos a position between -PI and PI where we want the module to be
   * @param isVeloMode changes between velocity mode and dutyCycle mode
   */
  public void driveOneModule(int moduleNumber,double moveSpeed, double rotatePos, boolean isVeloMode){
    //test that moduleNumber is between 0-3, return if not(return;)
    if (moduleNumber > 3 && moduleNumber < 0){
      System.out.println("Module " + moduleNumber + " is out of bounds.");
      return;
    }else if(rotatePos < -Math.PI || rotatePos > Math.PI){
      System.out.println("Input angle out of range.");
      return;
    }

    SwerveModuleState oneSwerveState = new SwerveModuleState(moveSpeed, new Rotation2d(rotatePos));
    //code to drive one module in a testing form
    swerveModules[moduleNumber].setModuleState( oneSwerveState , isVeloMode );

  }

  /**
   * Stops all module motion, then lets all the modules spin freely.
   */
  public void stopAllModules(){
    //run a for loop to call each mmodule
    for (int i=0; i<4; i++){
      //use the stopAll method, which stops both the drive and rotation motor.
      swerveModules[i].stopAll();
    }
  }

  /**
   * Pull the current Position from the odometry 
   * class, this should be in meters.
   * 
   * @return a Pose2d representing the current position
   */
  public Pose2d getCurPose2d(){
    return driveOdometry.getPoseMeters();
  }

  /**
   * Sets current position in the odometry class
   * 
   * @param pose new current position
   */
  public void setCurPose2d(Pose2d pose) {
    driveOdometry.resetPosition(pose, getGyroRotation2d());
    hasPoseBeenSet = true;
  }

  /**
   * Returns whether or not the position of the robo on the 
   * field is known because Pose2D has been set
   * @return
   */
  public boolean hasPoseBeenSet() {
    return hasPoseBeenSet;
  }

  /**
   * Estimates the closest angle to the target, 
   * given that the odometry is working
   * @return angle in radians (not restricted to PI to -PI)
   */
  public double getAngleOfTarget(){
    //get the current angle    
    double currentAngle = getGyroInRad();

    //if the pose hasn't been set return current angle.
    if(!hasPoseBeenSet){
      return currentAngle;
    }

    // Calculating current angle between -pi and pi
    double absoluteCurrentAngle = currentAngle%Constants.TWO_PI;
    if(absoluteCurrentAngle > Math.PI){
      absoluteCurrentAngle -= 2*(Math.PI);
    }
    else if(absoluteCurrentAngle < -Math.PI){
      absoluteCurrentAngle += 2 * Math.PI;
    }

    // Finds where the center of the field is with respect to the robot
    Translation2d target = driveOdometry.getPoseMeters().getTranslation().minus(Constants.FIELD_CENTER);
    //based on that, find the angle of the above Tanslation2d object
    double desiredAngle = Math.atan2(target.getY(), target.getX());

    // Calculate the robot's target angle given the continuous angle of the gyroscope
    return currentAngle - absoluteCurrentAngle + desiredAngle;
  }

  /**
   * A function that allows the user to reset the gyro, this 
   * makes the current orientation of the robot 0 degrees on 
   * the gyro.
   */
  public void resetGyro(){
    //Resets the gyro(zero it)
    if(!imu.getYawAxis().equals(IMUAxis.kY)){
      return;
    }
    imu.reset();
  }

  public void resetGyroZ(){
    if(!imu.getYawAxis().equals(IMUAxis.kY)){
      return;
    }
    imu.reset();
  }

  /**
   * A function that allows the user to set the gyro to a 
   * specific angle. This will make the current orientation 
   * of the robot the input value. This must be in degrees 
   * for gyro.
   * @param newCurrentAngle value the gyro should now read in degrees.
   */
  public void setGyro(double newCurrentAngle){
    if(!imu.getYawAxis().equals(IMUAxis.kY)){
      return;
    }
    imu.setGyroAngle(newCurrentAngle);
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
    return Rotation2d.fromDegrees(getGyroInDeg());
    //note that counterclockwise rotation is positive
  }

  /**
   * This polls the onboard gyro, which, when the robot boots,
   * assumes and angle of zero, this needs to be positive when
   * turning left
   * @return the angle of the robot in radians
   */
  public double getGyroInRad(){
    return imu.getYawAxis().equals(ADIS.IMUAxis.kY)? Math.toRadians(getGyroInDeg()) : 0.0;// Pull the gyro in degrees, convert and return in radians
    //note that counterclockwise rotation is positive
  }

  public double getGyroInRadZ(){
    return imu.getYawAxis().equals(ADIS.IMUAxis.kZ)? Math.toRadians(getGyroInDegZ()) : 0.0;// Pull the gyro in degrees, convert and return in radians
    //note that counterclockwise rotation is positive
  }

  public void setGyroAxis(ADIS.IMUAxis axis){
    if(imu.getYawAxis().equals(axis)){
      return;
    }
    imu.setYawAxis(axis);
  }

  public IMUAxis getGyroAxis(){
    return imu.getYawAxis();
  }

  /**
   * This polls the onboard gyro, which, when the robot boots,
   * assumes and angle of zero, this needs to be positive when
   * turning left
   * @return the angle of the robot in degrees
   */
  public double getGyroInDeg(){
    return imu.getYawAxis().equals(ADIS.IMUAxis.kY)? imu.getAngle() : 0.0;//getYawAxis();//*-1;//Pull gyro in degrees
    //note counterclockwise rotation is positive
  }

  public double getGyroInDegZ() {
    return imu.getYawAxis().equals(ADIS.IMUAxis.kZ)? imu.getAngle() : 0.0;
  }

  public boolean isTiltedBack(){
    return (imu.getYawAxis().equals(ADIS.IMUAxis.kZ)) && (Math.abs(getRotationalVelocityZ()) < 5) && (getGyroInDegZ() < -36 && getGyroInDegZ() > -40);
  }

  public boolean isTiltedUp(){
    return (imu.getYawAxis().equals(ADIS.IMUAxis.kZ)) && (Math.abs(getRotationalVelocityZ()) < 5) && (getGyroInDegZ() < -20 && getGyroInDegZ() > -25);
  }

  public boolean isLiftToThirdReady(){
    return (imu.getYawAxis().equals(ADIS.IMUAxis.kZ)) && (Math.abs(getRotationalVelocityZ()) < 2) && (getGyroInDegZ() > 15 );
  }

  /**
   * Returns the speed of rotation of the robot, 
   * counterclockwise is positive.
   * @return degrees per second
   */
  public double getRotationalVelocity(){
    return imu.getYawAxis().equals(ADIS.IMUAxis.kY)? imu.getRate() : 0.0;
  }

  public double getRotationalVelocityZ(){
    return imu.getYawAxis().equals(ADIS.IMUAxis.kZ)? imu.getRate() : 0.0;
  }

  /**
   * Returns all values from the module's absolute 
   * encoders, and returns them in an array of 
   * doubles, as degrees, in module order.
   * 
   * @return array of doubles, in degrees
   */
  public double[] getAllAbsModuleAngles(){
    double[] moduleAngles = new double[4];
    for(int i=0; i<4; i++){
      moduleAngles[i]=swerveModules[i].getAbsPosInDeg();
    }
    return moduleAngles;
  }

  /**
   * Returns all values from the rotational motor's 
   * reletive encoders in an array of doubles. This 
   * array is in order of module number.
   * 
   * @return array of doubles, representing tick count.
   */
  public double[] getAllModuleRelEnc(){
    double[] moduleRelEnc = new double[4];
    for(int i=0; i<4; i++){
      moduleRelEnc[i]=swerveModules[i].getRelEncCount();
    }
    return moduleRelEnc;
  }

  /**
   * Returns the collective distance as seen by the 
   * drive motor's encoder, for each module.
   * 
   * @return an array of doubles in meters
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
   * method to configure all modules DriveMotor PIDF
   * these are the PIDF on the TalonFX
   */
  public void setDrivePIDF(double P, double I, double D, double F){
    for (int i=0; i<4; i++){
      swerveModules[i].setDriveMotorPIDF(P, I, D, F);
    }
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

  /**
   * 
   * @param target an angle in radians
   * @return a value to give the rotational input, -1.0 to 1.0
   */
  public double getRobotRotationPIDOut(double target){
    double currentGyroPos = getGyroInRad();
    double output = robotSpinController.calculate(currentGyroPos, target);
    // System.out.println("targetAngle:"+Math.toDegrees(target)+"   angle:"+Math.toDegrees(currentGyroPos)+"atSP:"+robotSpinController.atSetpoint()+"  pid output"+output);
    if(robotSpinController.atSetpoint()){
      return 0.0;
    } else {
      if (Math.abs(output) < Constants.MINIMUM_ROTATIONAL_OUTPUT){
        return Constants.MINIMUM_ROTATIONAL_OUTPUT*Math.signum(output);
      }else {
        return output;
      }
    }
  }

  public double getCounterRotationPIDOut(double target){
    double currentGyroPos = getGyroInRad();
    return robotCounterSpinController.calculate(currentGyroPos, target);
  }

}
