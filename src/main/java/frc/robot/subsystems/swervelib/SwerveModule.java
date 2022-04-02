/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.swervelib;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;

import frc.robot.Constants;

/**
 * This is the class containing both motor controllers, 
 * the CANCodder and all functions needed to run one swerve 
 * module. This class handles all access to these objects.
 * 
 * This class hanndles the inverting the drive motor, in 
 * order to change wheel direction faster, as the optimize 
 * is in this class.
 * 
 * More on swerve found here:
 * https://docs.google.com/presentation/d/1feVl0L5lgIKSZhKCheWgWhkOydIu-ibgdp7oqA0yqAQ/edit?usp=sharing
 */
public class SwerveModule {
    private TalonFX driveMotor;
    private TalonFX rotationMotor;

    // The rotateAbsSensor is an absolute position sensor ranging from -180 to 180,
    // We'll use it to tell where we are and where we want to be, but due to the
    // discontinuity from -180 to 180, it can't be used for motion. We use the 
    // rotateRelEncoder for that.
    private CANCoder rotateAbsSensor;

    /**
     * Creates a new SwerveModule object
     * 
     * @param driveMotorID    The CAN ID of the TalonFX connected to the drive
     *                        motor(expecting Falon 500)
     * @param rotationMotorID The CAN ID of the SparkMax connected to the module
     *                        rotation motor(expecting NEO 550)
     * @param canCoderID      The CAN ID of the rotation sensor
     */
    public SwerveModule(int driveMotorID, int rotationMotorID, int canCoderID, boolean isInverted) {
        
        //contruct and setup drive falcon
        driveMotor = new TalonFX(driveMotorID);
        driveMotor.configFactoryDefault();
        // use the integrated sensor with the primary closed loop and timeout is 0.
        driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        driveMotor.configSelectedFeedbackCoefficient(1);//Constants.DRIVE_ENC_TO_METERS_FACTOR);
        // above uses configSelectedFeedbackCoefficient(), to scale the
        // driveMotor to real distance, DRIVE_ENC_TO_METERS_FACTOR
        driveMotor.setNeutralMode(NeutralMode.Brake);
        driveMotor.setInverted(isInverted);// Set motor inverted(set to false)
        driveMotor.enableVoltageCompensation(true);
        driveMotor.configVoltageCompSaturation(Constants.MAXIMUM_VOLTAGE);
        setDriveMotorPIDF(Constants.SWERVE_DRIVE_P_VALUE, Constants.SWERVE_DRIVE_I_VALUE,
                          Constants.SWERVE_DRIVE_D_VALUE, Constants.SWERVE_DRIVE_FF_VALUE);
        driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 253);//TODO: rethink if we need this speed, I don't think we do
        driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);//This is key to odometry must be around same as code loop
        driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 251);
        driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 241);
        driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 239);
        driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 233);
        driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 229);
        driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus, 255);
        driveMotor.setSelectedSensorPosition(0.0);

        //contruct and setup rotation falcon
        rotationMotor = new TalonFX(rotationMotorID);
        rotationMotor.configFactoryDefault();
        // use the integrated sensor with the primary closed loop and timeout is 0.
        rotationMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        rotationMotor.configSelectedFeedbackCoefficient(1);
        rotationMotor.setNeutralMode(NeutralMode.Brake);
        rotationMotor.setInverted(true);// Set motor inverted(set to true)
        rotationMotor.enableVoltageCompensation(true);
        rotationMotor.configVoltageCompSaturation(Constants.MAXIMUM_VOLTAGE);
        setRotationMotorPIDF(Constants.SWERVE_ROT_P_VALUE, Constants.SWERVE_ROT_I_VALUE,
                          Constants.SWERVE_ROT_D_VALUE, Constants.SWERVE_ROT_FF_VALUE);
        rotationMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 240);//This packet is the motor output, limit switches, faults, we care about none of those
        rotationMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);//This is the sensor feedback, i.e. relative encoder
        rotationMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 251);
        rotationMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 241);
        rotationMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 239);
        rotationMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 233);
        rotationMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 229);
        rotationMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus, 255);
        // System.out.println( "Status_10_MotionMagic" + rotationMotor.getStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic));
        // System.out.println( "Status_1_General" + rotationMotor.getStatusFramePeriod(StatusFrameEnhanced.Status_1_General));
        // System.out.println( "11_UartGadgeteer" + rotationMotor.getStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer));
        // System.out.println( "Status_13_Base_PIDF0" + rotationMotor.getStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0));
        // System.out.println( "Status_14_Turn_PIDF1" + rotationMotor.getStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1));
        // System.out.println( "Status_15_FirmwareApiStatus" + rotationMotor.getStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus));
        // System.out.println( "Status_3_Quadrature" + rotationMotor.getStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature));
        // System.out.println( "Status_4_AinTempVbat" + rotationMotor.getStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat));
        // System.out.println( "Status_6_Misc" + rotationMotor.getStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc));
        // System.out.println( "Status_7_CommStatus" + rotationMotor.getStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus));
        // System.out.println( "Status_8_PulseWidth" + rotationMotor.getStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth));
        // System.out.println( "Status_9_MotProfBuffer" + rotationMotor.getStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer));
        rotationMotor.setSelectedSensorPosition(0.0);
        rotationMotor.configAllowableClosedloopError(0, Constants.SWERVE_MODULE_TOLERANCE, 0);
        

        //the following sensor is angle of the module, as an absolute value
        rotateAbsSensor = new CANCoder(canCoderID);
        rotateAbsSensor.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        rotateAbsSensor.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 20);//The default on this is 10, but 20 might be better given our code loop rate
        rotateAbsSensor.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 255);
        System.out.println( "VbatFaults" + rotateAbsSensor.getStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults));
        // rotateAbsSensor.getStatusFramePeriod(frame);
    }


    /**
     * Set the speed of the drive motor in percent duty cycle
     * 
     * @param dutyCycle a number between -1.0 and 1.0, where 0.0 is not moving, as
     *                  percent duty cycle
     */
    public void setDriveMotor(double dutyCycle) {
        driveMotor.set(TalonFXControlMode.PercentOutput, dutyCycle );
    }

    /**
     * Set the speed of the drive motor in meter per second, this relies on the
     * PIDController built into the TalonFX.
     * 
     * @param speed a speed in meters per second
     */
    public void setDriveSpeed(double speed) {
        // driveMotor.set(TalonFXControlMode.Velocity, speed );
        driveMotor.set(TalonFXControlMode.PercentOutput, speed / Constants.MOTOR_MAXIMUM_VELOCITY);
    }

   /**
     * Sets drive motor to brake or coast
     * 
     * @param neutral whether to brake or coast   
     */
    public void setDriveMotorMode(NeutralMode neutral){
        driveMotor.setNeutralMode(neutral);
    }

    /**
     * @return the distance the drive wheel has traveled
     */
    public double getDriveDistance() {
        return driveMotor.getSensorCollection().getIntegratedSensorPosition()*Constants.DRIVE_ENC_TO_METERS_FACTOR;
    }

    /**
     * Returns the speed of the drive wheel in Meters per second
     * 
     * @return speed of the drive wheel
     */
    public double getDriveVelocity() {
        return driveMotor.getSensorCollection().getIntegratedSensorVelocity()*10*Constants.DRIVE_ENC_TO_METERS_FACTOR;
    }

    /**
     * A method to set the position of the drive encoder to zero,
     * essentially resetting it. 
     */
    public void resetDriveMotorEncoder() {
        driveMotor.setSelectedSensorPosition(0.0);// this code sets the Drive position to 0.0
    }

    /**
     * sets the drive motor's PIDF for the PIDF controller on the TalonFX
     * 
     * @param P value of the P constant
     * @param I value of the I constant
     * @param D value of the D constant
     * @param F value of the F constant
     */
    public void setDriveMotorPIDF(double P, double I, double D, double F) {
        driveMotor.config_kP(0, P);
        driveMotor.config_kI(0, I);
        driveMotor.config_kD(0, D);
        driveMotor.config_kF(0, F);
    }

    /**
     * sets the rotation motor's PIDF for the PIDF controller on the TalonFX
     * 
     * @param P value of the P constant
     * @param I value of the I constant
     * @param D value of the D constant
     * @param F value of the F constant
     */
    public void setRotationMotorPIDF(double P, double I, double D, double F) {
        rotationMotor.config_kP(0, P);
        rotationMotor.config_kI(0, I);
        rotationMotor.config_kD(0, D);
        rotationMotor.config_kF(0, F);
    }

    /**
     * The CANCoder has a mechanical zero point, this is hard 
     * to move, so this method is used to set the offset of the 
     * CANCoder so we can dictate the zero position. 
     * INPUTS MUST BE IN DEGREES. 
     * 
     * @param value a number between -180 and 180, where 0 is straight ahead
     */
    private void setRotateAbsSensor(double value) {
        rotateAbsSensor.configMagnetOffset(value, 0);
    }

    /**
     * The CANCoder has a mechanical zero point, this is hard 
     * to move, so this method is used to change the offset of 
     * the CANCoder so we dictate the zero position as the 
     * current position of the module.
     */
    public void zeroAbsPositionSensor() {
        //find the current offset, subtract the current position, and makes this number the new offset.
        setRotateAbsSensor(this.rotateAbsSensor.configGetMagnetOffset()-getAbsPosInDeg());
    }

    /**
     * The CANCoder reads the absolute rotational position
     * of the module. This method returns that positon in 
     * degrees.
     * note: NOT Inverted module safe (use getPosInRad())
     * 
     * @return the position of the module in degrees, should limit from -180 to 180
     */
    public double getAbsPosInDeg() {
        return rotateAbsSensor.getAbsolutePosition();
    }

    /**
     * Returns the current angle of the swerve module, 
     * as read by the absolute rotational sensor, as a 
     * Rotation2d object. This is measured from the 
     * front of the robot, where counter-clockwise is 
     * positive.
     * 
     * @return A Rotation2d object, current position of the module
     */
    public Rotation2d getCurRot2d(){
        return Rotation2d.fromDegrees(getAbsPosInDeg());
    }

    /**
     * This method gets the current position in radians and 
     * normally the zero is at the front of the robot.
     * 
     * @return the position of the module in radians, should limit from -PI to PI
     */
    public double getPosInRad() {
        //get the current position and convert it to radians.
        return Math.toRadians(getAbsPosInDeg());
    }
   
    /**
     * Returns the current state of the swerve module 
     * as a SwerveModuleState. The speed of the module 
     * should be in m/s and the rotational position is 
     * in the form of a Rotation2d object.
     * 
     * @return a SwerveModuleState
     */
    public SwerveModuleState getModuleState(){
        return new SwerveModuleState(getDriveVelocity(), getCurRot2d());
    }

    /**
     * This is a method meant for testing by getting the count from the 
     * rotational encoder which is internal to the NEO550. This encoder 
     * is relative, and does not easily translate to a specific rotational 
     * position of the swerve module.
     * 
     * @return the encoder count(no units, naturally just the count)
     */
    public double getRelEncCount() {
        return rotationMotor.getSelectedSensorPosition();
    }
    
    /**
     * The method to set the module to a position and speed. 
     * This method does the opitimization internally. The 
     * speed should be from -1.0 to 1.0 if isVeloMode is false, 
     * and should be between -MAX_VELOCITY and MAX_VELOCITY if 
     * isVeloMode is true.
     * 
     * @param targetState SwerveModuleState
     * @param isVeloMode true if velocity mode, false if percent output mode
     */
    public void setModuleState(SwerveModuleState targetState, boolean isVeloMode){
        
        // Instatiate Rotation2d object and fill with call from getCurRot2d()
        Rotation2d curPosition = getCurRot2d();
        
        // Optimize targetState with Rotation2d object pulled from above
        targetState = optimize(targetState, curPosition);
        
        // System.out.println("curAngle: "+curPosition.getDegrees()+"\t\t\t tarAngle: "+targetState.angle.getDegrees());

        // Find the difference between the target and current position
        double posDiff = targetState.angle.getRadians() - curPosition.getRadians(); 
        double absDiff = Math.abs(posDiff);
        
        // if the distance is more than a half circle, we are going the wrong way
        if (absDiff > Math.PI) {
            // the distance the other way around the circle
            posDiff = posDiff - (Constants.TWO_PI * Math.signum(posDiff));
        }
        
        // Convert the shortest distance of rotation to relative encoder value(use convertion factor)
        double targetAngle = posDiff * Constants.RAD_TO_ENC_CONV_FACTOR;
        // add the encoder distance to the current encoder count
        double outputEncValue = targetAngle + getRelEncCount();

        if(Math.abs(targetState.speedMetersPerSecond) < (isVeloMode?Constants.MINIMUM_DRIVE_SPEED:Constants.MINIMUM_DRIVE_DUTY_CYCLE)) {
            stopAll();
        } else {
            // Set the setpoint using setReference on the TalonFX
            rotationMotor.set(TalonFXControlMode.Position, outputEncValue);

            // Output to drive motor based on velomode or not
            if (isVeloMode) {
                setDriveSpeed(targetState.speedMetersPerSecond);
            } else {
                setDriveMotor(targetState.speedMetersPerSecond);
            }
        }
    }  

    /**
     * The method to set the module to a position and speed. 
     * This method does the opitimization internally. The 
     * speed should be from -1.0 to 1.0 if isVeloMode is false, 
     * and should be between -MAX_VELOCITY and MAX_VELOCITY if 
     * isVeloMode is true.
     * 
     * @param targetState SwerveModuleState
     * @param isVeloMode true if velocity mode, false if percent output mode
     */
    public void setModuleStateRot(SwerveModuleState targetState, boolean isVeloMode){
        
        // Instatiate Rotation2d object and fill with call from getCurRot2d()
        Rotation2d curPosition = getCurRot2d();
        
        // Optimize targetState with Rotation2d object pulled from above
        targetState = optimize(targetState, curPosition);
        
        // System.out.println("curAngle: "+curPosition.getDegrees()+"\t\t\t tarAngle: "+targetState.angle.getDegrees());

        // Find the difference between the target and current position
        double posDiff = targetState.angle.getRadians() - curPosition.getRadians(); 
        double absDiff = Math.abs(posDiff);
        
        // if the distance is more than a half circle, we are going the wrong way
        if (absDiff > Math.PI) {
            // the distance the other way around the circle
            posDiff = posDiff - (Constants.TWO_PI * Math.signum(posDiff));
        }
        
        // Convert the shortest distance of rotation to relative encoder value(use convertion factor)
        double targetAngle = posDiff * Constants.RAD_TO_ENC_CONV_FACTOR;
        // add the encoder distance to the current encoder count
        double outputEncValue = targetAngle + getRelEncCount();
        
        // Set the setpoint using setReference on the TalonFX
        rotationMotor.set(TalonFXControlMode.Position, outputEncValue);

        // Output to drive motor based on velomode or not
        if (isVeloMode) {
            setDriveSpeed(targetState.speedMetersPerSecond);
        } else {
            setDriveMotor(targetState.speedMetersPerSecond);
        }

    }  

    /**
     * This is a testing method, used to drive the module's rotation.
     * It takes pure motor duty cycle(percent output). Positive input 
     * should result in counter-clockwise rotation. If not, the motor
     * output must be inverted.
     * 
     * @param speed a percent output from -1.0 to 1.0, where 0.0 is stopped
     */
    public void driveRotateMotor(double speed) {
        this.rotationMotor.set(TalonFXControlMode.PercentOutput, speed);
    }

    /**
     * This method is used to stop the module completely. The drive 
     * motor is switched to percent voltage and and output of 0.0 
     * percent volts. The rotation motor's PIDController is set to 
     * DutyCyclevoltage control mode, and output of 0.0% output.
     */
    public void stopAll() {
        driveMotor.set(TalonFXControlMode.PercentOutput, 0.0);
        rotationMotor.set(TalonFXControlMode.PercentOutput, 0.0);
    }

    /**
   * Minimize the change in heading the desired swerve module state would require by potentially
   * reversing the direction the wheel spins. If this is used with the PIDController class's
   * continuous input functionality, the furthest a wheel will ever rotate is 90 degrees.
   *
   * @param desiredState The desired state.
   * @param currentAngle The current module angle.
   * @return Optimized swerve module state.
   */
    public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
    var delta = desiredState.angle.minus(currentAngle);
    if (Math.abs(delta.getRadians()) > Constants.PI_OVER_TWO) {
        return new SwerveModuleState(
            -desiredState.speedMetersPerSecond,
            desiredState.angle.rotateBy(Constants.ROTATE_BY_PI));
    } else {
        return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
    }
}
    
}
