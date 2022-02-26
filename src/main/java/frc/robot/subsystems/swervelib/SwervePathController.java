package frc.robot.subsystems.swervelib;

import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;

@Deprecated
public class SwervePathController {
    private final PIDController posErrorController;
    private final PIDController headingErrorController;
    private final ProfiledPIDController rotationController;

    private Translation2d lastPosition;
    private double totalDistance;
    private Rotation2d currentHeading;

    /**
     * Construct a SwervePathController
     *
     * @param posErrorController     PIDController for the robot's position
     * @param headingErrorController PIDController for the robot's heading
     * @param rotationController     ProfiledPIDController for the robot's rotation
     */
    public SwervePathController(PIDController posErrorController, PIDController headingErrorController, ProfiledPIDController rotationController) {
        this.posErrorController = posErrorController;
        this.headingErrorController = headingErrorController;
        this.headingErrorController.enableContinuousInput(-Math.PI, Math.PI);
        this.rotationController = rotationController;
        if(rotationController != null) {
            this.rotationController.enableContinuousInput(-Math.PI, Math.PI);
        }
        this.lastPosition = new Translation2d();
        this.totalDistance = 0;
        this.currentHeading = new Rotation2d(0);
    }

    public SwervePathController(PIDController posErrorController, PIDController headingErrorController) {
        this(posErrorController, headingErrorController, null);
    }

    public double getPosError(){
        return posErrorController.getPositionError();
    }

    /**
     * Reset the state of the path controller
     *
     * @param currentPose The current pose of the robot
     */
    public void reset(Pose2d currentPose) {
        this.posErrorController.reset();
        this.headingErrorController.reset();
        if(rotationController != null) {
            this.rotationController.reset(currentPose.getRotation().getRadians());
        }
        this.lastPosition = currentPose.getTranslation();
        this.totalDistance = 0;
        this.currentHeading = new Rotation2d(0);
    }

    public double getTotalDistance(){
        return this.totalDistance;
    }

    public Rotation2d getCurrentHeading(){
        return this.currentHeading;
    }

    /**
     * Calculate the robot's speeds to match the path
     *
     * @param currentPose     Current pose of the robot
     * @param goalState       Goal state of the robot
     * @return The calculated speeds and rotation
     */
    public ChassisSpeeds calculate(Pose2d currentPose, PathPlannerState goalState, double deltaTime, boolean doHeading) {
        Translation2d currentPos = currentPose.getTranslation();
        Rotation2d currentRotation = currentPose.getRotation();

        totalDistance += lastPosition.getDistance(currentPos);
        double xV = (currentPos.getX() - lastPosition.getX()) / deltaTime;
        double yV = (currentPos.getY() - lastPosition.getY()) / deltaTime;
        this.currentHeading = new Rotation2d(Math.atan2(yV, xV));

        double vel = goalState.velocityMetersPerSecond;
        Rotation2d heading = goalState.poseMeters.getRotation();
        double rotSpeed = (rotationController != null) ? rotationController.calculate(currentRotation.getRadians(), goalState.holonomicRotation.getRadians()) : 0;

        vel += posErrorController.calculate(0, goalState.poseMeters.getTranslation().getDistance(currentPose.getTranslation()));

        if(doHeading) {
            heading = heading.plus(new Rotation2d(headingErrorController.calculate
            (this.currentHeading.getRadians(), goalState.holonomicRotation.getRadians())));
        }

        // System.out.println("vel: "+vel);
        // System.out.println("heading cos: "+heading.getCos());
        // System.out.println("heading sin: "+heading.getSin());
        double xVel = vel * heading.getCos();
        double yVel = vel * heading.getSin();
        // System.out.println("xVel: "+xVel);
        // System.out.println("yVel: "+yVel);

        this.lastPosition = currentPos;

        return ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, rotSpeed, currentRotation);
    }
}