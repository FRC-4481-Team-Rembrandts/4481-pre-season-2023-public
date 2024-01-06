package frc.team4481.lib.path;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.PPLibTelemetry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4481.lib.swerve.HolonomicDriveController;

public class PathFollower {

    private final HolonomicDriveController driveController;

    private Double startTime = Double.NaN;
    private Translation2d prevTranslation = new Translation2d();
    private boolean isFinished = false;
    private PathPlannerTrajectory trajectory;

    private TrajectoryHandler trajectoryHandler = TrajectoryHandler.getInstance();

    public PathFollower(HolonomicDriveController driveController) {

        this.driveController = driveController;
    }

    public void setPath(String pathName, Pose2d currentPose, ChassisSpeeds startingSpeeds) {
        startTime = Double.NaN;
        PathPlannerPath path = trajectoryHandler.setPath(pathName);
        trajectory = new PathPlannerTrajectory(path, startingSpeeds, currentPose.getRotation());
        isFinished = false;
        prevTranslation = currentPose.getTranslation();
        driveController.reset(currentPose, startingSpeeds);


        // Current path for PathPlanner Telemetry
        PPLibTelemetry.setCurrentPath(path);
    }


    public ChassisSpeeds update(double timestamp, Pose2d currentPose) {
        if (startTime.isNaN()) {
            startTime = timestamp;
        }

        if (timestamp > startTime + trajectory.getTotalTimeSeconds()) {
            isFinished = true;
        }

        if (trajectory == null) {
            return new ChassisSpeeds();
        }

        PathPlannerTrajectory.State desiredState = trajectory.sample(timestamp - startTime);

        // Target pose for PathPlanner Telemetry
        PPLibTelemetry.setTargetPose(desiredState.getTargetHolonomicPose());
        // Current pose for PathPlanner Telemetry
        PPLibTelemetry.setCurrentPose(currentPose);
        // Inaccuracy for PathPlanner Telemetry
        // Distance of previous target with current target
        PPLibTelemetry.setPathInaccuracy(prevTranslation.getDistance(currentPose.getTranslation()));
        prevTranslation = desiredState.positionMeters;

        SmartDashboard.putString("PF/position in meters", desiredState.positionMeters.toString());

        SmartDashboard.putNumber("PF/running time", timestamp - startTime);
        SmartDashboard.putNumber("PF/max ang vel in rot per second", desiredState.constraints.getMaxAngularVelocityRps());
        SmartDashboard.putNumber("PF/desired holonomic rotation (degrees)", desiredState.targetHolonomicRotation.getDegrees());
        SmartDashboard.putNumber("PF/desired heading (degrees)", desiredState.heading.getDegrees());

        return driveController.calculateFieldRelativeSpeeds(currentPose, desiredState);
    }


    public boolean isFinished() {
        return trajectory != null && isFinished;
    }

}
