package frc.team4481.lib.path;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;

import java.util.ArrayList;
import java.util.List;

public class PathPlannerUtil {

    /**
     * Convert a {@code PathPlannerTrajectory} to a {@code Trajectory}
     */
    public static Trajectory convertTrajectoryType(PathPlannerTrajectory trajectory) {
        List<Trajectory.State> trajectoryStates = new ArrayList<>();

        for (PathPlannerTrajectory.State pathPlannerState : trajectory.getStates()) {
            trajectoryStates.add(new Trajectory.State(
                    pathPlannerState.timeSeconds,
                    pathPlannerState.velocityMps,
                    pathPlannerState.accelerationMpsSq,
                    new Pose2d(pathPlannerState.positionMeters, pathPlannerState.targetHolonomicRotation),
                    pathPlannerState.curvatureRadPerMeter));
        }

        return new Trajectory(trajectoryStates);
    }

    public static double calculateLinearSpeed(ChassisSpeeds speeds) {
        return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    }

}
