package frc.team4481.robot.auto.actions;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.team4481.lib.auto.actions.Action;
import frc.team4481.lib.path.PathFollower;
import frc.team4481.lib.path.TrajectoryHandler;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.robot.subsystems.Drivetrain;
import frc.team4481.robot.subsystems.DrivetrainManager;

/**
 * Action to drive a predefined path using the Pure Pursuit algorithm.
 */
public class SetInitalPositionAction implements Action {
    private final String pathName;

    private final SwerveDrivePoseEstimator poseEstimator;
    private final Drivetrain drivetrain;
    private final DrivetrainManager drivetrainManager;

    TrajectoryHandler trajectoryHandler = TrajectoryHandler.getInstance();
    private final PathFollower pathFollower;

    /**
     * Create a new {@code DrivePathAction} that will follow a {@code Trajectory} using the Pure Pursuit algorithm.
     *
     * @param pathName Trajectory that the robot has to follow. Use the filename without extension from Path Planner
     */
    public SetInitalPositionAction(String pathName){
        this.pathName = pathName;

        SubsystemHandler subsystemHandler = SubsystemHandler.getInstance();
        drivetrain = (Drivetrain) subsystemHandler.getSubsystemByClass(Drivetrain.class);
        drivetrainManager = drivetrain.getSubsystemManager();

        poseEstimator = drivetrainManager.getPoseEstimator();
        pathFollower = drivetrainManager.getPathFollower();
    }

    @Override
    public void start() {
        DataLogManager.log("Initial Position Action Start");
        pathFollower.setPath(pathName, new Pose2d(), new ChassisSpeeds());

        PathPlannerTrajectory currentTrajectory = new PathPlannerTrajectory(trajectoryHandler.getCurrentPath(), new ChassisSpeeds(), new Rotation2d());

        drivetrain.resetOdometry(currentTrajectory.getInitialTargetHolonomicPose());
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void done() {
        DataLogManager.log("Initial Position Action End");
    }
}
