package frc.team4481.robot.auto.actions;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4481.lib.auto.actions.Action;
import frc.team4481.lib.path.PathFollower;
import frc.team4481.lib.path.PathPlannerUtil;
import frc.team4481.lib.path.TrajectoryHandler;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.robot.subsystems.Drivetrain;
import frc.team4481.robot.subsystems.DrivetrainManager;

/**
 * Action to drive a predefined path using the Pure Pursuit algorithm.
 */
public class DrivePathAction implements Action {
    private final String pathName;

    private final Drivetrain drivetrain;
    private final DrivetrainManager drivetrainManager;

    private TrajectoryHandler trajectoryHandler = TrajectoryHandler.getInstance();
    private final PathFollower pathFollower;

    /**
     * Create a new {@code DrivePathAction} that will follow a {@code PathPlannerPath}.
     *
     * @param pathName that the robot has to follow. Use the filename without extension from Path Planner
     */
    public DrivePathAction(String pathName){
        this.pathName = pathName;

        SubsystemHandler subsystemHandler = SubsystemHandler.getInstance();
        drivetrain = (Drivetrain) subsystemHandler.getSubsystemByClass(Drivetrain.class);
        drivetrainManager = drivetrain.getSubsystemManager();

        pathFollower = drivetrainManager.getPathFollower();
    }

    @Override
    public void start() {
        DataLogManager.log("Drive Action Start");
        SmartDashboard.putString("Auto/trajectory name", pathName);

        // Get Current drivetrain info
        ChassisSpeeds startingSpeeds = drivetrain.getCurrentChassisSpeeds();
        Pose2d currentPose = drivetrainManager.getCurrentPose();

        // Set path
        pathFollower.setPath(pathName, currentPose, startingSpeeds);

        // Display trajectory on Driver Station
        PathPlannerTrajectory currentTrajectory = new PathPlannerTrajectory(trajectoryHandler.getCurrentPath(), startingSpeeds, currentPose.getRotation());
        drivetrain.field.getObject(pathName).setTrajectory(PathPlannerUtil.convertTrajectoryType(currentTrajectory));

        // Tell drivetrain to follow the path
        drivetrainManager.setControlState(DrivetrainManager.controlState.PATH_FOLLOWING);
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        return pathFollower.isFinished();
    }

    @Override
    public void done() {
        drivetrainManager.setControlState(DrivetrainManager.controlState.DISABLED);
        DataLogManager.log("Drive Action Done");
    }
}
