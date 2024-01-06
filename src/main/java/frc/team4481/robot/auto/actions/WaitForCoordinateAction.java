package frc.team4481.robot.auto.actions;

import edu.wpi.first.math.geometry.Pose2d;
import frc.team4481.lib.auto.actions.Action;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.robot.subsystems.Drivetrain;
import frc.team4481.robot.subsystems.DrivetrainManager;

// TODO test class

public class WaitForCoordinateAction implements Action {
    private Pose2d startingPose;
    private boolean isXCoordiante;
    private double coordinate;

    private final Drivetrain drivetrain;
    private final DrivetrainManager drivetrainManager;

    public WaitForCoordinateAction(boolean isXCoordinate, double coordinate) {
        this.isXCoordiante = isXCoordinate;
        this.coordinate = coordinate;

        SubsystemHandler subsystemHandler = SubsystemHandler.getInstance();
        drivetrain = (Drivetrain) subsystemHandler.getSubsystemByClass(Drivetrain.class);
        drivetrainManager = drivetrain.getSubsystemManager();
    }

    @Override
    public void start() {
        startingPose = drivetrainManager.getCurrentPose();
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        Pose2d currentPose = drivetrainManager.getCurrentPose();
        double startCoordinate;
        double currentCoordinate;
        if (isXCoordiante) {
            startCoordinate = startingPose.getX();
            currentCoordinate = currentPose.getX();
        } else {
            startCoordinate = startingPose.getY();
            currentCoordinate = currentPose.getY();
        }

        return Math.signum(startCoordinate - coordinate) != Math.signum(currentCoordinate - coordinate);
    }

    @Override
    public void done() {

    }
}
