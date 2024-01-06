package frc.team4481.robot.HIDlayout;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4481.lib.controller.ControlDevice;
import frc.team4481.lib.hid.HIDLayout;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.lib.throwable.HardwareException;
import frc.team4481.robot.configuration.Configuration;
import frc.team4481.robot.configuration.ConfigurationHandler;
import frc.team4481.robot.subsystems.Drivetrain;
import frc.team4481.robot.subsystems.*;

import static frc.team4481.lib.controller.IPS4HID.Axis.*;
import static frc.team4481.lib.controller.IPS4HID.Button.*;
import static frc.team4481.robot.Constants.*;

public class EventLayout extends HIDLayout {
    private final SubsystemHandler subsystemHandler = SubsystemHandler.getInstance();
    private final ConfigurationHandler configHandler = ConfigurationHandler.getInstance();
    private Drivetrain drivetrain;
    private DrivetrainManager drivetrainManager;
    private SlewRateLimiter driveSpeedSlewRateLimiter = new SlewRateLimiter(8);

    public EventLayout(ControlDevice driver, ControlDevice operator) {
        super(driver, operator);
    }

    @Override
    public void getSubsystemManagers() {
        drivetrain = (Drivetrain) subsystemHandler.getSubsystemByClass(Drivetrain.class);
        drivetrainManager = drivetrain.getSubsystemManager();

    }

    @Override
    public void updateOrange() throws HardwareException {
        //Update configuration
        Configuration config = configHandler.getConfig();
        // Drivetrain control
        setDrivetrainSpeed(
                filterDeadband(curveAxis(-driver.getAxisValue(LEFTSTICK_Y)) * config.getMaxDriveVelocity(), ORANGE_LEFTSTICK_DEADBAND),
                filterDeadband(curveAxis(-driver.getAxisValue(LEFTSTICK_X)) * config.getMaxDriveVelocity(), ORANGE_LEFTSTICK_DEADBAND),
                filterDeadband(-driver.getAxisValue(RIGHTSTICK_X) * config.getMaxDriveVelocity(), ORANGE_RIGHTSTICK_DEADBAND)
        );

        // Reset drivetrain rotation offset
        zeroDrivetrainRotation(driver.getButtonValue(OPTIONS));
    }

    @Override
    public void updateBlack() throws HardwareException {
    }

    /**
     * Sets the target velocities of a {@code Drivetrain} equal to the specified input velocities
     *
     * @param forwardVelocity velocity in a forward direction in m/s
     * @param rightVelocity   velocity in a right direction in m/s
     * @param angularVelocity velocity in a counterclockwise direction in rad/s
     */
    private void setDrivetrainSpeed(double forwardVelocity, double rightVelocity, double angularVelocity) {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(forwardVelocity, rightVelocity, angularVelocity);
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
            chassisSpeeds = new ChassisSpeeds(-forwardVelocity, -rightVelocity, angularVelocity);
        }
        drivetrainManager.setDesiredSpeeds(chassisSpeeds);
        SmartDashboard.putNumber("DT/slewrated target velocity", driveSpeedSlewRateLimiter.calculate(Math.hypot(forwardVelocity, rightVelocity)));
    }

    /**
     * Filters controllerValue with a deadband
     *
     * @param controllerValue controller value
     * @param deadband        deadband a positive value
     * @return filtered controller value
     */
    private double filterDeadband(double controllerValue, double deadband) {
        if (Math.abs(controllerValue) <= deadband) {
            return 0;
        } else {
            return controllerValue;
        }
    }

    /**
     * Zeroes drivetrain sensors
     *
     * @param button controller button
     */
    private void zeroDrivetrainRotation(boolean button) {
        if (button) {
            if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
                drivetrain.resetRotation(Rotation2d.fromDegrees(180));
            } else {
                drivetrain.resetRotation(new Rotation2d());
            }
        }
    }

    private void rotateDrivetrain(boolean button, Rotation2d rot) {
        if (button) {
            PathPlannerTrajectory.State state = new PathPlannerTrajectory.State();
            Pose2d currentPose = drivetrainManager.getCurrentPose();
            state.positionMeters = currentPose.getTranslation();
            state.targetHolonomicRotation = rot;
            state.constraints = new PathConstraints(0,0,540.0,360.0);
            state.velocityMps = 0;
            drivetrainManager.setDesiredSpeeds(drivetrain.driveController.calculateFieldRelativeSpeeds(currentPose, state));
        }
    }

    /**
     * Make the drivetrain face a certain point on the field
     *
     * @param button controller button
     */
    private void drivetrainLookAtPosition(boolean button) {
        if (button) {
            drivetrain.lookAtPosition(new Translation2d());
        }
    }

    private double curveAxis(double controllerValue) {
        // Exponential
        return controllerValue * Math.exp(Math.abs(controllerValue) - 1);

        // Polynomial
        // return Math.pow(controllerValue, 3);
    }

}





