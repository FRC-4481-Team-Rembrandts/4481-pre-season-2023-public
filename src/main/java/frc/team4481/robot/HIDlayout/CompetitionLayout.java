package frc.team4481.robot.HIDlayout;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.team4481.lib.controller.ControlDevice;
import frc.team4481.lib.hid.HIDLayout;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.lib.throwable.HardwareException;
import frc.team4481.robot.configuration.Configuration;
import frc.team4481.robot.configuration.ConfigurationHandler;
import frc.team4481.robot.subsystems.Drivetrain;
import frc.team4481.robot.subsystems.DrivetrainManager;

import static frc.team4481.lib.controller.IPS4HID.Axis.*;
import static frc.team4481.lib.controller.IPS4HID.Button.BUMPER_L1;
import static frc.team4481.lib.controller.IPS4HID.Button.OPTIONS;
import static frc.team4481.robot.Constants.ORANGE_LEFTSTICK_DEADBAND;
import static frc.team4481.robot.Constants.ORANGE_RIGHTSTICK_DEADBAND;

public class CompetitionLayout extends HIDLayout {
    private final SubsystemHandler subsystemHandler = SubsystemHandler.getInstance();
    private final ConfigurationHandler configHandler = ConfigurationHandler.getInstance();
    private Drivetrain drivetrain;
    private DrivetrainManager drivetrainManager;

    public CompetitionLayout(ControlDevice driver, ControlDevice operator) {
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
        double deadbandedY = filterDeadband(-driver.getAxisValue(LEFTSTICK_Y) , ORANGE_LEFTSTICK_DEADBAND); //Inverted because of particular joystick output
        double deadbandedX = filterDeadband(-driver.getAxisValue(LEFTSTICK_X), ORANGE_LEFTSTICK_DEADBAND); //Inverted because of particular joystick output

        Translation2d scaledTranslation = curveCombinedAxis(deadbandedX, deadbandedY);
        double xInput = scaledTranslation.getX();
        double yInput = scaledTranslation.getY();
        setDrivetrainSpeed(
                xInput * config.getMaxDriveVelocity(),
                yInput * config.getMaxDriveVelocity(),
                filterDeadband(-driver.getAxisValue(RIGHTSTICK_X) , ORANGE_RIGHTSTICK_DEADBAND) * config.getMaxDriveVelocity()
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

    /**
     * Apply a function to the x and y input of the joystick,
     * while making sure that the output x and y are still along a circle.
     * Squaring for example the x and y separately converts the circle into
     * a square, which is not desirable.
     * @param x x input
     * @param y y input
     * @return Translation with a new distance to (0,0), with the same angle
     */
    private Translation2d curveCombinedAxis(double x, double y){
        Translation2d inputTranslation = new Translation2d(x, y);
        double distance = inputTranslation.getNorm(); //Distance from (0,0) to the coordinate (x,y)
        Rotation2d angle = inputTranslation.getAngle(); //Angle of the vector to the coordinate (x,y)

        double newDistance = Math.pow(distance, 2); //Square the radius to make the robot easier to control

        return new Translation2d(newDistance, angle);
    }

}





