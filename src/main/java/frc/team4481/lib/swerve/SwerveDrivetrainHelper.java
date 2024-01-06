package frc.team4481.lib.swerve;


import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDrivetrainHelper
{
    private SwerveModule frontLeft;
    private SwerveModule frontRight;
    private SwerveModule backLeft;
    private SwerveModule backRight;

    private SwerveDriveKinematics kinematics;
    private SecondOrderSwerveKinematics secondKinematics;

    private SwerveDrivePoseEstimator poseEstimator;

    private double maxVelocity;
    private double maxVelocityBoost;
    private boolean fieldRelative;

    public SwerveDrivetrainHelper(
            SwerveModule frontLeft,
            SwerveModule frontRight,
            SwerveModule backLeft,
            SwerveModule backRight,
            SwerveDriveKinematics kinematics,
            SecondOrderSwerveKinematics secondKinematics,
            SwerveDrivePoseEstimator poseEstimator
            )
    {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.kinematics = kinematics;
        this.secondKinematics = secondKinematics;
        this.poseEstimator = poseEstimator;
    }

    public void updateSwerveModuleStates(ChassisSpeeds desiredSpeeds) {
        // Convert robot  relative speeds to field relative speeds if desired
        ChassisSpeeds desiredSpeedsFieldRelative = null;
        if (fieldRelative) {
            desiredSpeedsFieldRelative = desiredSpeeds;
            double xSpeed = desiredSpeeds.vxMetersPerSecond;
            double ySpeed = desiredSpeeds.vyMetersPerSecond;
            double rot = desiredSpeeds.omegaRadiansPerSecond;

            desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, poseEstimator.getEstimatedPosition().getRotation());
        }

        // Convert chassis speeds to individual module states
        // The second order kinematics will return swerveModuleStates and desired rotational speeds of the turn motors
        // Both the swerveModuleStates and the desired turn velocity are saved in the secondOrderSwerveModuleState object
        SecondOrderSwerveModuleStates secondOrderSwerveModuleStates = secondKinematics.toSwerveModuleState(desiredSpeedsFieldRelative, poseEstimator.getEstimatedPosition().getRotation());
        SwerveModuleState[] swerveModuleStates = secondOrderSwerveModuleStates.getSwerveModuleStates();
        double[] moduleTurnSpeeds = secondOrderSwerveModuleStates.getModuleTurnSpeeds();

        //Desaturate the individual module velocity with the right max velocity
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxVelocity);

        SwerveModule[] modules =  new SwerveModule[4];
        modules[0] = frontLeft;
        modules[1] = frontRight;
        modules[2] = backLeft;
        modules[3] = backRight;

        double[] moduleTargetStates = new double[2*4];
        double[] moduleCurrStates = new double[2*4];
        for (int i = 0; i < 4; i++) {

            SwerveModuleState optimizedState = SwerveModuleState.optimize(swerveModuleStates[i], modules[i].getAbsoluteAngle());

            moduleTargetStates[i * 2] = optimizedState.angle.getRadians();
            moduleTargetStates[i * 2 + 1] = optimizedState.speedMetersPerSecond;

            moduleCurrStates[i * 2] = modules[i].getAbsoluteAngle().getRadians();
            moduleCurrStates[i * 2 + 1] = modules[i].getDriveVelocity();
        }

        SmartDashboard.putNumberArray("DT/swerve target states", moduleTargetStates);
        SmartDashboard.putNumberArray("DT/swerve current states", moduleCurrStates);

        // Assign desired module states to modules
        frontLeft.setDesiredState(swerveModuleStates[0], moduleTurnSpeeds[0]);
        frontRight.setDesiredState(swerveModuleStates[1], moduleTurnSpeeds[1]);
        backLeft.setDesiredState(swerveModuleStates[2], moduleTurnSpeeds[2]);
        backRight.setDesiredState(swerveModuleStates[3], moduleTurnSpeeds[3]);
    }

    /**
     * Sets {@code ServeModule} in cross configuration
     */
    public void idleSwerveModuleStates() {
        // TODO add enum for different idle positions
        int numModules = 4;
        SwerveModuleState[] swerveModuleStates = new SwerveModuleState[numModules];
        int[] angles = {45, -45, -45, 45}; // Cross config

        for (int i = 0; i < numModules; i++) {
            Rotation2d angle = Rotation2d.fromDegrees(angles[i]);
            swerveModuleStates[i] = new SwerveModuleState(0.001, angle);
        }

        // Assign desired module states to modules
        frontLeft.setDesiredState(swerveModuleStates[0], 0);
        frontRight.setDesiredState(swerveModuleStates[1], 0);
        backLeft.setDesiredState(swerveModuleStates[2], 0);
        backRight.setDesiredState(swerveModuleStates[3], 0);
    }

    /**
     * Gets the current states of the swerve modules
     *
     * @return the current states of the swerve modules
     */
    public SwerveModuleState[] getSwerveModuleStates() {
        return new SwerveModuleState[]{
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState()
        };
    }

    public void setMaxVelocity(double velocity) {
        maxVelocity = velocity;
    }

    public double getMaxVelocity() {
        return maxVelocity;
    }

    public void setFieldRelative(boolean isFieldRelative) {
        fieldRelative = isFieldRelative;
    }

    public boolean getFieldRelative() {
        return fieldRelative;
    }
}
