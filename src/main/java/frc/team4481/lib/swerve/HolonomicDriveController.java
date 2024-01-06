package frc.team4481.lib.swerve;

// Class adapted from PPHolonomicDriveController from PathplannerLib

import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class HolonomicDriveController {
    private final PIDController xController;
    private final PIDController yController;
    private final ProfiledPIDController rotationController;
    private final double maxModuleSpeed;
    private final double mpsToRps;

    public HolonomicDriveController(PIDConstants translationConstants, PIDConstants rotationConstants, double period, double maxModuleSpeed, double driveBaseRadius) {
        this.xController = new PIDController(translationConstants.kP, translationConstants.kI, translationConstants.kD, period);
        this.xController.setIntegratorRange(-translationConstants.iZone, translationConstants.iZone);
        this.yController = new PIDController(translationConstants.kP, translationConstants.kI, translationConstants.kD, period);
        this.yController.setIntegratorRange(-translationConstants.iZone, translationConstants.iZone);
        this.rotationController = new ProfiledPIDController(rotationConstants.kP, rotationConstants.kI, rotationConstants.kD, new TrapezoidProfile.Constraints(0.0, 0.0), period);
        this.rotationController.setIntegratorRange(-rotationConstants.iZone, rotationConstants.iZone);
        this.rotationController.enableContinuousInput(-3.141592653589793, Math.PI);
        this.maxModuleSpeed = maxModuleSpeed;
        this.mpsToRps = 1.0 / driveBaseRadius;
    }

    public HolonomicDriveController(PIDConstants translationConstants, PIDConstants rotationConstants, double maxModuleSpeed, double driveBaseRadius) {
        this(translationConstants, rotationConstants, 0.02, maxModuleSpeed, driveBaseRadius);
    }

    public void reset(Pose2d currentPose, ChassisSpeeds currentSpeeds) {
        this.rotationController.reset(currentPose.getRotation().getRadians(), currentSpeeds.omegaRadiansPerSecond);
    }

    public ChassisSpeeds calculateFieldRelativeSpeeds(Pose2d currentPose, PathPlannerTrajectory.State targetState) {
        double xFF = targetState.velocityMps * targetState.heading.getCos();
        double yFF = targetState.velocityMps * targetState.heading.getSin();
        double xFeedback = this.xController.calculate(currentPose.getX(), targetState.positionMeters.getX());
        double yFeedback = this.yController.calculate(currentPose.getY(), targetState.positionMeters.getY());
        double angVelConstraint = targetState.constraints.getMaxAngularVelocityRps();
        double maxAngVelModule = Math.max(0.0, this.maxModuleSpeed - targetState.velocityMps) * this.mpsToRps;
        double maxAngVel = Math.min(angVelConstraint, maxAngVelModule);
        TrapezoidProfile.Constraints rotationConstraints = new TrapezoidProfile.Constraints(maxAngVel, targetState.constraints.getMaxAngularAccelerationRpsSq());
        double targetRotationVel = this.rotationController.calculate(currentPose.getRotation().getRadians(), new TrapezoidProfile.State(targetState.targetHolonomicRotation.getRadians(), 0.0), rotationConstraints);

        return new ChassisSpeeds(xFF + xFeedback, yFF + yFeedback, targetRotationVel);
    }
}
