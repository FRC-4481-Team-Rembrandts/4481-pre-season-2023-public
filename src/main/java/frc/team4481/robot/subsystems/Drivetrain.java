package frc.team4481.robot.subsystems;

import com.ctre.phoenix.sensors.BasePigeon;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PPLibTelemetry;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4481.lib.path.PathFollower;
import frc.team4481.lib.path.PathPlannerUtil;
import frc.team4481.lib.subsystems.SubsystemBase;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.lib.swerve.*;
import frc.team4481.robot.Constants;
import frc.team4481.robot.configuration.Configuration;
import frc.team4481.robot.configuration.ConfigurationHandler;
import frc.team4481.robot.motion.WeekendSlewRateLimiter;

import static frc.team4481.robot.Constants.HardwareMap.*;
import static frc.team4481.robot.Constants.Drivetrain.*;
import static frc.team4481.robot.Constants.*;
import static frc.team4481.robot.Constants.Field.*;

public class Drivetrain extends SubsystemBase<DrivetrainManager> {
    private final SubsystemHandler subsystemHandler = SubsystemHandler.getInstance();
    private final ConfigurationHandler configHandler = ConfigurationHandler.getInstance();

    public final BasePigeon pigeon;

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    private final SwerveDriveKinematics kinematics;

    private final SecondOrderSwerveKinematics secondKinematics;

    // Odometry
    private final SwerveDrivePoseEstimator poseEstimator;

    private final SwerveDrivetrainHelper swerveDrivetrainHelper;

    // Path Following
    public final HolonomicDriveController driveController = new HolonomicDriveController(
            new PIDConstants(PathFollowing.DRIVE_CONTROLLER_XY_kP, 0, PathFollowing.DRIVE_CONTROLLER_XY_kD),
            new PIDConstants(PathFollowing.DRIVE_CONTROLLER_THETA_kP, 0, 0),
            PathFollowing.MAX_VELOCITY,
            Constants.Drivetrain.DRIVETRAIN_WHEELBASE_DISTANCE*Math.sqrt(2)
    );


    private final PathFollower pathFollower = new PathFollower(driveController);
    public final Field2d field = new Field2d();

    //Slew rate limiter
    private WeekendSlewRateLimiter directionRateLimiter;

    //Objects used for heading correction
    Timer timer = new Timer();
    double previousT;
    double offT;

    // Is this drive train SDS?
    public boolean isSDS = MODULE_TYPE == SWERVE_MODULE_TYPE.SDS_NEO_NEO;

    public Drivetrain(){
        name = "Drivetrain";
        subsystemManager = new DrivetrainManager();

        SmartDashboard.putBoolean("DT/drivetrain is SDS", isSDS);

        pigeon = new BasePigeon(PIGEON_IMU, "");

        SwerveModule[] modules =  new SwerveModule[4];

        for (int i = 0; i < modules.length; i++) {
            SwerveTurnHelper turnHelper;
            if (isSDS) {
                turnHelper = new SwerveTurnHelper(
                        TURN_IDS[i],
                        TURN_PID_VALUES,
                        TURN_FF_VALUES,
                        STALL_LIMIT_TURN,
                        TURN_ENCODER_IDS[i],
                        TURN_ENCODER_OFFSET_DEGREES[i],
                        TURN_GEAR_RATIO
                );
            } else {
                turnHelper = new SwerveTurnHelper(
                        TURN_IDS[i],
                        TURN_PID_VALUES,
                        TURN_FF_VALUES,
                        STALL_LIMIT_TURN
                );
            }
            SwerveDriveHelper driveHelper = new SwerveDriveHelper(
                    DRIVE_IDS[i],
                    INVERTED[i],
                    DRIVE_POSITION_CONVERSION_FACTOR,
                    DRIVE_VELOCITY_CONVERSION_FACTOR,
                    DRIVE_PID_VALUES,
                    DRIVE_FF_VALUES,
                    STALL_LIMIT_DRIVE,
                    DRIVE_IDLE_MODE
            );

            modules[i] = new SwerveModule(i, turnHelper, driveHelper);
        }

        frontLeft = modules[0];
        frontRight = modules[1];
        backLeft = modules[2];
        backRight = modules[3];

        kinematics = new SwerveDriveKinematics(
                FRONT_LEFT_LOCATION,
                FRONT_RIGHT_LOCATION,
                BACK_LEFT_LOCATION,
                BACK_RIGHT_LOCATION
        );

        secondKinematics = new SecondOrderSwerveKinematics(
                FRONT_LEFT_LOCATION,
                FRONT_RIGHT_LOCATION,
                BACK_LEFT_LOCATION,
                BACK_RIGHT_LOCATION
        );

        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
            poseEstimator = new SwerveDrivePoseEstimator(
                    kinematics,
                    Rotation2d.fromDegrees(pigeon.getYaw()),
                    getSwerveModulePositions(),
                    DRIVETRAIN_START_POSITION_RED
            );
            subsystemManager.setTargetHeading(DRIVETRAIN_START_POSITION_RED.getRotation());
        } else {
            poseEstimator = new SwerveDrivePoseEstimator(
                    kinematics,
                    Rotation2d.fromDegrees(pigeon.getYaw()),
                    getSwerveModulePositions(),
                    DRIVETRAIN_START_POSITION_BLUE
            );
            subsystemManager.setTargetHeading(DRIVETRAIN_START_POSITION_BLUE.getRotation());
        }

        subsystemManager.setPathFollower(pathFollower);
        subsystemManager.setPoseEstimator(poseEstimator);

        swerveDrivetrainHelper = new SwerveDrivetrainHelper(
                frontLeft,
                frontRight,
                backLeft,
                backRight,
                kinematics,
                secondKinematics,
                poseEstimator
        );

        swerveDrivetrainHelper.setMaxVelocity(configHandler.getConfig().getMaxDriveVelocity());
        subsystemManager.setVelocityLimit(configHandler.getConfig().getMaxDriveVelocity());
        swerveDrivetrainHelper.setFieldRelative(true);

        directionRateLimiter = new WeekendSlewRateLimiter(MAX_DIRECTION_RATE_LIMIT);
    }

    @Override
    public void onStart(double timestamp) {
        if (DriverStation.isAutonomous()){
            subsystemManager.setControlState(DrivetrainManager.controlState.DISABLED);
        } else {
            subsystemManager.setControlState(DrivetrainManager.controlState.ENABLED);
        }

        zeroSensors();


        subsystemManager.setDriveCurrentLimit(STALL_LIMIT_DRIVE);
        int limit = subsystemManager.getDriveCurrentLimit();
        frontLeft.setDriveCurrentLimit(limit);
        frontRight.setDriveCurrentLimit(limit);
        backLeft.setDriveCurrentLimit(limit);
        backRight.setDriveCurrentLimit(limit);
    }

    @Override
    public void readPeriodicInputs() {

    }

    @Override
    public void onLoop(double timestamp) {
        // Update velocity config
        Configuration config = configHandler.getConfig();
        subsystemManager.setVelocityLimit(config.getMaxDriveVelocity());
        swerveDrivetrainHelper.setMaxVelocity(config.getMaxDriveVelocity());

        // Check if current limit is changed
        if (!subsystemManager.isCurrentLimitUpdated()){
            int limit = subsystemManager.getDriveCurrentLimit();
            frontLeft.setDriveCurrentLimit(limit);
            frontRight.setDriveCurrentLimit(limit);
            backLeft.setDriveCurrentLimit(limit);
            backRight.setDriveCurrentLimit(limit);
            subsystemManager.setCurrentLimitUpdated(true);
        }

        // Use vision measurements
        subsystemManager.setUseVision(checkVisionMeasurements(timestamp));

        // Update current pose using poseEstimator
        subsystemManager.setCurrentPose(
                poseEstimator.updateWithTime(
                        timestamp,
                        Rotation2d.fromDegrees(pigeon.getYaw()),
                        getSwerveModulePositions()
                )
        );

        switch (subsystemManager.getControlState()) {
            case DISABLED:
                swerveDrivetrainHelper.idleSwerveModuleStates();
                break;
            case LOCKED:
                swerveDrivetrainHelper.idleSwerveModuleStates();
                break;
            case ENABLED:
                ChassisSpeeds desiredSpeeds = subsystemManager.getDesiredSpeeds();

                //Correct heading when heading deviates when changing direction
                ChassisSpeeds correctedSpeeds = correctHeading(desiredSpeeds);

                //Limit the rate of change of the direction of the robot based on current speed
                ChassisSpeeds correctedLimitedSpeeds = limitDirectionChange(correctedSpeeds);

                SmartDashboard.putString("DT/currentChassisSpeedsDesire", correctedLimitedSpeeds.toString());

                swerveDrivetrainHelper.updateSwerveModuleStates(correctedLimitedSpeeds);
                break;
            case PATH_FOLLOWING:
                Pose2d currentPose = subsystemManager.getCurrentPose();

                // FIELD RELATIVE input
                ChassisSpeeds targetSpeeds = pathFollower.update(timestamp, currentPose);

                ChassisSpeeds currentSpeeds = getCurrentChassisSpeeds();
                // Velocities for PathPlanner Telemetry
                PPLibTelemetry.setVelocities(
                        PathPlannerUtil.calculateLinearSpeed(currentSpeeds),
                        PathPlannerUtil.calculateLinearSpeed(targetSpeeds),
                        currentSpeeds.omegaRadiansPerSecond,
                        targetSpeeds.omegaRadiansPerSecond);

                swerveDrivetrainHelper.updateSwerveModuleStates(targetSpeeds);
                break;
            case TEST:
                ChassisSpeeds desiredSpeedsTest = subsystemManager.getDesiredSpeeds();
                ChassisSpeeds currentSpeedsTest = getCurrentChassisSpeeds();

                SmartDashboard.putString("DT/test desired chassispeeds", desiredSpeedsTest.toString());
                SmartDashboard.putNumber("DT/test desired omega rad per sec", desiredSpeedsTest.omegaRadiansPerSecond);
                SmartDashboard.putNumber("DT/test current omega rad per sec", currentSpeedsTest.omegaRadiansPerSecond);

                swerveDrivetrainHelper.updateSwerveModuleStates(desiredSpeedsTest);
                break;
        }



    }


    @Override
    public void writePeriodicOutputs() {
    }

    @Override
    public void onStop(double timestamp) {
        swerveDrivetrainHelper.idleSwerveModuleStates();
        terminate();
    }

    @Override
    public void zeroSensors() {
        timer.reset();
        timer.start();
        previousT = 0;
        offT = 0;
    }

    public void resetRotation(Rotation2d rotation){
        resetOdometry(new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(), rotation));
        subsystemManager.setTargetHeading( rotation );
    }

    @Override
    public void terminate() {
        subsystemManager.setControlState(DrivetrainManager.controlState.DISABLED);
    }

    @Override
    public void outputData() {
        SmartDashboard.putNumber("DT/Robot Yaw", getYaw().getDegrees());

        // Update position on the field
        field.setRobotPose(subsystemManager.getCurrentPose());

        SmartDashboard.putString("DT/currentChassisSpeedsMeasure", getCurrentChassisSpeeds().toString());

        SwerveModulePosition[] swervePos = getSwerveModulePositions();

        for (int i = 0; i < 4; i++) {
            SmartDashboard.putNumber("DT/Current angle deg/" + i, swervePos[i].angle.getDegrees());
        }

        SmartDashboard.putNumber("DT/Current rel angel rad/FL", frontLeft.getRelativeAngle());
        SmartDashboard.putNumber("DT/Current rel angel rad/FR", frontRight.getRelativeAngle());
        SmartDashboard.putNumber("DT/Current rel angel rad/BL", backLeft.getRelativeAngle());
        SmartDashboard.putNumber("DT/Current rel angel rad/BR", backRight.getRelativeAngle());

        SmartDashboard.putData(field);
        try {
            SmartDashboard.putString("DT/Current pose", subsystemManager.getCurrentPose().toString());

            SmartDashboard.putNumberArray("DT/Estimated pose array", poseToArray(subsystemManager.getCurrentPose()));
        } catch (Exception e) {}
        SmartDashboard.putBoolean("DT/vision/use vision", subsystemManager.getUseVision());
        SmartDashboard.putNumber("DT/average wheel velocity", getAverageWheelVelocity());
        SmartDashboard.putNumber("DT/front back diff", getFrontBackDifferene());
        SmartDashboard.putString("DT/state", subsystemManager.getControlState().toString());
        SmartDashboard.putNumber("DT/target speed", Math.hypot(subsystemManager.getDesiredSpeeds().vxMetersPerSecond, subsystemManager.getDesiredSpeeds().vyMetersPerSecond));
        SmartDashboard.putNumber("DT/target heading", subsystemManager.getTargetHeading().getDegrees());
        SmartDashboard.putNumber("DT/maximum velocity", subsystemManager.getVelocityLimit());

    }

    public void updateCurrentPose() {
        subsystemManager.setCurrentPose(poseEstimator.getEstimatedPosition());
    }

    /**
     * Set the target heading of the drivetrain such that it faces towards the
     * position that is input
     *
     * @param lookAtTrans {@code Translation2d} object that represents the position
     *                                         that the drivetrain will face
     */
    public void lookAtPosition(Translation2d lookAtTrans){
        Translation2d currentTrans =  subsystemManager.getCurrentPose().getTranslation();
        Translation2d deltaTrans = lookAtTrans.minus(currentTrans);

        subsystemManager.setTargetHeading (
                new Rotation2d( Math.atan2(deltaTrans.getY(), deltaTrans.getX()) )
        );
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose){
        subsystemManager.setCurrentPose(pose);

        poseEstimator.resetPosition(
                Rotation2d.fromDegrees(pigeon.getYaw()),
                getSwerveModulePositions(),
                pose
        );
    }

    /**
     * Gets the current positions of the swerve modules
     *
     * @return the current positions of the swerve modules
     */
    public SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[]{
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        };
    }

    /**
     * Gets the current chassisspeed of the drivetrain
     *
     * @return the current chassisspeed of the drivetrain
     */
    public ChassisSpeeds getCurrentChassisSpeeds() {
        return kinematics.toChassisSpeeds(swerveDrivetrainHelper.getSwerveModuleStates());
    }

    /**
     * Optimizes the chassis speed that is put into the kinematics object to allow the robot to hold its heading
     * when no angular velocity is input.
     * The robot will therefore correct itself when it turns without telling it to do so.
     *
     * @param desiredSpeed desired chassis speed that is input by the controller
     * @return corrected {@code ChassisSpeeds} which takes into account that the robot needs to have the same heading
     * when no rotational speed is input
     */
    private ChassisSpeeds correctHeading(ChassisSpeeds desiredSpeed){

        //Determine time interval
        double currentT = timer.get();
        double dt = currentT - previousT;

        //Get desired rotational speed in radians per second and absolute translational speed in m/s
        double vr = desiredSpeed.omegaRadiansPerSecond;
        double v = Math.hypot(desiredSpeed.vxMetersPerSecond, desiredSpeed.vyMetersPerSecond);

        if (vr > 0.01 || vr < -0.01){
            offT = currentT;
            subsystemManager.setTargetHeading(getYaw());
            return desiredSpeed;
        }
        if (currentT - offT < 0.5){
            subsystemManager.setTargetHeading(getYaw());
            return desiredSpeed;
        }
        if (v < 0.1 && !subsystemManager.isTargetAligning()){
            subsystemManager.setTargetHeading(getYaw());
            return desiredSpeed;
        }

        //Determine target and current heading
        subsystemManager.setTargetHeading( subsystemManager.getTargetHeading().plus(new Rotation2d(vr * dt)) );
        Rotation2d currentHeading = getYaw();

        //Calculate the change in heading that is needed to achieve the target
        Rotation2d deltaHeading = subsystemManager.getTargetHeading().minus(currentHeading);

        if (Math.abs(deltaHeading.getDegrees()) < TURNING_DEADBAND){
            return desiredSpeed;
        }
        double correctedVr = deltaHeading.getRadians() / dt * HEADING_kP;

        previousT = currentT;

        return new ChassisSpeeds(desiredSpeed.vxMetersPerSecond, desiredSpeed.vyMetersPerSecond, correctedVr);
    }

    private ChassisSpeeds limitDirectionChange(ChassisSpeeds desiredSpeed){
        double vr = desiredSpeed.omegaRadiansPerSecond;
        double v = Math.hypot(desiredSpeed.vxMetersPerSecond, desiredSpeed.vyMetersPerSecond);
        double direction = Math.atan2(desiredSpeed.vyMetersPerSecond, desiredSpeed.vxMetersPerSecond);

        double rateLimit = ( configHandler.getConfig().getMaxDriveVelocity() / Math.abs(frontLeft.getDriveVelocity())) * MAX_DIRECTION_RATE_LIMIT * (Math.PI / 180);
        rateLimit = MathUtil.clamp(rateLimit, -1e3, 1e3);
        directionRateLimiter.setRateLimit(rateLimit);
        double limitedDirection = directionRateLimiter.calculate(direction);

        double vx = Math.cos(limitedDirection) * v;
        double vy = Math.sin(limitedDirection) * v;

        return new ChassisSpeeds(vx, vy, vr);
    }

    public void updateOdoWithVision() {
        double[] limelightPoseDouble = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpiblue").getDoubleArray(new double[]{0});
        Pose2d limelightPose = new Pose2d(new Translation2d(limelightPoseDouble[0], limelightPoseDouble[1]), Rotation2d.fromDegrees(limelightPoseDouble[5]));

    }

    private boolean checkVisionMeasurements(double timestamp){
        try {
            double[] limelightPoseDoubleTop = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpiblue").getDoubleArray(new double[]{0});
            double[] limelightTargetSpacePoseDoubleTop = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_targetspace").getDoubleArray(new double[]{0});
            double tagAreaTop = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
            double[] limelightPoseDoubleBottom = NetworkTableInstance.getDefault().getTable("limelight-bottom").getEntry("botpose_wpiblue").getDoubleArray(new double[]{0});
            double[] limelightTargetSpacePoseDoubleBottom = NetworkTableInstance.getDefault().getTable("limelight-bottom").getEntry("botpose_targetspace").getDoubleArray(new double[]{0});
            double tagAreaBottom = NetworkTableInstance.getDefault().getTable("limelight-bottom").getEntry("ta").getDouble(0);

            //Check if the limelight rejects the position because it is too far away
            //This has to be done with the targetspace pose instead of the wpilibblue pose, because limelight still
            //updates the wpilibblue pose when it shouldn't update according to the pipeline settings
            double sumTop = 0;
            for (double value : limelightTargetSpacePoseDoubleTop) {
                sumTop += value;
            }
            boolean topInRange = sumTop != 0;
            double sumBottom = 0;
            for (double value : limelightTargetSpacePoseDoubleTop) {
                sumBottom += value;
            }
            boolean bottomInRange = sumBottom != 0;


            // Stop if limelight array is not full
            // Stop if translation X or Y is not in field
            // Stop if translation Z is negative
            double[] limelightPoseDouble;
            double tagArea;
            boolean topValid = topInRange &&
                    (limelightPoseDoubleTop.length > 5
                    && limelightPoseDoubleTop[0] > 0.75
                    && limelightPoseDoubleTop[0] < FIELD_LENGTH - 0.75
                    && limelightPoseDoubleTop[1] > 0.3
                    && limelightPoseDoubleTop[1] < FIELD_WIDTH - 0.3
                    && limelightPoseDoubleTop[2] >= 0
            );
            boolean bottomValid = bottomInRange &&
                    (limelightPoseDoubleBottom.length > 5
                    && limelightPoseDoubleBottom[0] > 0.75
                    && limelightPoseDoubleBottom[0] < FIELD_LENGTH - 0.75
                    && limelightPoseDoubleBottom[1] > 0.3
                    && limelightPoseDoubleBottom[1] < FIELD_WIDTH - 0.3
                    && limelightPoseDoubleBottom[2] >= 0);

            if (topValid && bottomValid){
                //Take the measurements from the top limelight, because that is the best
                limelightPoseDouble = limelightPoseDoubleTop;
                //Except for the x and y position, take the average of that
                limelightPoseDouble[0] = (limelightPoseDoubleBottom[0] + limelightPoseDoubleTop[0])/2;
                limelightPoseDouble[1] = (limelightPoseDoubleBottom[1] + limelightPoseDoubleTop[1])/2;
                //Take the tag area of the top limelight
                tagArea = tagAreaTop;
            } else if (bottomValid){
                limelightPoseDouble = limelightPoseDoubleBottom;
                tagArea = tagAreaBottom;
            } else if (topValid){
                limelightPoseDouble = limelightPoseDoubleTop;
                tagArea = tagAreaTop;
            } else {
                return false;
            }

            // Make pose from limelight translation and rotation
            // April tag rotation accuracy is lower than pigeon
//            Pose2d limelightPose = new Pose2d(new Translation2d(limelightPoseDouble[0], limelightPoseDouble[1]), Rotation2d.fromDegrees(pigeon.getYaw()));
            Pose2d limelightPose = new Pose2d(new Translation2d(limelightPoseDouble[0], limelightPoseDouble[1]), Rotation2d.fromDegrees(limelightPoseDouble[5]));
//            SmartDashboard.putString("DT/vision/limelight pose", limelightPose.toString());
//            SmartDashboard.putNumber("DT/vision/limelight yaw", limelightPoseDouble[5]);

//            NetworkTableInstance.getDefault().getTable("logtable").getEntry("limelightpose").setDoubleArray(poseToArray(limelightPose));
            SmartDashboard.putNumberArray("DT/vision/LL-top pose", new Double[]{limelightPoseDoubleTop[0], limelightPoseDoubleTop[1], limelightPoseDoubleTop[5]});
            SmartDashboard.putNumberArray("DT/vision/LL-bottom pose", new Double[]{limelightPoseDoubleBottom[0], limelightPoseDoubleBottom[1], limelightPoseDoubleBottom[5]});
            SmartDashboard.putNumberArray("DT/vision/LL final pose", new Double[]{limelightPoseDouble[0], limelightPoseDouble[1], limelightPoseDouble[5]});

            try {
                double[] limelightTagCorners = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tcornxy").getDoubleArray(new double[]{0});
                double[] limelightTagCornersX = new double[4];
                double[] limelightTagCornersY = new double[4];
                double[] limelightBottomTagCorners = NetworkTableInstance.getDefault().getTable("limelight-bottom").getEntry("tcornxy").getDoubleArray(new double[]{0});
                double[] limelightBottomTagCornersX = new double[4];
                double[] limelightBottomTagCornersY = new double[4];
                for (int i = 0; i < 4; i++) {
                    limelightTagCornersX[i] = limelightTagCorners[i * 2];
                    limelightTagCornersY[i] = limelightTagCorners[i * 2 + 1];

                    limelightBottomTagCornersX[i] = limelightBottomTagCorners[i * 2];
                    limelightBottomTagCornersY[i] = limelightBottomTagCorners[i * 2 + 1];
                }
                SmartDashboard.putNumberArray("DT/vision/LL-top tag corners x", limelightTagCornersX);
                SmartDashboard.putNumberArray("DT/vision/LL-top tag corners y", limelightTagCornersY);
                SmartDashboard.putNumberArray("DT/vision/LL-bottom tag corners x", limelightBottomTagCornersX);
                SmartDashboard.putNumberArray("DT/vision/LL-bottom tag corners y", limelightBottomTagCornersY);
            } catch (Exception e){}


            // Add estimator trust using april tag area
            double stdX = 0.13;
            double stdY = stdX;
            if (tagArea < 0.5){
                stdY *= 50;
                stdX *= 5;
            }
            Matrix<N3, N1> stdDevsMatrix = new Matrix<>(Nat.N3(), Nat.N1());
            stdDevsMatrix.set(0,0,stdX);
            stdDevsMatrix.set(1,0,stdY);
            stdDevsMatrix.set(2,0,stdY*10);
            poseEstimator.setVisionMeasurementStdDevs(stdDevsMatrix);
            SmartDashboard.putNumber("DT/vision/april tag std X", stdX);
            SmartDashboard.putNumber("DT/vision/april tag std Y", stdY);

            // Add limelight latency
            double limelightLatency = limelightPoseDouble[6] / 1000;
            poseEstimator.addVisionMeasurement(limelightPose, timestamp - limelightLatency);

            return true;

        } catch (Exception e) {
            return false;
        }
    }

    private Rotation2d getYaw(){
        return poseEstimator.getEstimatedPosition().getRotation();
    }

    private double getAverageWheelVelocity(){
        return (Math.abs(frontLeft.getDriveVelocity()) + Math.abs(frontRight.getDriveVelocity()) + Math.abs(backLeft.getDriveVelocity()) + Math.abs(backRight.getDriveVelocity()))/4;
    }

    private double getFrontBackDifferene(){
        return (Math.abs(frontLeft.getDriveVelocity()) + Math.abs(frontRight.getDriveVelocity()))/2 - (Math.abs(backRight.getDriveVelocity()) + Math.abs(backLeft.getDriveVelocity()))/2;
    }

    private double[] poseToArray(Pose2d pose) {
        double[] arr = new double[3];
        arr[0] = pose.getX();
        arr[1] = pose.getY();
        arr[2] = pose.getRotation().getDegrees();

        return arr;
    }

}