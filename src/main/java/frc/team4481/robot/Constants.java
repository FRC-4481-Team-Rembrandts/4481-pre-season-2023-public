package frc.team4481.robot;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.team4481.lib.swerve.FFValueContainer;
import frc.team4481.lib.swerve.PIDValueContainer;

/*
* IMPORTANT:
* If you want to switch between rev and sds swerve modules do CTRL+F and type SDS
* then you will find the places where some constants need to be changed
* */

public class Constants {
    /* ---------------------------------------- */
    /* LOOPER */
    /* ---------------------------------------- */
    public static final double kLooperDt = 0.02;
    public static double kHIDLooperDt = 0.02;

    public enum SWERVE_MODULE_TYPE {
        REV_NEO_NEO550(0),
        SDS_NEO_NEO(1),
        REV_VORTEX_NEO550(2);

        public final int index;

        SWERVE_MODULE_TYPE(int index) {
            this.index = index;
        }
    }

    // IMPORTANT: change this value based on swerve module type
    public static final SWERVE_MODULE_TYPE MODULE_TYPE = SWERVE_MODULE_TYPE.SDS_NEO_NEO;
    /**
     * Static class that holds all the CAN IDs and IO port addresses on the RoboRIO
     */
    public static class HardwareMap {
        // Sensors
        public static final int PIGEON_IMU = new int[]{9, 15}[MODULE_TYPE.index];

        // Drivetrain
        public static final int DT_FRONT_LEFT_DRIVE_ID = new int[]{11, 21}[MODULE_TYPE.index];
        public static final int DT_FRONT_LEFT_TURN_ID = new int[]{12, 22}[MODULE_TYPE.index];
        public static final boolean DT_FRONT_LEFT_INVERTED = new boolean[]{true, false}[MODULE_TYPE.index];
        public static final int DT_FRONT_LEFT_TURN_ENCODER_ID = new int[]{-1, 13}[MODULE_TYPE.index];
        public static final double DT_FRONT_LEFT_TURN_ENCODER_OFFSET = new double[]{Double.NaN, 105}[MODULE_TYPE.index];

        public static final int DT_FRONT_RIGHT_DRIVE_ID = new int[]{13, 23}[MODULE_TYPE.index];
        public static final int DT_FRONT_RIGHT_TURN_ID = new int[]{14, 24}[MODULE_TYPE.index];
        public static final boolean DT_FRONT_RIGHT_INVERTED = new boolean[]{false, true}[MODULE_TYPE.index];
        public static final int DT_FRONT_RIGHT_TURN_ENCODER_ID = new int[]{-1, 12}[MODULE_TYPE.index];
        public static final double DT_FRONT_RIGHT_TURN_ENCODER_OFFSET = new double[]{Double.NaN, 32}[MODULE_TYPE.index];

        public static final int DT_BACK_LEFT_DRIVE_ID = new int[]{15, 25}[MODULE_TYPE.index];
        public static final int DT_BACK_LEFT_TURN_ID = new int[]{16, 26}[MODULE_TYPE.index];
        public static final boolean DT_BACK_LEFT_INVERTED = new boolean[]{true, false}[MODULE_TYPE.index];
        public static final int DT_BACK_LEFT_TURN_ENCODER_ID = new int[]{-1, 11}[MODULE_TYPE.index];
        public static final double DT_BACK_LEFT_TURN_ENCODER_OFFSET = new double[]{Double.NaN, 1}[MODULE_TYPE.index];

        public static final int DT_BACK_RIGHT_DRIVE_ID = new int[]{17, 27}[MODULE_TYPE.index];
        public static final int DT_BACK_RIGHT_TURN_ID = new int[]{18, 28}[MODULE_TYPE.index];
        public static final boolean DT_BACK_RIGHT_INVERTED = new boolean[]{false, true}[MODULE_TYPE.index];
        public static final int DT_BACK_RIGHT_TURN_ENCODER_ID = new int[]{-1, 14}[MODULE_TYPE.index];
        public static final double DT_BACK_RIGHT_TURN_ENCODER_OFFSET = new double[]{Double.NaN, 238}[MODULE_TYPE.index];

        public static final int[] DRIVE_IDS =
                { DT_FRONT_LEFT_DRIVE_ID, DT_FRONT_RIGHT_DRIVE_ID, DT_BACK_LEFT_DRIVE_ID, DT_BACK_RIGHT_DRIVE_ID };
        public static final int[] TURN_IDS =
                { DT_FRONT_LEFT_TURN_ID, DT_FRONT_RIGHT_TURN_ID, DT_BACK_LEFT_TURN_ID, DT_BACK_RIGHT_TURN_ID };
        public static final boolean[] INVERTED =
                { DT_FRONT_LEFT_INVERTED, DT_FRONT_RIGHT_INVERTED, DT_BACK_LEFT_INVERTED, DT_BACK_RIGHT_INVERTED };
        public static final int[] TURN_ENCODER_IDS =
                {DT_FRONT_LEFT_TURN_ENCODER_ID, DT_FRONT_RIGHT_TURN_ENCODER_ID, DT_BACK_LEFT_TURN_ENCODER_ID, DT_BACK_RIGHT_TURN_ENCODER_ID};
        public static final double[] TURN_ENCODER_OFFSET_DEGREES =
                {DT_FRONT_LEFT_TURN_ENCODER_OFFSET, DT_FRONT_RIGHT_TURN_ENCODER_OFFSET, DT_BACK_LEFT_TURN_ENCODER_OFFSET, DT_BACK_RIGHT_TURN_ENCODER_OFFSET};


    }

    /**
     * Static class containing all the constants for the drivetrain subsystem
     */
    public static class Drivetrain {
        /**
         * 1-dimensional distance between the center of a wheel and the center of the drivetrain in m
         */
        public static final double DRIVETRAIN_WHEELBASE_DISTANCE = new double[]{0.285, 0.45 / 2}[MODULE_TYPE.index];
        public static final double DRIVETRAIN_WIDTH = new double[]{0.82, 0.82}[MODULE_TYPE.index];
        public static final double DRIVE_GEAR_RATIO = new double[]{4.71, 1 / ((14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 60.0))}[MODULE_TYPE.index];
        public static final double WHEEL_RADIUS = new double[]{0.0762 / 2, 0.1016 / 2}[MODULE_TYPE.index];
        public static final double TURN_GEAR_RATIO = new double[]{2.89 * 3.61 /14 * 62, (15.0 / 32.0) * (10.0 / 60.0)}[MODULE_TYPE.index];


        public static final int STALL_LIMIT_DRIVE = 60; //Amps
        public static final int STALL_LIMIT_TURN = 23; //in Amps

        public static final Translation2d FRONT_LEFT_LOCATION =
                new Translation2d(DRIVETRAIN_WHEELBASE_DISTANCE, DRIVETRAIN_WHEELBASE_DISTANCE);
        public static final Translation2d FRONT_RIGHT_LOCATION =
                new Translation2d(DRIVETRAIN_WHEELBASE_DISTANCE, -DRIVETRAIN_WHEELBASE_DISTANCE);
        public static final Translation2d BACK_LEFT_LOCATION =
                new Translation2d(-DRIVETRAIN_WHEELBASE_DISTANCE, DRIVETRAIN_WHEELBASE_DISTANCE);
        public static final Translation2d BACK_RIGHT_LOCATION =
                new Translation2d(-DRIVETRAIN_WHEELBASE_DISTANCE, -DRIVETRAIN_WHEELBASE_DISTANCE);

        /**
         *  Maximum limit for the robot heading slew rate limiter in DEG/S
         *  The used limit depends on the current speed of the robot
         */
        public static final double MAX_DIRECTION_RATE_LIMIT = 250;

        /**
         * Margin that the heading correction algorithm has in degrees
         */
        public static final double TURNING_DEADBAND = 2;
        public static final double HEADING_kP = 0.04; //P constant for heading correction

        /**
         * Drive motor RPM to wheel velocity in m/s
         */
        public static final double DRIVE_VELOCITY_CONVERSION_FACTOR = (2 * Math.PI * WHEEL_RADIUS)/(DRIVE_GEAR_RATIO * 60);
        /**
         * Drive motor revolutions to distance in m
         */
        public static final double DRIVE_POSITION_CONVERSION_FACTOR = (2 * Math.PI * WHEEL_RADIUS)/(DRIVE_GEAR_RATIO);
        /**
         * Turn motor RPM to module angular velocity in rad/s
         */
        public static final double TURN_VELOCITY_CONVERSION_FACTOR = 2 * Math.PI / (TURN_GEAR_RATIO * 60);
        /**
         * Turn motor rotations to radians
         */
        public static final double TURN_POSITION_CONVERSION_FACTOR = 2 * Math.PI / TURN_GEAR_RATIO;


        public static final double DRIVE_kP = 0.1;
        public static final double DRIVE_kI = 0;
        public static final double DRIVE_kD = 0;

        public static final double DRIVE_kA = 0;
        public static final double DRIVE_kS = 0.175;
        public static final double DRIVE_kV = new double[]{2.50, 2.95}[MODULE_TYPE.index];

        public static final double TURN_kA = 0;
        public static final double TURN_kS = 0;
        public static final double TURN_kV = 0;

        public static final double TURN_kP = 1.0;
        public static final double TURN_kI = 0;
        public static final double TURN_kD = 0;

        public static final PIDValueContainer TURN_PID_VALUES = new PIDValueContainer(TURN_kP, TURN_kI, TURN_kD);
        public static final FFValueContainer TURN_FF_VALUES = new FFValueContainer(TURN_kS, TURN_kV, TURN_kA);
        public static final PIDValueContainer DRIVE_PID_VALUES = new PIDValueContainer(DRIVE_kP, DRIVE_kI, DRIVE_kD);
        public static final FFValueContainer DRIVE_FF_VALUES = new FFValueContainer(DRIVE_kS, DRIVE_kV, DRIVE_kA);

        public static final CANSparkMax.IdleMode DRIVE_IDLE_MODE = CANSparkMax.IdleMode.kBrake;

        public static final Pose2d DRIVETRAIN_START_POSITION_BLUE = new Pose2d();
        public static final Pose2d DRIVETRAIN_START_POSITION_RED = new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180));
    }

    /**
     * Static class containing important field dimensions
     */
    public static class Field {
        public static final double FIELD_WIDTH = 8.20;
        public static final double FIELD_LENGTH = 16.5;
    }

    /* ---------------------------------------- */
    /* PATH PLANNING */
    /* ---------------------------------------- */
    /*
     * Maximum robot acceleration in m/s^2
     * Maximum robot velocity in m/s
     * Minimum robot velocity in m/s
     */

    public static class PathFollowing {
        public static final double MAX_VELOCITY = 5;
        public static final double MAX_TURN_VELOCITY = 2 * Math.PI;

        public static final double DRIVE_CONTROLLER_XY_kP = 1;
        public static final double DRIVE_CONTROLLER_XY_kD = 0;
        public static final double DRIVE_CONTROLLER_THETA_kP = 4;
    }

    /* ---------------------------------------- */
    /* CONTROLLERS */
    /* ---------------------------------------- */
    public static final double ORANGE_LEFTSTICK_DEADBAND = 0.03;
    public static final double ORANGE_RIGHTSTICK_DEADBAND = 0.03;

}