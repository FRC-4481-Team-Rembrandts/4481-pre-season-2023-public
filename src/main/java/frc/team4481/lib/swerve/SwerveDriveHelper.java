package frc.team4481.lib.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class SwerveDriveHelper 
{
    public CANSparkMax driveMotor;
    public SimpleMotorFeedforward FFController;

    /**
     *
     * @param motorID
     * @param isInverted
     * @param PIDValues
     * @param FFValues
     */
    public SwerveDriveHelper(
            int motorID,
            boolean isInverted,
            double encoderPositionConversionFactor,
            double encoderVelocityConversionFactor,
            PIDValueContainer PIDValues,
            FFValueContainer FFValues,
            int current_limit,
            CANSparkMax.IdleMode idleMode
            )
    {
        driveMotor = new CANSparkMax(motorID, CANSparkMax.MotorType.kBrushless);
        driveMotor.restoreFactoryDefaults();
        driveMotor.setIdleMode(idleMode);
        driveMotor.setInverted(isInverted);
        driveMotor.setSmartCurrentLimit(current_limit);

        RelativeEncoder relativeEncoder = driveMotor.getEncoder();
        relativeEncoder.setPositionConversionFactor(encoderPositionConversionFactor);
        relativeEncoder.setVelocityConversionFactor(encoderVelocityConversionFactor);

        SparkPIDController controller = driveMotor.getPIDController();
        controller.setFeedbackDevice(relativeEncoder);
        controller.setP(PIDValues.kP);
        controller.setI(PIDValues.kI);
        controller.setD(PIDValues.kD);
        driveMotor.burnFlash();

        FFController = new SimpleMotorFeedforward(
                FFValues.kS,
                FFValues.kV,
                FFValues.kA
        );
    }
}
