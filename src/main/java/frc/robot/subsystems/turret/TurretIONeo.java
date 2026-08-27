package frc.robot.subsystems.turret;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

public class TurretIONeo implements TurretIO {
    // Replace these constants
    private static final double ENCODER_OFFSET_DEGREES = 0.0;
    private static final double kP = 0.05;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    private final SparkMax turretMotor = new SparkMax(0, MotorType.kBrushless);
    private final SparkAbsoluteEncoder absoluteEncoder = turretMotor.getAbsoluteEncoder();
    private final SparkClosedLoopController turretController = turretMotor.getClosedLoopController();

    public TurretIONeo() {
        SparkMaxConfig turretConfig = new SparkMaxConfig();

        turretConfig.absoluteEncoder
                .positionConversionFactor(360.0)
                .velocityConversionFactor(360.0 / 60.0)
                .zeroOffset(ENCODER_OFFSET_DEGREES)
                .inverted(false);

        turretConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .p(kP)
                .i(kI)
                .d(kD);

        turretConfig.softLimit
                .reverseSoftLimit(0)
                .reverseSoftLimitEnabled(true)
                .forwardSoftLimit(360)
                .forwardSoftLimitEnabled(true);

        turretConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(40);

        turretMotor.configure(
                turretConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public void turnDegrees(double targetAngleDegrees) {
        turretController.setSetpoint(targetAngleDegrees, ControlType.kPosition);
    }

    public double getCurrentAngle() {
        return absoluteEncoder.getPosition();
    }

    public void moveToCenter() {
        turnDegrees(180);
    }

    public void stop() {
        turretMotor.stopMotor();
    }
}