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
    // initialized all required hardware components
    private final SparkMax turretMotor = new SparkMax(0, MotorType.kBrushless);
    private final SparkAbsoluteEncoder absoluteEncoder = turretMotor.getAbsoluteEncoder();
    private final SparkClosedLoopController turretController = turretMotor.getClosedLoopController();

    public TurretIONeo() {

        // initialized config for the motor
        SparkMaxConfig turretConfig = new SparkMaxConfig();

        // added conversation factors to the absoulute encoder
        turretConfig.absoluteEncoder
        .positionConversionFactor(360.0)
        .velocityConversionFactor(360.0 / 60.0)
        .zeroOffset(ENCODER_OFFSET_DEGREES)
        .inverted(false);

        // added PID constants and feedback sensor
        turretConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .p(kP)
        .i(kI)
        .d(kD);

        // limited rotation to between 0 and 360
        turretConfig.softLimit
        .reverseSoftLimit(0)
        .reverseSoftLimitEnabled(true)
        .forwardSoftLimit(360)
        .forwardSoftLimitEnabled(true);

        // braking when idle
        turretConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40);

        // added all configuration to the motor
        turretMotor.configure(
        turretConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    }
    
    @Override
    // turns the motor to a target angle using position control
    public void turnDegrees(double targetAngleDegrees) {
        turretController.setSetpoint(targetAngleDegrees, ControlType.kPosition);
    }

    @Override
    // returns current angle of the turret
    public double getCurrentAngle() {
        return absoluteEncoder.getPosition();
    }

    @Override
    // centers the turret - 180 degrees
    public void moveToCenter() {
        turnDegrees(180);
    }

    @Override
    // make the turret stop turning
    public void stop() {
        turretMotor.stopMotor();
    }
}