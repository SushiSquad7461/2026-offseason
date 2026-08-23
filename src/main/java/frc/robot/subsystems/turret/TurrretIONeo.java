package frc.robot.subsystems.turret;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.AbsoluteEncoder;



public class TurrretIONeo extends TurretIO {
    private final SparkMax turret;
    private final AbsoluteEncoder absoluteEncoder;
    private final SparkClosedLoopController turretController;
    ;

    public TurretIONeo() {
        absoluteEncoder = turret.getAbsoluteEncoder();
        turretController = turret.getClosedLoopController();

        SparkMaxConfig turretConfig = new SparkMaxConfig();

        config.absoluteEncoder
            .positionConversionFactor(360.0)     
            .velocityConversionFactor(360.0 / 60.0) 
            .zeroOffset(ENCODER_OFFSET_DEGREES)   
            .inverted(false);

        config.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .p(kP)
            .i(kI)
            .d(kD);
        config.softLimit
            .reverseSoftLimit(0)
            .reverseSoftLimitEnabled(true)
            .forwardSoftLimit(360)
            .forwardSoftLimitEnabled(true);
        config
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);
        turretMotor.configure(
            config, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );
    }

    public void turnDegrees(double targetAngleDegrees) {
        turretController.setReference(targetAngleDegrees, ControlType.kPosition);
    }

    public double getCurrentAngle() {
        return absoluteEncoder.getPosition();
    }

    public void moveToCenter() {
        turretMotor.stopMotor();
    }
}
