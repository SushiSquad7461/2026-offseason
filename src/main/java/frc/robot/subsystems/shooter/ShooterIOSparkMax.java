package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;


/**
 * Physical hardware implementation of the combined Shooter and Hood using NEO motors.
 */
public class ShooterIOSparkMax implements ShooterIO {
    // Flywheel Hardware (The left/right motors power both the main flywheels AND backspin wheels)
    private final SparkMax leftMotor;
    private final SparkMax rightMotor;
    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    // Hood Hardware
    private final SparkMax hoodMotor;
    private final RelativeEncoder hoodEncoder;
    private static final double HOOD_ROTATIONS_TO_DEGREES = 360.0 / 10.0; // Example 10:1 gear ratio

    // Hardware PID Controllers
    private final SparkPIDController leftPid;
    private final SparkPIDController rightPid;
    private final SparkPIDController hoodPid;

    public ShooterIOSparkMax() {
        // Initialize Flywheel motors (IDs 10, 11)
        leftMotor = new SparkMax(10, MotorType.kBrushless);
        rightMotor = new SparkMax(11, MotorType.kBrushless);
        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();
        leftMotor.setInverted(false);
        rightMotor.setInverted(true);
        
        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();

        leftPid = leftMotor.getPIDController();
        rightPid = rightMotor.getPIDController();
        
        // P value for Velocity Control onboard
        leftPid.setP(0.001);
        rightPid.setP(0.001);

        // Initialize Hood motor (ID 12)
        hoodMotor = new SparkMax(12, MotorType.kBrushless);
        hoodMotor.restoreFactoryDefaults();
        hoodMotor.setInverted(false);
        
        hoodEncoder = hoodMotor.getEncoder();
        hoodPid = hoodMotor.getPIDController();

        // P value for Position Control onboard
        hoodPid.setP(0.05);

        // Save settings to flash
        leftMotor.burnFlash();
        rightMotor.burnFlash();
        hoodMotor.burnFlash();
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // Log flywheels
        inputs.leftFlywheelVelocityRPM = leftEncoder.getVelocity();
        inputs.leftFlywheelAppliedVolts = leftMotor.getAppliedOutput() * leftMotor.getBusVoltage();
        inputs.leftFlywheelCurrentAmps = new double[] { leftMotor.getOutputCurrent() };

        inputs.rightFlywheelVelocityRPM = rightEncoder.getVelocity();
        inputs.rightFlywheelAppliedVolts = rightMotor.getAppliedOutput() * rightMotor.getBusVoltage();
        inputs.rightFlywheelCurrentAmps = new double[] { rightMotor.getOutputCurrent() };

        // Log hood
        inputs.hoodAngleDegrees = hoodEncoder.getPosition() * HOOD_ROTATIONS_TO_DEGREES;
        inputs.hoodAppliedVolts = hoodMotor.getAppliedOutput() * hoodMotor.getBusVoltage();
        inputs.hoodCurrentAmps = new double[] { hoodMotor.getOutputCurrent() };
    }

    @Override
    public void setFlywheelVelocityRPM(double rpm, double feedforwardVolts) {
        // Runs at 1000hz on the SparkMax directly! ArbFF adds the baseline voltage.
        leftPid.setReference(rpm, SparkMax.ControlType.kVelocity, 0, feedforwardVolts, SparkPIDController.ArbFFUnits.kVoltage);
        rightPid.setReference(rpm, SparkMax.ControlType.kVelocity, 0, feedforwardVolts, SparkPIDController.ArbFFUnits.kVoltage);
    }
    
    @Override
    public void setHoodPositionDegrees(double degrees) {
        // Convert degrees back into motor rotations for the SparkMax
        double rotations = degrees / HOOD_ROTATIONS_TO_DEGREES;
        hoodPid.setReference(rotations, SparkMax.ControlType.kPosition);
    }

    @Override
    public void stop() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
        hoodMotor.stopMotor();
    }
}
