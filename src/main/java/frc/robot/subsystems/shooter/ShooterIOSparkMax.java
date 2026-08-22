package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * Physical hardware implementation of the combined Shooter and Hood using NEO motors.
 */
public class ShooterIOSparkMax implements ShooterIO {
    // Flywheel Hardware (The left/right motors power both the main flywheels AND backspin wheels)
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    // Hood Hardware
    private final CANSparkMax hoodMotor;
    private final RelativeEncoder hoodEncoder;
    private static final double HOOD_ROTATIONS_TO_DEGREES = 360.0 / 10.0; // Example 10:1 gear ratio

    public ShooterIOSparkMax() {
        // Initialize Flywheel motors (IDs 10, 11)
        leftMotor = new CANSparkMax(10, MotorType.kBrushless);
        rightMotor = new CANSparkMax(11, MotorType.kBrushless);
        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();
        leftMotor.setInverted(false);
        rightMotor.setInverted(true);
        
        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();

        // Initialize Hood motor (ID 12)
        hoodMotor = new CANSparkMax(12, MotorType.kBrushless);
        hoodMotor.restoreFactoryDefaults();
        hoodMotor.setInverted(false);
        
        hoodEncoder = hoodMotor.getEncoder();

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
    public void setFlywheelVoltage(double volts) {
        leftMotor.setVoltage(volts);
        rightMotor.setVoltage(volts);
    }
    
    @Override
    public void setHoodVoltage(double volts) {
        hoodMotor.setVoltage(volts);
    }

    @Override
    public void stop() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
        hoodMotor.stopMotor();
    }
}
