package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/**
 * Combined AdvantageKit IO layer for the Shooter and Hood.
 */
public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        // Flywheel inputs
        public double leftFlywheelVelocityRPM = 0.0;
        public double leftFlywheelAppliedVolts = 0.0;
        public double[] leftFlywheelCurrentAmps = new double[] {};

        public double rightFlywheelVelocityRPM = 0.0;
        public double rightFlywheelAppliedVolts = 0.0;
        public double[] rightFlywheelCurrentAmps = new double[] {};

        // Hood inputs
        public double hoodAngleDegrees = 0.0;
        public double hoodAppliedVolts = 0.0;
        public double[] hoodCurrentAmps = new double[] {};
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ShooterIOInputs inputs) {}

    /** Sets the voltage applied to the flywheels. */
    public default void setFlywheelVoltage(double volts) {}

    /** Sets the voltage applied to the hood motor. */
    public default void setHoodVoltage(double volts) {}

    /** Stops all motors in the subsystem. */
    public default void stop() {}
}
