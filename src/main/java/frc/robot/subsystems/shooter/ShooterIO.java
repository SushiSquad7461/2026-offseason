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

    /** 
     * Sets the target velocity for the flywheels (handled by onboard SparkMax PID).
     * @param rpm Target RPM
     * @param feedforwardVolts The calculated baseline baseline voltage to assist the PID
     */
    public default void setFlywheelVelocityRPM(double rpm, double feedforwardVolts) {}

    /** 
     * Sets the target angle for the hood (handled by onboard SparkMax PID).
     * @param degrees Target angle
     */
    public default void setHoodPositionDegrees(double degrees) {}

    /** Stops all motors in the subsystem. */
    public default void stop() {}
}
