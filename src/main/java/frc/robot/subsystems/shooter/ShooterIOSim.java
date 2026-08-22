package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.math.util.Units;

/**
 * Combined Simulation physics logic for the Shooter and Hood.
 */
public class ShooterIOSim implements ShooterIO {
    // Flywheel sim
    private final FlywheelSim leftFlywheelSim = new FlywheelSim(DCMotor.getNEO(1), 1.0, 0.005);
    private final FlywheelSim rightFlywheelSim = new FlywheelSim(DCMotor.getNEO(1), 1.0, 0.005);
    
    // Hood sim
    private final SingleJointedArmSim hoodSim = new SingleJointedArmSim(
        DCMotor.getNEO(1),      // Gearbox: 1 NEO
        10.0,                   // Gearing ratio: 10 to 1
        0.5,                    // Moment Of Inertia (kg m^2)
        0.3,                    // Length of the hood (meters)
        Units.degreesToRadians(8), // Min angle (hard stop)
        Units.degreesToRadians(40),// Max angle (hard stop)
        true,                   // Simulate gravity
        Units.degreesToRadians(8)  // Starting angle (resting at min)
    );

    private double leftAppliedVolts = 0.0;
    private double rightAppliedVolts = 0.0;
    private double hoodAppliedVolts = 0.0;

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // Step simulations forward
        leftFlywheelSim.update(0.02);
        rightFlywheelSim.update(0.02);
        hoodSim.update(0.02);

        // Update flywheel inputs
        inputs.leftFlywheelVelocityRPM = leftFlywheelSim.getAngularVelocityRPM();
        inputs.leftFlywheelAppliedVolts = leftAppliedVolts;
        inputs.leftFlywheelCurrentAmps = new double[] { leftFlywheelSim.getCurrentDrawAmps() };

        inputs.rightFlywheelVelocityRPM = rightFlywheelSim.getAngularVelocityRPM();
        inputs.rightFlywheelAppliedVolts = rightAppliedVolts;
        inputs.rightFlywheelCurrentAmps = new double[] { rightFlywheelSim.getCurrentDrawAmps() };

        // Update hood inputs
        inputs.hoodAngleDegrees = Units.radiansToDegrees(hoodSim.getAngleRads());
        inputs.hoodAppliedVolts = hoodAppliedVolts;
        inputs.hoodCurrentAmps = new double[] { hoodSim.getCurrentDrawAmps() };
    }

    @Override
    public void setFlywheelVoltage(double volts) {
        leftAppliedVolts = volts;
        rightAppliedVolts = volts;
        leftFlywheelSim.setInputVoltage(volts);
        rightFlywheelSim.setInputVoltage(volts);
    }
    
    @Override
    public void setHoodVoltage(double volts) {
        hoodAppliedVolts = volts;
        hoodSim.setInputVoltage(volts);
    }

    @Override
    public void stop() {
        setFlywheelVoltage(0);
        setHoodVoltage(0);
    }
}
