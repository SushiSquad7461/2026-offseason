package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.PIDController;

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

    // Because the Simulation doesn't have a real SparkMax doing the math 1000x a second, 
    // we fake the hardware PID loop right here in the simulation code!
    private final PIDController simLeftFlywheelPid = new PIDController(0.001, 0, 0);
    private final PIDController simRightFlywheelPid = new PIDController(0.001, 0, 0);
    private final PIDController simHoodPid = new PIDController(0.05, 0, 0);

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
    public void setFlywheelVelocityRPM(double rpm, double feedforwardVolts) {
        // Emulate the SparkMax calculating voltage internally
        leftAppliedVolts = feedforwardVolts + simLeftFlywheelPid.calculate(leftFlywheelSim.getAngularVelocityRPM(), rpm);
        rightAppliedVolts = feedforwardVolts + simRightFlywheelPid.calculate(rightFlywheelSim.getAngularVelocityRPM(), rpm);
        
        leftFlywheelSim.setInputVoltage(leftAppliedVolts);
        rightFlywheelSim.setInputVoltage(rightAppliedVolts);
    }
    
    @Override
    public void setHoodPositionDegrees(double degrees) {
        // Emulate the SparkMax position control
        hoodAppliedVolts = simHoodPid.calculate(Units.radiansToDegrees(hoodSim.getAngleRads()), degrees);
        hoodSim.setInputVoltage(hoodAppliedVolts);
    }

    @Override
    public void stop() {
        leftAppliedVolts = 0.0;
        rightAppliedVolts = 0.0;
        hoodAppliedVolts = 0.0;
        leftFlywheelSim.setInputVoltage(0);
        rightFlywheelSim.setInputVoltage(0);
        hoodSim.setInputVoltage(0);
    }
}
