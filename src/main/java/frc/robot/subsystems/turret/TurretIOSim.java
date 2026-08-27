package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;

public class TurretIOSim implements TurretIO {

    
    private double simulatedDegrees = 0; // current angle of the turret
    private double targetDegrees = 0; // angle the turret should move to

    @Override
    // turns the turret to "degrees" angle - clamped between 0 - 360
    public void turnDegrees(double degrees) {
        targetDegrees = MathUtil.clamp(
                degrees,
                0,
                360);

        simulatedDegrees = targetDegrees;
    }

    @Override
    // returns current angle of the turret
    public double getCurrentAngle() {
        return simulatedDegrees;
    }

    @Override
    // moves the turret to face forward
    public void moveToCenter() {
        targetDegrees = 180;
        simulatedDegrees = targetDegrees;
    }

    @Override
    // stops the turret
    public void stop() {
        targetDegrees = simulatedDegrees;
    }
}