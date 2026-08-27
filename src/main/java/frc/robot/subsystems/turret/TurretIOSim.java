package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;

public class TurretIOSim implements TurretIO {

    private double simulatedDegrees = 0;
    private double targetDegrees = 0;

    @Override
    public void turnDegrees(double degrees) {
        targetDegrees = MathUtil.clamp(
                simulatedDegrees + degrees,
                0,
                360);

        simulatedDegrees = targetDegrees;
    }

    @Override
    public double getCurrentAngle() {
        return simulatedDegrees;
    }

    @Override
    public void moveToCenter() {
        targetDegrees = 0;
        simulatedDegrees = targetDegrees;
    }

    @Override
    public void stop() {
        targetDegrees = simulatedDegrees;
    }
}