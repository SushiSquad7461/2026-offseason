package frc.robot.subsystems.turret;

public interface TurretIO {
    void turnDegrees(double degrees);
    double getCurrentAngle();
    void moveToCenter();
}
