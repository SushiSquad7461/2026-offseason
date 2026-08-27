package frc.robot.subsystems.turret;

public interface TurretIO {
    void turnDegrees(double degrees);
    void stop();
    double getCurrentAngle();
    void moveToCenter();
}
