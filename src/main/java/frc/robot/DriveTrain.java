package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

    // distance from center of robot to wheels in meters
    // i js put random numbers cause idk the actual numbers
    private static final double TRACK_WIDTH = 0.8;
    private static final double WHEEL_BASE = 0.8;

    // where the four swerve modules are
    private final Translation2d frontLeftLocation =
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2);

    private final Translation2d frontRightLocation =
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2);

    private final Translation2d backLeftLocation =
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2);

    private final Translation2d backRightLocation =
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2);


    public Drivetrain() {
    }
}
