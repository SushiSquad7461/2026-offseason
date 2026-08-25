package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

    // distance from center of robot to wheels
    // These are temporary values until we know the real robot dimensions.
    private static final double TRACK_WIDTH = 0.5;
    private static final double WHEEL_BASE = 0.5;

    // where the four swerve models are 
    private final Translation2d frontLeftLocation =
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2);

    private final Translation2d frontRightLocation =
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2);

    private final Translation2d backLeftLocation =
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2);

    private final Translation2d backRightLocation =
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2);

    // swerve
    private final SwerveDriveKinematics kinematics =
            new SwerveDriveKinematics(
                    frontLeftLocation,
                    frontRightLocation,
                    backLeftLocation,
                    backRightLocation
            );

    public Drivetrain() {
    }
}
