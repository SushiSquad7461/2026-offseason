package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveSubsystem extends SubsystemBase {
    
    // positions of four wheels relative to center of frame (meters)
    private final Translation2d frontLeftLocation = new Translation2d(0.3, 0.3);
    private final Translation2d frontRightLocation = new Translation2d(0.3, -0.3);
    private final Translation2d backLeftLocation = new Translation2d(-0.3, 0.3);
    private final Translation2d backRightLocation = new Translation2d(-0.3, -0.3);

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation
    );

    // gyroscope
    private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

    // serve modules- change id numbers to match the map
    private final SwerveModule frontLeft = new SwerveModule(0, 1, 0, 1);
    private final SwerveModule frontRight = new SwerveModule(2, 3, 2, 3);
    private final SwerveModule backLeft = new SwerveModule(4, 5, 4, 5);
    private final SwerveModule backRight = new SwerveModule(6, 7, 6, 7);

    public SwerveDriveSubsystem() {
        gyro.reset();
    }

    
   // drive instructions with xspeed and yspeed
    
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        // scale to max of the robot 
        ChassisSpeeds speeds = fieldRelative 
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed * 4.5, ySpeed * 4.5, rot * 2.0, gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed * 4.5, ySpeed * 4.5, rot * 2.0);

        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, 4.5);

        frontLeft.setDesiredState(moduleStates[0]);
        frontRight.setDesiredState(moduleStates[1]);
        backLeft.setDesiredState(moduleStates[2]);
        backRight.setDesiredState(moduleStates[3]);
    }
}
