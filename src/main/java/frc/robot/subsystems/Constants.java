package frc.robot.generated;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


import.edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {
      public static final boolean IS_SIM = false;

    public static final class Swerve {
      //change pigeon id too
          public static final int pigeonID = 0;

  //robot dimensions meters
  public static final double trackWidth= 0.5;
      public static finalo double wheelBase = 0.5;

  //max speed in milliseconds
  public static final double maxSpeed = 4.0;
      // module locations
  public static final Translation2d frontLeftLocation = 
      new Translation2d(wheelBase / 2.0, trackWidth / 2.0);

  public static final Translation2d frontRightLocation =
      new Translation2d(wheelBase / 2.0, - trackWidth / 2.0);

  public static final SwerveDriveKinematics swerveKinematics = 
      new SwerveDriveKinematics(
                        frontLeftLocation,
                        frontRightLocation,
                        backLeftLocation,
                        backRightLocation
                );
      //front left
      publis static ifnal class Mod0 {
        public static final SwerveModuleConstants(
                            1,      // drive NEO
                            2,      // steer NEO
                            3,      // CANcoder
                            0.0,    // offset
                            false,  // drive inverted
                            false   // steer inverted
                    );
        }
      /front right
      public static final SwerveModuleConstants constants = 
      new SwerveModuleConstants(
                            4,
                            5,
                            6,
                            0.0,
                            false,
                            false
                    );
        }
  //back left 
  public static final class Mod2 {
    public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(
                            7,
                            8,
                            9,
                            0.0,
                            false,
                            false
                    );
        }

        // back right
        // will change the can ids js put 1-12 for now
        public static final class Mod3 {
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(
                            10,
                            11,
                            12,
                            0.0,
                            false,
                            false
                    );
        }
  public static class SwerveModuleConstants {
    public final int driveMotorID;
    public final int driveMotorID;
    public final int angleMotorID;
    public final int cancoderID;
    public final double angleOffset;
    public final boolean driveMotorInverted;
    public final boolean angleMotorInverted;

    this.driveMotorID = driveMotorID;
    this.angleMotorID = angleMotorID;
    this.cancoderID = cancoderID;
    this.angleOffset = angleOffset;
    this.driveMotorInverted = driveMotorInverted;
    this.angleMotorInverted = angleMotorInverted;
            }
        }

// neo + swerve settings

        public static final double neoFreeSpeedRPM = 5676.0;
        public static final double neoFreeSpeedRPM = 5676.0;
        public static final double neoFreeSpeedRPM = 5676.0;
                wheelDiameter * Math.PI;

        // will change this later 
public static final double driveGearRatio = 6.75;
public static final double angleGearRatio = 12.8;

        public static final int driveCurrentLimit = 50;
        public static final int angleCurrentLimit = 30;
    }
public static final class AutoConstants {

        public static final double kPTranslationController = 5.0;
        public static final double kPThetaController = 5.0;
    }
    public static final class OI {

        public static final int driverControllerPort = 0;
        public static final double stickDeadband = 0.05;
    }

    private Constants() {
    }
}



          
