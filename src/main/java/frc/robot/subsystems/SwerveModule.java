package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class SwerveModule {
  
    private final PWMSparkMax driveMotor;
    private final PWMSparkMax steerMotor;
    private final Encoder steerEncoder;
    private final PIDController steerPID;

    public SwerveModule(int driveChannel, int steerChannel, int encoderChannelA, int encoderChannelB) {
        this.driveMotor = new PWMSparkMax(driveChannel);
        this.steerMotor = new PWMSparkMax(steerChannel);
        
        this.steerEncoder = new Encoder(encoderChannelA, encoderChannelB);
        this.steerEncoder.setDistancePerPulse((2 * Math.PI) / 2048.0); 

        this.steerPID = new PIDController(1.0, 0.0, 0.0);
        this.steerPID.enableContinuousInput(-Math.PI, Math.PI); // Allows wheels to spin past 360 smoothly
    }

    public Rotation2d getSteerAngle() {
        return Rotation2d.fromRadians(steerEncoder.getDistance());
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // wheel never turns more than 90 degrees
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getSteerAngle());

        // set drive speed %
        driveMotor.set(optimizedState.speedMetersPerSecond / 4.5);

        // steering power using pid 
        double steerOutput = steerPID.calculate(getSteerAngle().getRadians(), optimizedState.angle.getRadians());
        steerMotor.set(steerOutput);
    }
}
