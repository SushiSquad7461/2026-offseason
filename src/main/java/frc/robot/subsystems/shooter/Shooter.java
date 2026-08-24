package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * The unified Shooter and Hood subsystem.
 */
public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    // Flywheel Control
    private final PIDController flywheelPid = new PIDController(0.001, 0, 0); 
    private final SimpleMotorFeedforward flywheelFeedforward = new SimpleMotorFeedforward(0.1, 0.12);
    private double targetRPM = 0.0;
    
    // Hood Control
    private final ProfiledPIDController hoodPid = new ProfiledPIDController(
        0.05, 0.0, 0.0, 
        new TrapezoidProfile.Constraints(90.0, 90.0)
    );
    private double targetAngleDegrees = 0.0;

    public enum ShooterState {
        IDLE,      // Mechanisms are stopped and resting
        SHOOTING   // Mechanisms are actively adjusting to target RPM and Hood angle
    }
    
    private ShooterState state = ShooterState.IDLE;

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        // Execute logic based on the State Enum
        switch (state) {
            case SHOOTING:
                // Flywheel Logic
                double ffVolts = flywheelFeedforward.calculate(targetRPM);
                double fPidVolts = flywheelPid.calculate(inputs.leftFlywheelVelocityRPM, targetRPM);
                io.setFlywheelVoltage(ffVolts + fPidVolts);
                
                // Hood Logic
                double hPidVolts = hoodPid.calculate(inputs.hoodAngleDegrees, targetAngleDegrees);
                io.setHoodVoltage(hPidVolts);
                break;
                
            case IDLE:
            default:
                io.setFlywheelVoltage(0.0);
                io.setHoodVoltage(0.0);
                break;
        }

        // Log our targets and current Enum state string
        Logger.recordOutput("Shooter/CurrentState", state.toString());
        Logger.recordOutput("Shooter/TargetRPM", targetRPM);
        Logger.recordOutput("Shooter/TargetAngleDeg", targetAngleDegrees);
    }

    /** Maps: (Distance Meters) -> (Flywheel RPM) */
    private static final InterpolatingDoubleTreeMap distanceToRPM = new InterpolatingDoubleTreeMap();
    /** Maps: (Distance Meters) -> (Hood Angle Degrees) */
    private static final InterpolatingDoubleTreeMap distanceToHoodAngle = new InterpolatingDoubleTreeMap();

    static {
        // [TODO] You physically test these on the real robot and populate them!
        distanceToRPM.put(2.0, 3000.0);
        distanceToHoodAngle.put(2.0, 20.0);

        distanceToRPM.put(4.5, 4500.0);
        distanceToHoodAngle.put(4.5, 40.0);

        distanceToRPM.put(7.0, 5500.0);
        distanceToHoodAngle.put(7.0, 50.0);
    }

    // TODO: Update this to match the actual X,Y coordinate of your goal on the field (in meters)
    private static final Translation2d GOAL_POSITION = new Translation2d(0.0, 0.0);

    /** 
     * Aim the turret using the Robot's global Field Pose.
     * The Vision system (or Odometry) determines where the robot is on the field, 
     * and we calculate the distance to the goal from that coordinate.
     */
    public void aimAtFieldPose(Pose2d robotPose) {
        // Calculate the straight-line distance from the center of the robot to the goal
        double distanceMeters = robotPose.getTranslation().getDistance(GOAL_POSITION);
        
        // Pass that calculated distance into our curvature map!
        setTargetRPM(distanceToRPM.get(distanceMeters));
        setTargetAngle(distanceToHoodAngle.get(distanceMeters));
    }

    /** Set the desired flywheel speed */
    public void setTargetRPM(double rpm) {
        this.targetRPM = rpm;
        this.state = ShooterState.SHOOTING;
    }
    
    /** Set the desired hood angle from 8.0 to 40.0 degrees */
    public void setTargetAngle(double degrees) {
        // Constrain to physical hard stops so the gear doesn't break
        this.targetAngleDegrees = edu.wpi.first.math.MathUtil.clamp(degrees, 8.0, 40.0);
        this.state = ShooterState.SHOOTING;
    }

    /** Stop all mechanisms by switching the state to IDLE */
    public void stop() {
        this.state = ShooterState.IDLE;
        this.targetRPM = 0;
        io.stop();
    }
}
