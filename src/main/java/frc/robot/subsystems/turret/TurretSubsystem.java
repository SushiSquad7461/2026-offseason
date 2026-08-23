package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.Constants;
public class TurretSubsystem extends SubsystemBase {
    private final TurretIO io;

    public enum TurretState {
        IDLE,
        TURNING,
        LOCKED
    }

    private TurretState state = TurretState.IDLE;

    public LoggedMechanism2d mech2d = new LoggedMechanism2d(?,?);


    public TurretSubsystem(TurretIO io) {
        this.io = io;
    }

    public void startTurning(){
        state = TurretState.MOVING;
    }

    public void stopTurning(){
        state = TurretState.IDLE;
    }

    public Command changeState(TurretState newState){
        this.state = newState;
        switch (newState) {
            case IDLE:
                return Commands.parallel(
                    Commands.runOnce(()->{
                        io.moveToCenter();
                    }));
            case TURNING:
                return Commands.parallel(
                    Commands.runOnce(()->{
                        io.startTurning();
                    })
                    Commands.runOnce(()->{
                        io.turnDegrees(degrees);
                    })
                );
            case LOCKED:
                return Commands.parallel(
                    Commands.runOnce(()->{
                        stopTurning();
                    }));
            default:
                return Commands.none();
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/TurretAngle",io.getCurrentAngle());
    }
}
