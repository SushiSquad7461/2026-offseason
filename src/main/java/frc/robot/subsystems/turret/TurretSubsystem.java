package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class TurretSubsystem extends SubsystemBase {
    private final TurretIO io;

    public enum TurretState {
        IDLE,
        TURNING,
        LOCKED
    }

    private TurretState state = TurretState.IDLE;

    public TurretSubsystem(TurretIO io) {
        this.io = io;
    }

    public Command changeState(TurretState newState){
        this.state = newState;
        switch (newState) {
            case IDLE:
                Commands.runOnce(()->{
                    io.moveToCenter();
                });
            case TURNING:
                return Commands.none();
            case LOCKED:
                return Commands.runOnce(() -> {
                    io.stop();
                });
            default:
                return Commands.none();
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/TurretAngle",io.getCurrentAngle());
    }
}
