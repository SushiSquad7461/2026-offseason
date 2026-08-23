package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.MathUtil;

public class TurretIOSim implements TurretIO{
    
    @AutoLog
    public static class ShooterData{
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }
    
    public final ShooterData data = new ShooterData();

    private double simulatedDegrees = 0; 
    private double targetDegrees = 0;

    @Override
    public void turnDegrees(double degrees){
        targetDegrees = MathUtil.clamp(simulatedDegrees + degrees, 0, 360);
        // implement?
        //data.appliedVolts = ;
    }

    @Override
    public double getCurrentAngle(){
        return simulatedDegrees;
    }

    @Override
    public void moveToCenter(){
        targetDegrees = 0;
    }

}
