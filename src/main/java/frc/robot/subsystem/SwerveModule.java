package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkBase.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxSimState;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.generated.Constants;



public class SwerveModule {

    public final int moduleNumber;
    SparkMax spark = new SparkMax(1, MotorType.kBrushless);
    

    private final Rotation2d angleOffset;

    // NEO motors
    private final SparkMax angleMotor;
    private final SparkMax driveMotor;

    // encoders built into the NEOs
    private final RelativeEncoder angleEncoder;
    private final RelativeEncoder driveEncoder;

    // cancoder for the absolute angle
    private final CANcoder canCoder;

    // pid for the motors will change this later
    private final PIDController anglePID = new PIDController(0.5, 0.0, 0.0);
    private final PIDController drivePID = new PIDController(0.1, 0.0, 0.0);

    // simulation 
    private final DCMotor driveMotorModel = DCMotor.getNEO(1);
    private final DCMotor angleMotorModel = DCMotor.getNEO(1);

    private final DCMotorSim driveSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            driveMotorModel,
            0.025,
            Constants.Swerve.driveGearRatio
        ),
        driveMotorModel
    );

    private final DCMotorSim angleSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            angleMotorModel,
            0.004,
            Constants.Swerve.angleGearRatio
        ),
        angleMotorModel
    );

    private final SparkMaxSimState driveMotorSim;
    private final SparkMaxSimState angleMotorSim;


    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {

        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        // set up the CANcoder
        canCoder = new CANcoder(moduleConstants.cancoderID);

        // set up the NEOs
        angleMotor = new SparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        driveMotor = new SparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);

        // get the encoders from the NEOs
        angleEncoder = angleMotor.getEncoder();
        driveEncoder = driveMotor.getEncoder();

        // reset the drive encoder when we start
        driveEncoder.setPosition(0.0);

        // get sim states
        driveMotorSim = driveMotor.getSimState();
        angleMotorSim = angleMotor.getSimState();

        // reset the angle to the absolute CANcoder position
        resetToAbsolute();
    }


    // made helper methods for sysid so the motors can be controlled directly
    public void setDriveVoltage(double volts) {
        driveMotor.setVoltage(volts);
    }

    public void setSteerVoltage(double volts) {
        angleMotor.setVoltage(volts);
    }


    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {

        // optimize the wheel angle so it doesn't turn way more than it needs to
        desiredState.optimize(getState().angle);

        // set the angle
        double currentAngle = angleEncoder.getPosition();
        double targetAngle = desiredState.angle.getRotations();

        double angleOutput = anglePID.calculate(
            currentAngle,
            targetAngle
        );

        angleMotor.setVoltage(angleOutput);

        // set the speed
        setSpeed(desiredState, isOpenLoop);
    }


    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {

        if (isOpenLoop) {

            // open loop = just give the motor a percentage
            double percentOutput =
                desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;

            driveMotor.set(percentOutput);

        } else {

            // closed loop = use pid to control the speed
            double targetRPS = Conversions.MPSToRPS(
                desiredState.speedMetersPerSecond,
                Constants.Swerve.wheelCircumference
            );

            double currentRPS = driveEncoder.getVelocity() / 60.0;

            double driveOutput = drivePID.calculate(
                currentRPS,
                targetRPS
            );

            driveMotor.setVoltage(driveOutput);
        }
    }


    public Rotation2d getCANcoder() {
        return Rotation2d.fromRotations(
            canCoder.getAbsolutePosition().getValueAsDouble()
        );
    }


    public void resetToAbsolute() {

        double absolutePosition =
            getCANcoder().getRotations()
            - angleOffset.getRotations();

        angleEncoder.setPosition(absolutePosition);

        System.out.println(
            "Module " + moduleNumber +
            " reset to absolute: " +
            absolutePosition
        );
    }


    public Rotation2d getCANcoderWithOffset() {

        return Rotation2d.fromRotations(
            getCANcoder().getRotations()
            - angleOffset.getRotations()
        );
    }


    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }


    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }


    public double getAnglePosition() {
        return angleEncoder.getPosition();
    }


    public SwerveModuleState getState() {

        return new SwerveModuleState(
            Conversions.RPSToMPS(
                driveEncoder.getVelocity() / 60.0,
                Constants.Swerve.wheelCircumference
            ),

            Rotation2d.fromRotations(
                angleEncoder.getPosition()
            )
        );
    }


    public SwerveModulePosition getPosition() {

        return new SwerveModulePosition(
            Conversions.rotationsToMeters(
                driveEncoder.getPosition(),
                Constants.Swerve.wheelCircumference
            ),

            Rotation2d.fromRotations(
                angleEncoder.getPosition()
            )
        );
    }


    // simulation stuff so we can test without the actual robot
    public double simulationPeriodic() {

        double supplyVoltage = RobotController.getBatteryVoltage();

        driveMotorSim.setSupplyVoltage(supplyVoltage);
        angleMotorSim.setSupplyVoltage(supplyVoltage);

        driveSim.setInputVoltage(
            driveMotorSim.getAppliedOutput()
            * supplyVoltage
        );

        angleSim.setInputVoltage(
            angleMotorSim.getAppliedOutput()
            * supplyVoltage
        );

        // update the simulated drive motor
        driveSim.update(0.02);

        // update the simulated angle motor
        angleSim.update(0.02);

        // put the simulated values back into the NEOs
        driveMotorSim.setRawRotorPosition(
            driveSim.getAngularPositionRotations()
            * Constants.Swerve.driveGearRatio
        );

        driveMotorSim.setRotorVelocity(
            Units.radiansToRotations(
                driveSim.getAngularVelocityRadPerSec()
                * Constants.Swerve.driveGearRatio
            )
        );

        angleMotorSim.setRawRotorPosition(
            angleSim.getAngularPositionRotations()
            * Constants.Swerve.angleGearRatio
        );

        angleMotorSim.setRotorVelocity(
            Units.radiansToRotations(
                angleSim.getAngularVelocityRadPerSec()
            )
            * Constants.Swerve.angleGearRatio
        );

        return Math.abs(driveSim.getCurrentDrawAmps())
            + Math.abs(angleSim.getCurrentDrawAmps());
    }
}
