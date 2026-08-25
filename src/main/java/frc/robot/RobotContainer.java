package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.Drivetrain;

public class RobotContainer {

    // controller
    private final CommandXboxController driver =
            new CommandXboxController(0);

    // drivetrain
    private final Drivetrain drivetrain =
            new Drivetrain();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // controller
    }

    public Command getAutonomousCommand() {
        // autonomous
        return null;
    }
}
