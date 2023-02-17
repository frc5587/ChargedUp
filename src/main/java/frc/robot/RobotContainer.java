package frc.robot;

import org.frc5587.lib.control.DeadbandCommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DualStickSwerve;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // INPUTS
    private DeadbandCommandXboxController xb = new DeadbandCommandXboxController(0);

    // SUBSYSTEMS
    private Limelight limelight = new Limelight();
    private Swerve swerve = new Swerve(limelight);

    // COMMANDS
    private DualStickSwerve dualStickSwerve = new DualStickSwerve(
            swerve, () -> xb.getRightY(), () -> xb.getRightX(), () -> xb.getLeftX(), () -> true); // last param is robotcentric, should be true

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        swerve.setDefaultCommand(dualStickSwerve);
        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {}

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }
}
