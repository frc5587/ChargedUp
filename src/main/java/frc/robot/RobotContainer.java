package frc.robot;

import org.frc5587.lib.control.DeadbandCommandXboxController;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

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
    private DeadbandCommandXboxController driveXb = new DeadbandCommandXboxController(0, .02);
    private DeadbandCommandXboxController xb = new DeadbandCommandXboxController(3);

    // SUBSYSTEMS
    public final ColorSensor colorSensor = new ColorSensor(I2C.Port.kMXP);
    private Limelight limelight = new Limelight();
    public Swerve swerve = new Swerve(limelight);
    private Arm arm = new Arm(colorSensor, swerve::getPose);
    private Wrist wrist = new Wrist(arm);
    public Intake intake = new Intake(colorSensor);
    public LEDs leds = new LEDs();

    // COMMANDS
    private DualStickSwerve dualStickSwerve = new DualStickSwerve(
            swerve, () -> Math.pow(driveXb.getRightY(), 3), // -Math.pow(driveXb.getRightY(), 3), 
                    () -> Math.pow(driveXb.getRightX(), 3), 
                    () -> Math.pow(driveXb.getLeftX(), 3), 
                    () -> false);

    public AutoCommands auto = new AutoCommands(intake, arm, swerve, leds);
    private AutoBalance autoBalance = new AutoBalance(swerve, leds);    

    // Other
    private PowerDistribution pdh = new PowerDistribution();

    public Command currentDrive = dualStickSwerve;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        leds.setRainbow();
        pdh.clearStickyFaults();
        pdh.close();
        
        swerve.setDefaultCommand(currentDrive);
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
    private void configureBindings() {
        Trigger wristUp = new Trigger(() -> xb.getRightY() > 0.1);
        Trigger wristDown = new Trigger(() -> xb.getRightY() < -0.1);
        Trigger resetWristCommand = new Trigger(() -> xb.getLeftY() < -0.1);

        xb.y().onTrue(new InstantCommand(arm::middleSetpoint));
        xb.povLeft().onTrue(new InstantCommand(arm::substationSetpoint));
        xb.povRight().onTrue(new InstantCommand(arm::highSetpoint));
        xb.a().onTrue(new InstantCommand(arm::lowSetpoint));
        xb.x().onTrue(new InstantCommand(arm::lowerFromGrid, arm));
        xb.b().onTrue(new InstantCommand(arm::liftAwayFromGrid, arm));
        xb.leftBumper().onTrue(new InstantCommand(intake::backward)).and(xb.rightBumper().negate()).onFalse(new InstantCommand(intake::stop));//holdElement));
        xb.rightBumper().onTrue(new InstantCommand(intake::forward)).and(xb.leftBumper().negate()).onFalse(new InstantCommand(intake::stop));//holdElement));
        xb.start().onTrue(new InstantCommand(leds::setPurple, leds));
        xb.back().onTrue(new InstantCommand(leds::setYellow, leds));
        driveXb.rightTrigger().whileTrue(autoBalance);
        xb.leftTrigger().onTrue(new InstantCommand(arm::stow));
        xb.povUp().onTrue(new InstantCommand(() -> intake.autoThrottle())).onFalse(new InstantCommand(intake::stop));
        wristUp.whileTrue(new RunCommand(() -> {wrist.setManualOverride(true); wrist.setGoal(wrist.getController().getGoal().position - Units.degreesToRadians(0.45));}));
        wristDown.whileTrue(new RunCommand(() -> {wrist.setManualOverride(true); wrist.setGoal(wrist.getController().getGoal().position + Units.degreesToRadians(0.45));}));
        resetWristCommand.onTrue(new InstantCommand(() -> {wrist.setManualOverride(false); wrist.setFollowArm(true); wrist.setRaised(false);}));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return auto.getSelectedCommand();
    }
}
