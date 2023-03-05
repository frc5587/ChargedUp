package frc.robot;

import org.frc5587.lib.control.DeadbandCommandXboxController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.DualStickSwerve;
import frc.robot.commands.SemiAuto;
import frc.robot.commands.AutoSetArm.GridHeight;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.*;
import frc.robot.util.CommandButtonBoard;

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
    private DeadbandCommandXboxController xb = new DeadbandCommandXboxController(0, .3);
    public CommandButtonBoard board = new CommandButtonBoard(1);

    // SUBSYSTEMS
    private Limelight limelight = new Limelight();
    private Swerve swerve = new Swerve(limelight);
    private Arm arm = new Arm();
    private Intake intake = new Intake();
    public LEDs leds = new LEDs();

    // COMMANDS
    private DualStickSwerve dualStickSwerve = new DualStickSwerve(
            swerve, () -> xb.getRightY(), () -> xb.getRightX(), () -> xb.getLeftX(), () -> true); // last param is robotcentric, should be true
    private SemiAuto semiAuto = new SemiAuto(swerve, arm, intake);
    private AutoBalance autoBalance = new AutoBalance(swerve, leds);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        swerve.setDefaultCommand(dualStickSwerve);
        leds.setChase();
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
        boolean usingRedPoses = DriverStation.getAlliance().equals(Alliance.Red);
        Trigger armLimitSwitch = new Trigger(arm::getLimitSwitchValue);

        armLimitSwitch.onTrue(new InstantCommand(() -> arm.resetEncoders()));

        xb.povLeft().onTrue(usingRedPoses ? semiAuto.new DriveToGrid(5) : semiAuto.new DriveToGrid(0));
        xb.povUp().onTrue(usingRedPoses ? semiAuto.new DriveToGrid(4) : semiAuto.new DriveToGrid(1));
        xb.povRight().onTrue(usingRedPoses ? semiAuto.new DriveToGrid(3) : semiAuto.new DriveToGrid(2));

        board.leftButton().onTrue(semiAuto.new DriveWithinGrid(0));
        board.centerButton().onTrue(semiAuto.new DriveWithinGrid(1));
        board.rightButton().onTrue(semiAuto.new DriveWithinGrid(2));
        // board.upButton().onTrue(new InstantCommand(arm::highSetpoint, arm));
        // board.middleButton().onTrue(new InstantCommand(arm::middleSetpoint, arm));
        // board.downButton().onTrue(new InstantCommand(arm::lowSetpoint, arm));
        board.upButton().onTrue(semiAuto.new ScoreInGrid(GridHeight.High)); //These are untested semiAuto commands!!!
        board.middleButton().onTrue(semiAuto.new ScoreInGrid(GridHeight.Middle)); //These are untested semiAuto commands!!!
        board.downButton().onTrue(semiAuto.new ScoreInGrid(GridHeight.Low)); //These are untested semiAuto commands!!!
        board.stowButton().onTrue(new InstantCommand(arm::stow, arm));
        // board.extendButton().onTrue(new InstantCommand(intake::extend, intake)); // TODO
        // board.retractButton().onTrue(new InstantCommand(intake::retract, intake)); // TODO
        board.purpleButton().onTrue(new InstantCommand(leds::setPurple, leds));
        board.yellowButton().onTrue(new InstantCommand(leds::setYellow, leds));
        board.balanceButton().onTrue(autoBalance);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }
}
