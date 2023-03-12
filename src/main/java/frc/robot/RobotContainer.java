package frc.robot;

import org.frc5587.lib.control.DeadbandCommandJoystick;
import org.frc5587.lib.control.DeadbandCommandXboxController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Auto;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutoSetArm;
import frc.robot.commands.DualStickSwerve;
import frc.robot.commands.IntakeIn;
import frc.robot.commands.IntakeOut;
import frc.robot.commands.JoystickSwerve;
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
    private DeadbandCommandXboxController xb = new DeadbandCommandXboxController(0, .05);
    private DeadbandCommandJoystick leftJoy = new DeadbandCommandJoystick(1, 1.5, 0.02);
    private DeadbandCommandJoystick rightJoy = new DeadbandCommandJoystick(2, 1.5, 0.02);
    public CommandButtonBoard board = new CommandButtonBoard(3);

    // SUBSYSTEMS
    private Limelight limelight = new Limelight();
    public Swerve swerve = new Swerve(limelight);
    private Arm arm = new Arm();
    private Intake intake = new Intake();
    public LEDs leds = new LEDs();

    // COMMANDS
    private DualStickSwerve dualStickSwerve = new DualStickSwerve(
            swerve, () -> Math.pow(xb.getRightY(), 3), () -> Math.pow(xb.getRightX(), 3), () -> Math.pow(xb.getLeftX(), 3), () -> true); // last param is robotcentric, should be true

    private DualStickSwerve dualJoystickSwerve = new DualStickSwerve(
            swerve, () -> Math.pow(rightJoy.getY(), 3), () -> Math.pow(rightJoy.getX(), 3), () -> Math.pow(leftJoy.getX(), 3), () -> true); // last param is robotcentric, should be true

    private JoystickSwerve joystickSwerve = new JoystickSwerve(
        swerve, leftJoy::getY, leftJoy::getX, leftJoy::getX, leftJoy::getTwist, () -> true);

    private SemiAuto semiAuto = new SemiAuto(swerve, arm, intake);
    private AutoBalance autoBalance = new AutoBalance(swerve, leds);
    private Auto auto = new Auto(swerve, intake);
    private SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // swerve.setDefaultCommand(dualStickSwerve);
        // swerve.setDefaultCommand(joystickSwerve);
        swerve.setDefaultCommand(dualJoystickSwerve);
        leds.setRB();
        configureBindings();
        PowerDistribution pdh = new PowerDistribution();
        pdh.clearStickyFaults();
        pdh.close();
        autoChooser.setDefaultOption("just spit", auto.justSpit());
        autoChooser.addOption("spittaxi near", auto.spitThenTaxiNear());
        autoChooser.addOption("spittaxi far", auto.spitThenTaxiAway());
        autoChooser.addOption("taxi near sub", auto.taxiNear());
        autoChooser.addOption("taxi far from sub", auto.taxiAway());
        autoChooser.addOption("NO COMMAND", auto.noAuto());
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

        // armLimitSwitch.onTrue(new InstantCommand(() -> arm.resetEncoders()));

        // xb.povLeft().onTrue(usingRedPoses ? semiAuto.new DriveToGrid(5) : semiAuto.new DriveToGrid(0));
        // xb.povUp().onTrue(usingRedPoses ? semiAuto.new DriveToGrid(4) : semiAuto.new DriveToGrid(1));
        // xb.povRight().onTrue(usingRedPoses ? semiAuto.new DriveToGrid(3) : semiAuto.new DriveToGrid(2));

        // board.leftButton().onTrue(semiAuto.new DriveWithinGrid(0));
        // board.centerButton().onTrue(semiAuto.new DriveWithinGrid(1));
        // board.rightButton().onTrue(semiAuto.new DriveWithinGrid(2));
        board.upButton().onTrue(new InstantCommand(arm::highSetpoint, arm));
        board.middleButton().onTrue(new InstantCommand(arm::middleSetpoint, arm));
        board.downButton().onTrue(new InstantCommand(arm::lowSetpoint, arm));
        // board.upButton().onTrue(semiAuto.new ScoreInGrid(GridHeight.High)); //These are untested semiAuto commands!!!
        // board.middleButton().onTrue(semiAuto.new ScoreInGrid(GridHeight.Middle)); //These are untested semiAuto commands!!!
        // board.downButton().onTrue(semiAuto.new ScoreInGrid(GridHeight.Low)); //These are untested semiAuto commands!!!
        board.stowButton().onTrue(new InstantCommand(arm::liftAwayFromGrid, arm));
        // board.intakeButton().onTrue(new ParallelCommandGroup(new IntakeIn(intake), new AutoSetArm(arm, GridHeight.Low)));
        // board.spitButton().onTrue(new IntakeOut(intake));
        board.intakeButton().onTrue(new ParallelCommandGroup(new InstantCommand(intake::backward), new InstantCommand(arm::lowSetpoint))).onFalse(new InstantCommand(intake::stop));
        board.spitButton().onTrue(new InstantCommand(intake::forward)).onFalse(new InstantCommand(intake::stop));
        board.purpleButton().onTrue(new InstantCommand(leds::setPurple, leds));
        board.yellowButton().onTrue(new InstantCommand(leds::setYellow, leds));
        board.balanceButton().whileTrue(autoBalance);




        xb.povUp().onTrue(new InstantCommand(arm::highSetpoint, arm));
        xb.povRight().onTrue(new InstantCommand(arm::middleSetpoint, arm));
        xb.povDown().onTrue(new InstantCommand(arm::lowSetpoint, arm));
        // board.upButton().onTrue(semiAuto.new ScoreInGrid(GridHeight.High)); //These are untested semiAuto commands!!!
        // board.middleButton().onTrue(semiAuto.new ScoreInGrid(GridHeight.Middle)); //These are untested semiAuto commands!!!
        // board.downButton().onTrue(semiAuto.new ScoreInGrid(GridHeight.Low)); //These are untested semiAuto commands!!!
        xb.y().onTrue(new InstantCommand(arm::liftAwayFromGrid, arm));
        xb.a().onTrue(new InstantCommand(arm::lowerFromGrid, arm));
        xb.x().onTrue(semiAuto.new ConeFlipper());
        // board.intakeButton().onTrue(new ParallelCommandGroup(new IntakeIn(intake), new AutoSetArm(arm, GridHeight.Low)));
        // board.spitButton().onTrue(new IntakeOut(intake));
        xb.rightBumper().onTrue(new ParallelCommandGroup(new InstantCommand(intake::backward))).onFalse(new InstantCommand(intake::stop));
        xb.leftBumper().onTrue(new InstantCommand(intake::forward)).onFalse(new InstantCommand(intake::stop));
        xb.start().onTrue(new InstantCommand(leds::setPurple, leds));
        xb.back().onTrue(new InstantCommand(leds::setYellow, leds));
        xb.rightTrigger().whileTrue(autoBalance);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
