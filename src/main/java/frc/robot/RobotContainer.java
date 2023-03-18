package frc.robot;

import org.frc5587.lib.control.DeadbandCommandJoystick;
import org.frc5587.lib.control.DeadbandCommandXboxController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutoSetArm;
import frc.robot.commands.DualStickSwerve;
import frc.robot.commands.IntakeIn;
import frc.robot.commands.IntakeOut;
import frc.robot.commands.PIDAutoBalance;
import frc.robot.commands.SemiAuto;
import frc.robot.commands.SwerveLock;
import frc.robot.commands.AutoSetArm.GridHeight;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.*;
import frc.robot.util.CommandButtonBoard;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;

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
    private DeadbandCommandJoystick leftJoy = new DeadbandCommandJoystick(1, 3, 0.05);
    private DeadbandCommandJoystick rightJoy = new DeadbandCommandJoystick(2, 3, 0.05);
    private DeadbandCommandXboxController driveXb = new DeadbandCommandXboxController(3);
    // public CommandButtonBoard board = new CommandButtonBoard(4);

    // SUBSYSTEMS
    public final ColorSensor colorSensor = new ColorSensor(I2C.Port.kOnboard);
    private Limelight limelight = new Limelight();
    public Swerve swerve = new Swerve(limelight);
    private Arm arm = new Arm(colorSensor, swerve::getPose);
    public Intake intake = new Intake(colorSensor);
    public LEDs leds = new LEDs();

    // COMMANDS
    // private DualStickSwerve dualStickSwerve = new DualStickSwerve(
    //         swerve, () -> -Math.pow(xb.getRightY(), 3), 
    //                 () -> Math.pow(xb.getRightX(), 3), 
    //                 () -> Math.pow(xb.getLeftX(), 3), 
    //                 () -> false);

    private DualStickSwerve dualJoystickSwerve = new DualStickSwerve(
            swerve, () -> -rightJoy.getY(), () -> rightJoy.getX(), () -> leftJoy.getX(), () -> false);

    private SemiAuto semiAuto = new SemiAuto(swerve, arm, intake, leds);
    public AutoCommands auto = new AutoCommands(swerve, intake, leds, semiAuto);
    private AutoBalance autoBalance = new AutoBalance(swerve, leds);
    private PIDAutoBalance pidAutoBalance = new PIDAutoBalance(swerve);
    

    // Other
    private PowerDistribution pdh = new PowerDistribution();
    private SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // swerve.setDefaultCommand(dualStickSwerve);
        swerve.setDefaultCommand(dualJoystickSwerve);
        // intake.setDefaultCommand(new InstantCommand(intake::holdElement, intake));

        leds.setRainbow();
        configureBindings();
        
        pdh.clearStickyFaults();
        pdh.close();

        autoChooser.setDefaultOption("just spit", auto.justSpit());
        autoChooser.addOption("CloseToSub", auto.close());
        autoChooser.addOption("FarFromSub", auto.far());
        autoChooser.addOption("MiddleToClose", auto.midToClose());
        autoChooser.addOption("MiddleToFar", auto.midToFar());
        autoChooser.addOption("MidCharge", auto.midMidCharge());
        autoChooser.addOption("CloseCharge", auto.midCloseCharge());
        autoChooser.addOption("FarCharge", auto.midFarCharge());
        autoChooser.addOption("thing", auto.spitCrossLine());
        autoChooser.addOption("charge", auto.charge());
        autoChooser.addOption("NO COMMAND", auto.noAuto());
        SmartDashboard.putData("AUTO", autoChooser);
        // SmartDashboard.putNumber("DriveToPose pose", 0);
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
        // board.upButton().onTrue(new InstantCommand(arm::highSetpoint, arm));
        // board.middleButton().onTrue(new InstantCommand(arm::middleSetpoint, arm));
        // board.downButton().onTrue(new InstantCommand(arm::lowSetpoint, arm));
        // // board.upButton().onTrue(semiAuto.new ScoreInGrid(GridHeight.High)); //These are untested semiAuto commands!!!
        // // board.middleButton().onTrue(semiAuto.new ScoreInGrid(GridHeight.Middle)); //These are untested semiAuto commands!!!
        // // board.downButton().onTrue(semiAuto.new ScoreInGrid(GridHeight.Low)); //These are untested semiAuto commands!!!
        // board.stowButton().onTrue(new InstantCommand(arm::liftAwayFromGrid, arm));
        // // board.intakeButton().onTrue(new ParallelCommandGroup(new IntakeIn(intake), new AutoSetArm(arm, GridHeight.Low)));
        // // board.spitButton().onTrue(new IntakeOut(intake));
        // board.intakeButton().onTrue(new ParallelCommandGroup(new InstantCommand(intake::backward), new InstantCommand(arm::lowSetpoint))).onFalse(new InstantCommand(intake::stop));
        // board.spitButton().onTrue(new InstantCommand(intake::forward)).onFalse(new InstantCommand(intake::stop));
        // board.purpleButton().onTrue(new InstantCommand(leds::setPurple, leds));
        // board.yellowButton().onTrue(new InstantCommand(leds::setYellow, leds));
        // board.balanceButton().whileTrue(autoBalance);

        // xb.povUp().onTrue(new InstantCommand(arm::highSetpoint, arm));
        // xb.povRight().onTrue(new InstantCommand(arm::middleSetpoint, arm));
        // xb.povDown().onTrue(new InstantCommand(arm::lowSetpoint, arm));
        // board.upButton().onTrue(semiAuto.new ScoreInGrid(GridHeight.High)); //These are untested semiAuto commands!!!
        // board.middleButton().onTrue(semiAuto.new ScoreInGrid(GridHeight.Middle)); //These are untested semiAuto commands!!!
        // board.downButton().onTrue(semiAuto.new ScoreInGrid(GridHeight.Low)); //These are untested semiAuto commands!!!
        // xb.povUp().onTrue(semiAuto.new ScoreInGrid(GridHeight.High)); //These are untested semiAuto commands!!!
        // xb.povRight().onTrue(semiAuto.new ScoreInGrid(GridHeight.Middle)); //These are untested semiAuto commands!!!
        // xb.povDown().onTrue(semiAuto.new ScoreInGrid(GridHeight.Low)); //These are untested semiAuto commands!!!
        xb.y().onTrue(new AutoSetArm(arm, GridHeight.High));
        xb.povLeft().onTrue(new AutoSetArm(arm, GridHeight.Substation));
        xb.a().onTrue(new AutoSetArm(arm, GridHeight.Low));
        xb.x().onTrue(new InstantCommand(arm::lowerFromGrid, arm));
        xb.b().onTrue(new InstantCommand(arm::liftAwayFromGrid, arm));
        // board.intakeButton().onTrue(new ParallelCommandGroup(new IntakeIn(intake), new AutoSetArm(arm, GridHeight.Low)));
        // board.spitButton().onTrue(new IntakeOut(intake));
        xb.leftBumper().onTrue(new InstantCommand(intake::backward)).and(xb.rightBumper().negate()).onFalse(new InstantCommand(intake::holdElement));
        xb.rightBumper().onTrue(new InstantCommand(intake::forward)).and(xb.leftBumper().negate()).onFalse(new InstantCommand(intake::holdElement));
        xb.start().onTrue(new InstantCommand(leds::setPurple, leds));
        xb.back().onTrue(new InstantCommand(leds::setYellow, leds));
        xb.rightTrigger().whileTrue(autoBalance);
        xb.leftTrigger().onTrue(new InstantCommand(arm::stow));

        // rightJoy.button(3).onTrue(semiAuto.new GrabFromSubstation());
        // rightJoy.button(3).whileTrue(new RunCommand(() -> swerve.crawl(0.4)));
        // rightJoy.button(4).whileTrue(new RunCommand(() -> swerve.crawl(-0.4)));
        // xb.rightTrigger().whileTrue(pidAutoBalance);
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
