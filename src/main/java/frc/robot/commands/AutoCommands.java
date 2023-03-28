package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.AutoSetArm.GridHeight;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Swerve;

public class AutoCommands {
    // subsystems
    private final Intake intake;
    private final Arm arm;
    private final Swerve swerve;
    private final LEDs leds;

    // Auto Paths
    private PathPlannerTrajectory p_pos1SpitAndCross = PathPlanner.loadPath("pos1SpitAndCross", AutoConstants.PATH_CONSTRAINTS);
    private PathPlannerTrajectory p_pos3SpitAndCross = PathPlanner.loadPath("pos3SpitAndCross", AutoConstants.PATH_CONSTRAINTS);
    private PathPlannerTrajectory p_pos4LinkAndCharge = PathPlanner.loadPath("pos4LinkAndCharge", AutoConstants.PATH_CONSTRAINTS); // TODO this shit is not gonna work

    private NetworkTableEntry chooseAutoPath = SmartDashboard.getEntry("Choose Auto Path");
    private Notifier chooseAutoNotifier = new Notifier(this::blinkIfNoPath);

    /** Starting with 1 pre-loaded game piece in position 1, spit into hybrid, then leave the community. */
    public final Command c_pos1SpitAndCross;

    /** Starting with 1 pre-loaded game piece in position 2, spit into hybrid.*/
    public final Command c_pos2Spit;

    /** Starting with 1 pre-loaded game piece in position 3, spit into hybrid, then leave the community. */
    public final Command c_pos3SpitAndCross;

    /**
     * Starting with 1 pre-loaded cube in position 4, shoot into hybrid, 
     */
    public final Command c_pos4LinkAndCharge;


    public final SwerveAutoBuilder autoBuilder;
    private HashMap<String, Command> eventMap;
    
    SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    public AutoCommands(Intake intake, Arm arm, Swerve swerve, LEDs leds, SemiAuto semiAuto) {
        this.intake = intake;
        this.arm = arm;
        this.swerve = swerve;
        this.leds = leds;

        // this.eventMap = new HashMap<>(Map.ofEntries(
        //         Map.entry("stopIntake", new InstantCommand(intake::stop)),
        //         Map.entry("autoBalance", new AutoBalance(swerve, leds)),
        //         Map.entry("intakeIn", new IntakeIn(intake)),
        //         Map.entry("autoScoreMid", semiAuto.new ScoreInGrid(GridHeight.Middle)),
        //         Map.entry("autoScoreHigh", semiAuto.new ScoreInGrid(GridHeight.High))));

        eventMap.put("intake", new IntakeIn(intake));
        eventMap.put("spit", new IntakeOut(intake));
        eventMap.put("shootCube", new InstantCommand(intake::shootCube));
        eventMap.put("stopIntake", new InstantCommand(intake::stop));
        eventMap.put("stowSetpoint", new InstantCommand(arm::stow));
        eventMap.put("intakeSetpoint", new InstantCommand(arm::intakeSetpoint));
        eventMap.put("lowSetpoint", new InstantCommand(arm::lowSetpoint));
        eventMap.put("midSetpoint", new InstantCommand(arm::middleSetpoint));
        eventMap.put("autoEngage", new AutoBalance(swerve, leds));

        this.autoBuilder = new SwerveAutoBuilder(
                swerve::getPose, 
                swerve::resetOdometry,
                AutoConstants.TRANSL_CONSTANTS, 
                AutoConstants.THETA_CONSTANTS, 
                swerve::setChassisSpeeds, 
                eventMap, 
                true, 
                swerve);

        // this.c_pos1SpitAndCross = new FollowPathWithEvents(AutoConstants.PPSwerveController(p_pos1SpitAndCross, swerve), p_pos1SpitAndCross.getMarkers(), eventMap);
        this.c_pos1SpitAndCross = autoBuilder.fullAuto(p_pos1SpitAndCross);
        this.c_pos2Spit = new ParallelCommandGroup(
                new InstantCommand(() -> swerve.resetOdometry(DriverStation.getAlliance() == Alliance.Blue
                        ? (new Pose2d(1.82, 2.73, new Rotation2d(Units.degreesToRadians(180))))
                        : new Pose2d(14.71, 2.73, new Rotation2d(Units.degreesToRadians(0))))),
                new IntakeOut(intake));
        this.c_pos3SpitAndCross = autoBuilder.fullAuto(p_pos3SpitAndCross);
        this.c_pos4LinkAndCharge = autoBuilder.fullAuto(p_pos4LinkAndCharge);
        
        autoChooser.addOption("1st Position with Spit & Cross", c_pos1SpitAndCross);
        autoChooser.addOption("2nd Position with Spit", c_pos2Spit);
        autoChooser.addOption("3rd Position with Spit & Cross", c_pos3SpitAndCross);
        SmartDashboard.putData(autoChooser);
            
        chooseAutoNotifier.startPeriodic(0.1);
    }

    // /**
    //  * takes a path command from autoBuilder and adds an outtake to the beginning of it
    //  * @param pathCommand the auto to play after the game element is spit out
    //  */
    // private Command spitTravelCommand(Command pathCommand) {
    //     return new SequentialCommandGroup(new InstantCommand(intake::autoThrottle), new WaitCommand(1.5), new InstantCommand(intake::stop), pathCommand);
    // }

    // /**
    //  * takes a path command from autoBuilder and adds an outtake to the beginning of it
    //  * @param pathName the name of the autoPath file to build and play after the game element is spit out
    //  */
    // private Command spitTravelCommand(String pathName) {
    //     return spitTravelCommand(autoBuilder.fullAuto(PathPlanner.loadPathGroup(pathName, AutoConstants.PATH_CONSTRAINTS)));
    // }

    // /**
    //  * empty auto command
    //  */
    // public Command noAuto() {
    //     return Commands.none();
    // }

    // public Command spitCrossLine() {
    //     return 
    //     new SequentialCommandGroup(
    //             new InstantCommand(intake::autoThrottle), 
    //             new WaitCommand(1.5), 
    //             new InstantCommand(intake::stop),
    //             new PrintCommand("SPCL"),
    //             new PrintCommand("SPCL"),
    //             new PrintCommand("SPCL"),
    //             new PrintCommand("SPCL"),
    //             new PrintCommand("SPCL"),
    //             new PrintCommand("SPCL"),
    //             new ParallelDeadlineGroup(new WaitCommand(13), new RunCommand(() -> swerve.crawl(-0.25), swerve)));
    // }

    // public Command charge() {
    //     return new SequentialCommandGroup(
    //         // new InstantCommand(intake::autoThrottle),
    //         new WaitCommand(1.5),
    //         new InstantCommand(intake::stop),
    //         new ParallelDeadlineGroup(new WaitCommand(6), new RunCommand(() -> swerve.crawl(-.33), swerve)),
    //         new AutoBalance(swerve, leds));
    // }

    // /**
    //  * Start far away from the substations
    //  */
    // public Command far() {
    //     return spitTravelCommand("StartFar");
    // }

    // /**
    //  * Start close to the substations
    //  */
    // public Command close() {
    //     return spitTravelCommand("StartClose");
    // }

    // /**
    //  * Start in the very middle, and move far from the substations
    //  */
    // public Command midToFar() {
    //     return spitTravelCommand("StartMiddleGoFar");
    // }

    // /**
    //  * Start in the very middle, and move closer to the substations
    //  */
    // public Command midToClose() {
    //     return spitTravelCommand("StartMiddleGoClose");
    // }

    // /**
    //  * Start in the very middle, and charge in the middle of the charging station
    //  * (ideal if another robot is not charging during auto)
    //  */
    // public Command midMidCharge() {
    //     return spitTravelCommand("StartMiddleChargeMiddle").andThen(new AutoBalance(swerve, leds));
    // }

    // /**
    //  * Start in the very middle, and charge far from the substation 
    //  * (ideal if another robot is charging close during auto)
    //  */
    // public Command midFarCharge() {
    //     return spitTravelCommand("StartMiddleChargeFar");
    // }

    // /**
    //  * Start in the very middle, and charge close to the substation 
    //  * (ideal if another robot is charging far during auto)
    //  */
    // public Command midCloseCharge() {
    //     return spitTravelCommand("StartMiddleChargeClose");
    // }

    // /**
    //  * Only outtake during auto, do not move.
    //  */
    // public Command justSpit() {
    //     return new InstantCommand(intake::autoThrottle, intake);
    // }

    public Command getSelectedCommand() {
        return autoChooser.getSelected();
    }

    private void blinkIfNoPath() {
        if(getSelectedCommand() == null) {
            chooseAutoPath.setBoolean(!chooseAutoPath.getBoolean(false));
        } else {
            chooseAutoPath.setBoolean(true);
        }
    }
}
