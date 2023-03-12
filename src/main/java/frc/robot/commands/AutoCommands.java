package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Swerve;

public class AutoCommands {
    private Intake intake;
    public final SwerveAutoBuilder autoBuilder;
    private final Map<String, Command> eventMap;

    public AutoCommands(Swerve swerve, Intake intake, LEDs leds) {
        this.intake = intake;

        this.eventMap = new HashMap<>(Map.ofEntries(
                Map.entry("stopIntake", new InstantCommand(intake::stop)),
                Map.entry("autoBalance", new AutoBalance(swerve, leds))));

        this.autoBuilder = new SwerveAutoBuilder(
                swerve::getPose, 
                swerve::resetOdometry, 
                AutoConstants.TRANSL_CONSTANTS, 
                AutoConstants.ROT_CONSTANTS, 
                swerve::setChassisSpeeds, 
                eventMap, 
                true, 
                swerve);
    }

    /**
     * takes a path command from autoBuilder and adds an outtake to the beginning of it
     * @param pathCommand the auto to play after the game element is spit out
     */
    private Command spitTravelCommand(Command pathCommand) {
        return new SequentialCommandGroup(new InstantCommand(intake::autoThrottle), new WaitCommand(1.5), new InstantCommand(intake::stop), pathCommand);
    }

    /**
     * takes a path command from autoBuilder and adds an outtake to the beginning of it
     * @param pathName the name of the autoPath file to build and play after the game element is spit out
     */
    private Command spitTravelCommand(String pathName) {
        return spitTravelCommand(autoBuilder.fullAuto(PathPlanner.loadPathGroup(pathName, AutoConstants.PATH_CONSTRAINTS)));
    }

    /**
     * empty auto command
     */
    public Command noAuto() {
        return Commands.none();
    }

    /**
     * Start far away from the substations
     */
    public Command far() {
        return spitTravelCommand("StartFar");
    }

    /**
     * Start close to the substations
     */
    public Command close() {
        return spitTravelCommand("StartClose");
    }

    /**
     * Start in the very middle, and move far from the substations
     */
    public Command midToFar() {
        return spitTravelCommand("StartMiddleGoFar");
    }

    /**
     * Start in the very middle, and move closer to the substations
     */
    public Command midToClose() {
        return spitTravelCommand("StartMiddleGoClose");
    }

    /**
     * Start in the very middle, and charge in the middle of the charging station
     * (ideal if another robot is not charging during auto)
     */
    public Command midMidCharge() {
        return spitTravelCommand("StartMiddleChargeMiddle");
    }

    /**
     * Start in the very middle, and charge far from the substation 
     * (ideal if another robot is charging close during auto)
     */
    public Command midFarCharge() {
        return spitTravelCommand("StartMiddleChargeFar");
    }

    /**
     * Start in the very middle, and charge close to the substation 
     * (ideal if another robot is charging far during auto)
     */
    public Command midCloseCharge() {
        return spitTravelCommand("StartMiddleChargeClose");
    }

    /**
     * Only outtake during auto, do not move.
     */
    public Command justSpit() {
        return new InstantCommand(intake::autoThrottle, intake);
    }
}