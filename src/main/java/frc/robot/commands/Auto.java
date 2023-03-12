package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Swerve;

public class Auto {
    private Swerve swerve;
    private final SwerveAutoBuilder autoBuilder;
    private static final Map<String, Command> eventMap = new HashMap<>(Map.ofEntries(
        Map.entry("startAway", Commands.print("Start Away from Sub")),
        Map.entry("startBySubstation", Commands.print("Start By Sub"))
    ));

    public Auto(Swerve swerve) {
        this.swerve = swerve;
        autoBuilder = new SwerveAutoBuilder(swerve::getPose, swerve::resetOdometry, AutoConstants.TRANSL_CONSTANTS, AutoConstants.ROT_CONSTANTS, swerve::setChassisSpeeds, eventMap, swerve);
    }
    
    public Command taxiAway() {
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup("StartAway", new PathConstraints(AutoConstants.MAX_SPEED_MPS, AutoConstants.MAX_ACCEL_MPS_2)));
    }    
    public Command taxiNear() {
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup("StartBySubstation", new PathConstraints(AutoConstants.MAX_SPEED_MPS, AutoConstants.MAX_ACCEL_MPS_2)));
    }
    public Command noAuto() {
        return Commands.none();
    }
}
