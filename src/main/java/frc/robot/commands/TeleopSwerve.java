package frc.robot.commands;

import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {    
    private Swerve swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;

    public TeleopSwerve(Swerve swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.swerve = swerve;
        addRequirements(swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void execute() {
        swerve.drive(
            new Translation2d(translationSup.getAsDouble(), strafeSup.getAsDouble()).times(SwerveConstants.MAX_SPEED), 
            rotationSup.getAsDouble() * SwerveConstants.MAX_ANGULAR_VELOCITY, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}