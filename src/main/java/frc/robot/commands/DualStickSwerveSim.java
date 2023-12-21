package frc.robot.commands;

import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.SwerveSim;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DualStickSwerveSim extends CommandBase {    
    private SwerveSim swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier fieldRelativeSup;

    public DualStickSwerveSim(SwerveSim swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier fieldRelativeSup) {
        this.swerve = swerve;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.fieldRelativeSup = fieldRelativeSup;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        Translation2d translation = new Translation2d(translationSup.getAsDouble(), strafeSup.getAsDouble()).times(SwerveConstants.MAX_SPEED);
        double rotation = rotationSup.getAsDouble() * SwerveConstants.MAX_ANGULAR_VELOCITY;
        
        swerve.drive(translation, rotation, 
            fieldRelativeSup.getAsBoolean(), 
            true
        );
    }
}