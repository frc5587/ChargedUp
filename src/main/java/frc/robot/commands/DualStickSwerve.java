package frc.robot.commands;

import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class DualStickSwerve extends CommandBase {    
    private Swerve swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier fieldRelativeSup;

    double rotation;

    private final PIDController headingController = 
        new PIDController(SwerveConstants.ANGLE_FPID.kP, SwerveConstants.ANGLE_FPID.kI, SwerveConstants.ANGLE_FPID.kD); // TODO Once this is working, move into constants and use for all headings

    public DualStickSwerve(Swerve swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier fieldRelativeSup) {
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
            rotation = rotationSup.getAsDouble() * SwerveConstants.MAX_ANGULAR_VELOCITY;
        
        swerve.drive(translation, rotation, 
            fieldRelativeSup.getAsBoolean(), 
            true
        );
    }
}