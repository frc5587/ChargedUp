package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

public class JoystickSwerve extends CommandBase {
    private Swerve swerve;
    private DoubleSupplier translationSup, strafeSup, rotationSup, activeRotateSup;
    private BooleanSupplier robotCentricSup;

    public JoystickSwerve(Swerve swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, DoubleSupplier activeRotateSup, BooleanSupplier robotCentricSup) {
        this.swerve = swerve;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.activeRotateSup = activeRotateSup;
        this.robotCentricSup = robotCentricSup;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        // swerve.drive(new Translation2d(translationSup.getAsDouble(), activeRotateSup.getAsDouble() ? 0 : strafeSup.getAsDouble()).times(SwerveConstants.MAX_SPEED), 
        //     activeRotateSup.getAsBoolean() ? rotationSup.getAsDouble() * SwerveConstants.MAX_ANGULAR_VELOCITY : 0, !robotCentricSup.getAsBoolean(), true); 
    }
}
