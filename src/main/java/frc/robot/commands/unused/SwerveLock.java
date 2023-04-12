package frc.robot.commands.unused;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class SwerveLock extends CommandBase {
    private Swerve swerve;

    public SwerveLock(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        swerve.setModuleStates(new SwerveModuleState[] {
            new SwerveModuleState(0, new Rotation2d(45)),
            new SwerveModuleState(0, new Rotation2d(-45)),
            new SwerveModuleState(0, new Rotation2d(-45)),
            new SwerveModuleState(0, new Rotation2d(45)),
        });
    }
}
