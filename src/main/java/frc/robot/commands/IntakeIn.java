package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeIn extends CommandBase {
    private final Intake intake;

    /**
     * Keeps running the intake inwards until it detects that it has an element.
     * 
     * @param intake intake subsystem
     */
    public IntakeIn(Intake intake) {
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.forward();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }

    @Override
    public boolean isFinished() {
        return intake.hasElement();
    }
}