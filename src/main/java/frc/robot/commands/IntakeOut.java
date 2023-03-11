package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class IntakeOut extends CommandBase {
    private final Intake intake;
    private final Timer timer = new Timer();

    /**
     * Keeps running the intake inwards until it detects that it has a crate. This
     * is done by the right wheel being stalled and the left wheel spinning somewhat
     * free. This happens because of how the eleatic band pushed on the left side of
     * the crate.
     * 
     * @param intake intake subsystem
     */
    public IntakeOut(Intake intake) {
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.backward();
        timer.reset();
        timer.start();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(IntakeConstants.EJECT_CRATE_RUNTIME);
    }
}