package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class AutoSetArm extends CommandBase {
    private final Arm arm;
    private final GridHeight gridHeight;

    public enum GridHeight {
        High,
        Low
    }

    public AutoSetArm(Arm arm, GridHeight gridHeight) {
        this.arm = arm;
        this.gridHeight = gridHeight;
    }

    @Override
    public void initialize() {
        if(gridHeight == GridHeight.High) {
            arm.highSetpoint();
        }

        else {
            arm.middleSetpoint();
        }
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }
}
