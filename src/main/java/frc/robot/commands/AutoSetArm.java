package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class AutoSetArm extends CommandBase {
    private final Arm arm;
    private final GridHeight gridHeight;

    public enum GridHeight {
        High,
        Middle,
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

        else if(gridHeight == GridHeight.Middle) {
            arm.middleSetpoint();
        }

        else {
            arm.intakeSetpoint();
        }
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }
}
