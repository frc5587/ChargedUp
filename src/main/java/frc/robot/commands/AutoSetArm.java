package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class AutoSetArm extends CommandBase {
    private final Arm arm;
    private final GridHeight gridHeight;

    public enum GridHeight {
        High,
        Middle,
        Low,
        Intake,
        Stow,
        Substation
    }

    public AutoSetArm(Arm arm, GridHeight gridHeight) {
        this.arm = arm;
        this.gridHeight = gridHeight;
    }

    @Override
    public void execute() {
        switch(gridHeight) {
            case High:
                arm.highSetpoint();
                break;
            case Middle:
                arm.middleSetpoint();
                break;
            case Low:
                arm.lowSetpoint();
                break;
            case Intake:
                arm.intakeSetpoint();
                break;
            case Stow:
                arm.stow();
                break;
            case Substation:
                arm.substationSetpoint();
                break;
            default:
                arm.stow();
        }
    }

    // @Override
    // public boolean isFinished() {
    //     return arm.getController().atSetpoint();
    // }
}
