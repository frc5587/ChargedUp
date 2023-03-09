package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class AutoSetArm extends CommandBase {
    private final Arm arm;
    private final Intake intake;
    private final GridHeight gridHeight;

    public enum GridHeight {
        High,
        Middle,
        Low
    }

    public AutoSetArm(Arm arm, Intake intake, GridHeight gridHeight) {
        this.arm = arm;
        this.intake = intake;
        this.gridHeight = gridHeight;
    }

    @Override
    public void execute() {
        if(gridHeight == GridHeight.High) {
            arm.highSetpoint();
            // if(arm.getController().atGoal()) {
            //     intake.forward();
            //     intake.extend();
            // }
        }

        else if(gridHeight == GridHeight.Middle) {
            arm.middleSetpoint();
            // if(arm.getController().atGoal()) {
            //     intake.forward();
            //     intake.extend();
            // }
        }

        else {
            arm.lowSetpoint();
            // intake.retract();
            // intake.backward();
        }

        if(arm.getController().atGoal()) {
            end(false);
        }
    }
}
