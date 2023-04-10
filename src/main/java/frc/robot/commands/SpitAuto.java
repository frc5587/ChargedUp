package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.util.GamePiece;

public class SpitAuto extends CommandBase {
    private Intake intake;
    private Arm arm;
    private GamePiece gamePiece;

    public SpitAuto(Intake intake, Arm arm, GamePiece gamePiece) {
        this.intake = intake;
        this.arm = arm;
        this.gamePiece = gamePiece;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        if(arm.getController().atSetpoint()) {
            if(gamePiece == GamePiece.CUBE) {
                intake.spitCube();
            } else {
                intake.spitCone();
            }
        }
    }
}
