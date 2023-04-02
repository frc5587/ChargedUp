package frc.robot.subsystems;

import org.frc5587.lib.control.DeadbandCommandXboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rumbler extends SubsystemBase {
    private DeadbandCommandXboxController xb;
    private Swerve swerve;

    public Rumbler(DeadbandCommandXboxController xb, Swerve swerve) {
        this.xb = xb;
        this.swerve = swerve;
    }

    @Override
    public void periodic() {
        double[] driveCurrents = new double[4];

        for(int i = 0; i < swerve.mSwerveMods.length; i++) {
            driveCurrents[i] = swerve.mSwerveMods[i].mDriveMotor.getStatorCurrent();
        }
    }
}
