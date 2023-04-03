package frc.robot.subsystems;

import org.frc5587.lib.control.DeadbandCommandXboxController;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class Rumbler extends SubsystemBase {
    private DeadbandCommandXboxController xb;
    private Swerve swerve;
    private double timeRumbling;
    private double strongRumbleTime = 0;
    private boolean isRumbling = false;

    public Rumbler(DeadbandCommandXboxController xb, Swerve swerve) {
        this.xb = xb;
        this.swerve = swerve;
        SmartDashboard.putNumber("RUMBLE VAL", 0);
    }

    @Override
    public void periodic() {
        double[] driveCurrents = new double[4];
        double avg = 0;
        double rumbleVal = 0;

        for(int i = 0; i < swerve.mSwerveMods.length; i++) {
            driveCurrents[i] = swerve.mSwerveMods[i].mDriveMotor.getStatorCurrent();
            avg += driveCurrents[i];

            if(i == swerve.mSwerveMods.length) {
                avg /= 4;
            }
        }

        if(timeRumbling == 0 && avg >= SwerveConstants.RUMBLE_THRESHOLD && !isRumbling) {
            isRumbling = true;
        }

        if(isRumbling) {
            timeRumbling += 0.02;
        }

        if(timeRumbling > 2) {
            isRumbling = false;
            timeRumbling = 0;
            strongRumbleTime += 0.02;
            xb.getHID().setRumble(RumbleType.kBothRumble, 1);
        }

        SmartDashboard.putNumber("RumbleTime", timeRumbling);
        SmartDashboard.putNumber("StrongTime", strongRumbleTime);

        if(isRumbling) {
            xb.getHID().setRumble(RumbleType.kLeftRumble, timeRumbling/5);
        }
        else if(strongRumbleTime > 0) {
            xb.getHID().setRumble(RumbleType.kBothRumble, 1);
            strongRumbleTime += 0.02;
            if(strongRumbleTime > 1) {
                strongRumbleTime = 0;
            }
        }
        else {
            xb.getHID().setRumble(RumbleType.kBothRumble, 0);
        }
    }
}
