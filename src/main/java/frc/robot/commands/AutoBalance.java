package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Swerve;

public class AutoBalance extends CommandBase {
    private Swerve swerve;
    private LEDs leds;

    public AutoBalance(Swerve swerve, LEDs leds) {
        this.swerve = swerve;
        this.leds = leds;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        leds.chargingLights();
    }

    @Override
    public void execute() {
        double currentPitch = swerve.getPitch();
        Rotation2d currentRotation = swerve.getYaw();
        double currentRotDegrees = currentRotation.getDegrees();

        if(currentPitch <= Math.abs(2.5)) {
            return;
        } else if (currentPitch <= -2.5) {
            double pitchAngleRad = currentPitch * (Math.PI / 180.);
            double speed = currentPitch <= -2.5 ? Math.sin(pitchAngleRad) : Math.sin(pitchAngleRad) * -1;
            
            swerve.drive(new Translation2d(0, speed), currentRotDegrees > 0 ? currentRotDegrees - currentRotDegrees : currentRotDegrees + currentRotDegrees, false, false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        DriverStation.isDisabled();
    }
}
