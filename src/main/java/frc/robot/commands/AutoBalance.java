package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Swerve;

public class AutoBalance extends CommandBase {
    private Swerve swerve;
    private LEDs leds;

    private Notifier blinkLeds = new Notifier(this::blinkLeds);

    private boolean isRedAlliance = DriverStation.getAlliance() == Alliance.Red;

    public AutoBalance(Swerve swerve, LEDs leds) {
        this.swerve = swerve;
        this.leds = leds;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        blinkLeds.startPeriodic(0.1); // TODO Check
    }

    @Override
    public void execute() {
        double currentPitch = swerve.getPitch();
        Rotation2d currentRotation = swerve.getYaw();
        double currentRotDegrees = currentRotation.getDegrees();

        if(currentPitch <= Math.abs(2.5)) {
            return;
        } 
        
        else if (currentPitch <= -2.5) {
            double pitchAngleRad = currentPitch * (Math.PI / 180.);
            double speed = currentPitch <= -2.5 ? Math.sin(pitchAngleRad) : Math.sin(pitchAngleRad) * -1;
            
            swerve.drive(new Translation2d(0, speed), currentRotDegrees > 0 ? currentRotDegrees - currentRotDegrees : currentRotDegrees + currentRotDegrees, false, false);
        }
    }

    boolean ledStatus = false;

    private void blinkLeds() {
        if(ledStatus == true) {
            leds.off();

            ledStatus = false;
        } 

        else {
            if(isRedAlliance) {
                leds.setColor(255, 0, 0); 
            } 
            
            else {
                leds.setColor(0, 0, 255);
            }

            ledStatus = true;
        }
    }
}
