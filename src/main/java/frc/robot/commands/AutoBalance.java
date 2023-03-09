package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
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
    private double angleDegrees;

    private final double speedInchesPerSec = 15;
    private final double positionThresholdDegrees = 3.0;
    private final double velocityThresholdDegreesPerSec = 8.0;

    public AutoBalance(Swerve swerve, LEDs leds) {
        this.swerve = swerve;
        this.leds = leds;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        blinkLeds.startPeriodic(0.4); // TODO Check

        angleDegrees = Double.POSITIVE_INFINITY;
    }

    @Override
    public void execute() {
        angleDegrees = swerve.getPitch();
        // double angleVelocityDegreesPerSec = swerve.getRotation().getCos() * Units.radiansToDegrees(swerve.getPitchVelocity()) + swerve.getRotation().getSin() * Units.radiansToDegrees(swerve.getRollVelocity);
        // boolean shouldStop = (angleDegrees < 0.0 && angleVelocityDegreesPerSec > velocityThresholdDegreesPerSec.get()) || (angleDegrees > 0.0 && angleVelocityDegreesPerSec < -velocityThresholdDegreesPerSec.get());
        boolean shouldStop = Math.abs(angleDegrees) < 2.5;

        if(shouldStop) {
            System.out.println("Within no move threshold");
            swerve.stop();
        } else {
            System.out.println("Autobalancing");
            swerve.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                Units.inchesToMeters(speedInchesPerSec) * (angleDegrees > 0.0 ? -1.0 : 1.0),
                0.0,
                0.0,
                swerve.getYaw()));
            
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stopWithLock();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(angleDegrees) < positionThresholdDegrees;
    }

    boolean ledStatus = false;

    // TODO test this
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
