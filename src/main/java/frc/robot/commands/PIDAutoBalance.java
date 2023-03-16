package frc.robot.commands;

import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PIDAutoBalance extends CommandBase {
    private Double desiredHeading = 0.0;
    private boolean fieldrelative = true;
    private boolean openLoop = false;
    private boolean isFinished = false;
    
    private final PIDController headingController = 
        new PIDController(SwerveConstants.ANGLE_FPID.kP, SwerveConstants.ANGLE_FPID.kI, SwerveConstants.ANGLE_FPID.kD); // TODO Once this is working, move into constants and use for all headings
       
    private double rotation;
    private Translation2d translation = new Translation2d(0, 0);
    private Swerve swerve;

    public PIDAutoBalance(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);  
    }
  
    @Override
    public void initialize() {
        isFinished = false;
        swerve.stopWithLock(true);
    }
    @Override
    public void execute() {
        rotation = headingController.calculate(swerve.getYaw().getDegrees(), desiredHeading);  
        if (swerve.getPitch() > 9) {
            translation = new Translation2d(-0.3, 0.0); // Speed is in Meters/s
        } else if (swerve.getPitch() < -9) {
            translation = new Translation2d(0.3, 0);
        } else {
            translation = new Translation2d(0 , 0);
            if(Timer.getMatchTime() <= 0.1) {
                swerve.stopWithLock(false);
                isFinished = true;
            }
        }

        swerve.drive(translation, rotation, fieldrelative, openLoop);
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean innterruped) {
        swerve.stopWithLock(false);
        isFinished = true;
    }
}
