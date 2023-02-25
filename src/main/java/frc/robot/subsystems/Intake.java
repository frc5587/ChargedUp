package frc.robot.subsystems;

import org.frc5587.lib.subsystems.PistonControl;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants.IntakeConstants;

public class Intake extends PistonControl {
    public static final DoubleSolenoid[] solenoid = { new DoubleSolenoid(PneumaticsModuleType.CTREPCM, IntakeConstants.FORWARD_CHANNEL, IntakeConstants.REVERSE_CHANNEL) };
    
    public Intake() {
        super(solenoid);
    }
}
