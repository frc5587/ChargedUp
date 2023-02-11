package frc.robot.subsystems;

import org.frc5587.lib.subsystems.PistonControl;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Intake extends PistonControl {
    public static final DoubleSolenoid[] solenoid = {new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1)};

    public Intake() {
        super(solenoid);
    }
}
