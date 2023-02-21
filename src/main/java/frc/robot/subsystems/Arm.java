package frc.robot.subsystems;

import org.frc5587.lib.subsystems.PivotingArmBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.util.Units;

public class Arm extends PivotingArmBase {
    private static WPI_TalonFX motor = new WPI_TalonFX(Constants.ArmConstants.motorPort);

    public Arm() {
        super(Constants.ArmConstants.armConstants, motor);
    }

    @Override
    public double getEncoderPosition() {
        return motor.getSelectedSensorPosition();
    }

    @Override
    public double getEncoderVelocity() {
        return motor.getSelectedSensorVelocity();
    }

    @Override
    public void setEncoderPosition(double position) {
        motor.setSelectedSensorPosition(position);
    }

    @Override
    public void configureMotors() {
        motor.configFactoryDefault();
        motor.setNeutralMode(NeutralMode.Brake);
        motor.setInverted(false);
    }
    
    public void moveToTop() {
        this.setGoal(Units.degreesToRadians(90));
    }

    public void moveToBottom() {
        this.setGoal(0);
    }
}
