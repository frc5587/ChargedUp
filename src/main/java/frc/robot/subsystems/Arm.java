package frc.robot.subsystems;

import org.frc5587.lib.subsystems.PivotingArmBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Arm extends PivotingArmBase {
    private static WPI_TalonFX motor = new WPI_TalonFX(Constants.ArmConstants.MOTOR_PORT);

    public Arm() {
        super(Constants.ArmConstants.ARM_CONSTANTS, motor);
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
    
    public void highSetpoint() {
        this.setGoal(ArmConstants.HIGH_SETPOINT);
    }

    public void middleSetpoint() {
        this.setGoal(ArmConstants.MEDIUM_SETPOINT);
    }

    public void intakeSetpoint() {
        this.setGoal(ArmConstants.INTAKE_SETPOINT);
    }

    public void stow() {
        this.setGoal(ArmConstants.STOW_SETPOINT);
    }
}
