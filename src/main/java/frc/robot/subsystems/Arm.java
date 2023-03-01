package frc.robot.subsystems;

import frc.robot.Constants.ArmConstants;
// import org.frc5587.lib.subsystems.PivotingArmBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class Arm extends PivotingArmBase {
    private static WPI_TalonFX leader = new WPI_TalonFX(ArmConstants.LEADER_PORT);
    private static WPI_TalonFX follower = new WPI_TalonFX(ArmConstants.FOLLOWER_PORT);
    private static MotorControllerGroup group = new MotorControllerGroup(leader, follower);
    private static DigitalInput limitSwitch = new DigitalInput(ArmConstants.SWITCH_PORT);

    public Arm() {
        super(ArmConstants.ARM_CONSTANTS, group);

        configureMotors();
        this.enable();
    }

    @Override
    public double getEncoderPosition() {
        return leader.getSelectedSensorPosition();
    }

    @Override
    public double getEncoderVelocity() {
        return leader.getSelectedSensorVelocity();
    }

    @Override
    public void setEncoderPosition(double position) {
        leader.setSelectedSensorPosition(position);
    }

    @Override
    public void configureMotors() {
        leader.configFactoryDefault();
        follower.configFactoryDefault();
        leader.setNeutralMode(NeutralMode.Brake);
        follower.setNeutralMode(NeutralMode.Brake);
        leader.setInverted(true);
        follower.setInverted(false);
    }

    /**
     * @param switchPort the port of the limit switch we want the value of
     * @return the limit switch's state, inverted if necessary.
     */
    public boolean getLimitSwitchValue() {
        return ArmConstants.SWITCH_INVERTED ? !limitSwitch.get() : limitSwitch.get();
    }

    /**
     * @param switchPort the port of the limit switch you want to get
     * @return the DigitalInput of the switch
     */
    public DigitalInput getLimitSwitchObject() {
        return limitSwitch;
    }
    
    public void highSetpoint() {
        getController().setGoal(ArmConstants.HIGH_SETPOINT);
    }

    public void middleSetpoint() {
        getController().setGoal(ArmConstants.MEDIUM_SETPOINT);
    }

    public void lowSetpoint() {
        getController().setGoal(ArmConstants.INTAKE_SETPOINT);
    }

    public void stow() {
        getController().setGoal(ArmConstants.STOW_SETPOINT);
    }

    public void liftAwayFromGrid() {
        this.setGoal(pidController.getGoal().position+Units.degreesToRadians(5));
    }
    
    public void lowerFromGrid() {
        this.setGoal(pidController.getGoal().position-Units.degreesToRadians(5));
    }
}
