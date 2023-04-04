package frc.robot.subsystems;

import java.util.function.Supplier;

// import org.frc5587.lib.subsystems.PivotingArmBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Robot;

public class Arm extends PivotingArmBase {
    private static WPI_TalonFX leader = new WPI_TalonFX(ArmConstants.LEADER_PORT);
    private static WPI_TalonFX follower = new WPI_TalonFX(ArmConstants.FOLLOWER_PORT);
    private static MotorControllerGroup group = new MotorControllerGroup(leader, follower);
    // private static DigitalInput limitSwitch = new DigitalInput(ArmConstants.SWITCH_PORT);
    private final DutyCycleEncoder throughBore = new DutyCycleEncoder(1);
    private final Supplier<Pose2d> poseSupplier;
    private boolean shouldLower, shouldLowerOverride = false;

    public Arm(ColorSensor colorSensor, Supplier<Pose2d> poseSupplier) {
        super("Arm", ArmConstants.ARM_CONSTANTS, group);
        this.poseSupplier = poseSupplier;
        configureMotors();
        this.enable();

        throughBore.setDutyCycleRange(1./1024., 1023./1024.);

        SmartDashboard.putBoolean("Arm Brake Mode", true);
    }

    @Override
    public double getEncoderPosition() {
        // return leader.getSelectedSensorPosition();
        return -(throughBore.getAbsolutePosition() - 0.676);
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

    // /**
    //  * @param switchPort the port of the limit switch we want the value of
    //  * @return the limit switch's state, inverted if necessary.
    //  */
    // public boolean getLimitSwitchValue() {
    //     return ArmConstants.SWITCH_INVERTED ? !limitSwitch.get() : limitSwitch.get();
    // }

    // /**
    //  * @param switchPort the port of the limit switch you want to get
    //  * @return the DigitalInput of the switch
    //  */
    // public DigitalInput getLimitSwitchObject() {
    //     return limitSwitch;
    // }
    
    public void highSetpoint() {
        shouldLowerOverride = true;
        getController().setGoal(ArmConstants.HIGH_SETPOINT);
    }

    public void middleSetpoint() {
        shouldLowerOverride = true;
        getController().setGoal(ArmConstants.MEDIUM_SETPOINT);
    }

    public void lowSetpoint() {
        shouldLowerOverride = true;
        getController().setGoal(ArmConstants.HOVER_SETPOINT); // TODO
    }

    public void hoverSetpoint() {
        shouldLowerOverride = true;
        getController().setGoal(ArmConstants.HOVER_SETPOINT);
    }

    public void intakeSetpoint() {
        shouldLowerOverride = true;
        getController().setGoal(ArmConstants.INTAKE_SETPOINT);
    }

    public void stow() {
        shouldLowerOverride = true;
        getController().setGoal(ArmConstants.STOW_SETPOINT);
    }

    public void substationSetpoint() {
        shouldLowerOverride = true;
        getController().setGoal(ArmConstants.SUB_SETPOINT);
        
    }

    public void liftAwayFromGrid() {
        this.setGoal(pidController.getGoal().position+Units.degreesToRadians(5));
    }
    
    public void lowerFromGrid() {
        this.setGoal(pidController.getGoal().position-Units.degreesToRadians(5));
    }

    public void setShouldLower(boolean shouldLower) {
        if(shouldLowerOverride) {
            shouldLower = false;
        }
        this.shouldLower = shouldLower;
    }

    public boolean inSubstation(Pose2d pose) {
        boolean withinX;
        boolean withinY;
        if(DriverStation.getAlliance() == Alliance.Red) {
            withinX = pose.getX() > AutoConstants.RED_SUBSTATION_BOUNDS[0].getX() && pose.getX() < AutoConstants.RED_SUBSTATION_BOUNDS[1].getX();
            withinY = pose.getY() > AutoConstants.RED_SUBSTATION_BOUNDS[0].getY() && pose.getY() < AutoConstants.RED_SUBSTATION_BOUNDS[1].getY();
        }
        else {
            withinX = pose.getX() > AutoConstants.BLUE_SUBSTATION_BOUNDS[0].getX() && pose.getX() < AutoConstants.BLUE_SUBSTATION_BOUNDS[1].getX();
            withinY = pose.getY() > AutoConstants.BLUE_SUBSTATION_BOUNDS[0].getY() && pose.getY() < AutoConstants.BLUE_SUBSTATION_BOUNDS[1].getY();
        }
        return withinX && withinY;
    }

    public boolean inLoweringArea(Pose2d pose) {
        boolean withinX = pose.getX() > AutoConstants.ARM_POSE_WINDOW[0].getX() && pose.getX() < AutoConstants.ARM_POSE_WINDOW[1].getX();
        boolean withinY = pose.getY() > AutoConstants.ARM_POSE_WINDOW[0].getY() && pose.getY() < AutoConstants.ARM_POSE_WINDOW[1].getY();
        return withinX && withinY;
    }

    @Override
    public void periodic() {
        super.periodic();
        if(Robot.m_debugMode) {
            SmartDashboard.putBoolean("In Substation", inSubstation(poseSupplier.get()));
            SmartDashboard.putBoolean("In Lowering Area", inLoweringArea(poseSupplier.get()));
        }
        // if(getLimitSwitchValue()) {
        //     this.resetEncoders();
        //     this.setGoal(Units.degreesToRadians(2));
        // }

        // if((inLoweringArea(poseSupplier.get()) && !inSubstation(poseSupplier.get()))
        //         && (pidController.getSetpoint().position > ArmConstants.HOVER_SETPOINT 
        //         || (colorSensor.hasElement() && pidController.getSetpoint().position < ArmConstants.HOVER_SETPOINT))) {
        //     setShouldLower(true);
        // }
        // else {
        //     setShouldLower(false);
        //     shouldLowerOverride = false;
        // }
        // if(shouldLower) {
        //     setGoal(ArmConstants.HOVER_SETPOINT);
        // }
            
        if(SmartDashboard.getBoolean("Arm Brake Mode", true)) {
                leader.setNeutralMode(NeutralMode.Brake);
                follower.setNeutralMode(NeutralMode.Brake);
        } else {
                leader.setNeutralMode(NeutralMode.Coast);
                follower.setNeutralMode(NeutralMode.Coast);
        }

        if(!throughBore.isConnected()) {
            this.disable();
            this.stop();
        }
    }
}
