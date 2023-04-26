package frc.robot.subsystems;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

// import org.frc5587.lib.subsystems.PivotingArmBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.util.titanlib.PivotingArmBase;
import frc.robot.Robot;

public class Arm extends PivotingArmBase {
    private static WPI_TalonFX leader = new WPI_TalonFX(ArmConstants.LEADER_PORT);
    private static WPI_TalonFX follower = new WPI_TalonFX(ArmConstants.FOLLOWER_PORT);
    private static MotorControllerGroup group = new MotorControllerGroup(leader, follower);
    // private static DigitalInput limitSwitch = new DigitalInput(ArmConstants.SWITCH_PORT);
    private final DutyCycleEncoder throughBore = new DutyCycleEncoder(1);
    private final Supplier<Pose2d> poseSupplier;
    private final Mechanism2d armMechanism2d = new Mechanism2d(2, 2);
    private final MechanismRoot2d armRoot = armMechanism2d.getRoot("armRoot", 0, Units.inchesToMeters(31.5));
    private final MechanismLigament2d structureLigament = armRoot.append(new MechanismLigament2d("structure", Units.inchesToMeters(31.5), 90));
    private final MechanismLigament2d armLigament = structureLigament.append(new MechanismLigament2d("structure", Units.inchesToMeters(31.5), 182));
    public final MechanismLigament2d wristLigament = armLigament.append(new MechanismLigament2d("wristLigament", Units.inchesToMeters(9.7), 90, 6., new Color8Bit(Color.kBlue)));
    public Pose3d armPose3d = new Pose3d();

    public Arm(ColorSensor colorSensor, Supplier<Pose2d> poseSupplier) {
        super("Arm", ArmConstants.ARM_CONSTANTS, group);
        this.poseSupplier = poseSupplier;
        configureMotors();
        this.enable();

        throughBore.setDutyCycleRange(1./1024., 1023./1024.);

        SmartDashboard.putBoolean("Arm Brake Mode", true);
        SmartDashboard.putData("Arm Mech", armMechanism2d);
    }

    @Override
    public double getEncoderPosition() {
        // return leader.getSelectedSensorPosition();
        return -(throughBore.getAbsolutePosition() - 0.717+0.084);
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
    
    public void highSetpoint() {
        getController().setGoal(ArmConstants.HIGH_SETPOINT);
    }

    public void middleSetpoint() {
        getController().setGoal(ArmConstants.MEDIUM_SETPOINT);
    }

    public void lowSetpoint() {
        getController().setGoal(ArmConstants.HOVER_SETPOINT);
    }

    public void hoverSetpoint() {
        getController().setGoal(ArmConstants.HOVER_SETPOINT);
    }

    public void intakeSetpoint() {
        getController().setGoal(ArmConstants.INTAKE_SETPOINT);
    }

    public void stow() {
        getController().setGoal(ArmConstants.STOW_SETPOINT);
    }

    public void substationSetpoint() {
        getController().setGoal(ArmConstants.SUB_SETPOINT);
        
    }

    public void liftAwayFromGrid() {
        this.setGoal(pidController.getGoal().position+Units.degreesToRadians(5));
    }
    
    public void lowerFromGrid() {
        this.setGoal(pidController.getGoal().position-Units.degreesToRadians(5));
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
            
        if(SmartDashboard.getBoolean("Arm Brake Mode", true)) {
                leader.setNeutralMode(NeutralMode.Brake);
                follower.setNeutralMode(NeutralMode.Brake);
        } else {
                leader.setNeutralMode(NeutralMode.Coast);
                follower.setNeutralMode(NeutralMode.Coast);
        }

        // note that we cannot do the else condition here to re-enable, as enabling every 
        // period causes issues in the pid controller.
        if(!throughBore.isConnected()) {
            this.disable();
            this.stop();
        }
    }

    @Override
    public void simulationPeriodic() {
        armLigament.setAngle(180 + Units.radiansToDegrees(getController().getGoal().position));
        armPose3d = new Pose3d(0.2,0,0.85,new Rotation3d(0,-getController().getGoal().position,0));
        Logger.getInstance().recordOutput("Arm Pose", armPose3d);
    }
}
