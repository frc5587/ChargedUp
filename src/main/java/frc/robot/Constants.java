package frc.robot;

import org.frc5587.lib.pid.FPID;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.swervelib.util.COTSFalconSwerveConstants;
import frc.robot.util.swervelib.util.SwerveModuleConstants;
import frc.robot.util.titanlib.PivotingArmBase.PivotingArmConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class SwerveConstants {
        public static final boolean INVERT_GYRO = true; // Always ensure Gyro is CCW+ CW-

        public static final boolean TUNING = false;

        public static final COTSFalconSwerveConstants CHOSEN_MODULE = 
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L1);

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = Units.inchesToMeters(19.51); // distance from left wheel to right wheel
        public static final double WHEEL_BASE = Units.inchesToMeters(21.25); // distance from front wheel to back wheel
        public static final double WHEEL_CIRCUMFERENCE_METERS = CHOSEN_MODULE.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));


        /* Module Gear Ratios */
        public static final double DRIVE_GEAR_RATIO = CHOSEN_MODULE.driveGearRatio;
        public static final double ANGLE_GEAR_RATIO = CHOSEN_MODULE.angleGearRatio;

        /* Motor Inverts */
        public static final boolean DRIVE_MOTOR_INVERTED = CHOSEN_MODULE.driveMotorInvert;
        public static final boolean ANGLE_MOTOR_INVERTED = CHOSEN_MODULE.angleMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean CANCODER_INVERTED = CHOSEN_MODULE.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int DRIVE_CONT_LIMIT = 35;
        public static final int DRIVE_PEAK_LIMIT = 40;
        public static final double DRIVE_PEAK_DURATION = 0.1;
        public static final boolean DRIVE_LIMIT_ENABLED = true;
        public static final int RUMBLE_THRESHOLD = 35;
        
        public static final double SLEW_RATE = 3; // m/s^2 // TODO CHANGE AFTER ARM IS ADDED

        public static final int ANGLE_CONT_LIMIT = 25;
        public static final int ANGLE_PEAK_LIMIT = 40;
        public static final double ANGLE_PEAK_DURATION = 0.1;
        public static final boolean ANGLE_LIMIT_ENABLED = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        /* Drive Motor PID Values */
        public static final FPID DRIVE_FPID = new FPID(
            0.02, 0.1, 0, 0);//0.05, 0.03, 0., 0.); // //2.8884 for P

        /* Angle Motor PID Values */
        public static final FPID ANGLE_FPID = new FPID(
                CHOSEN_MODULE.angleKF, CHOSEN_MODULE.angleKP, CHOSEN_MODULE.angleKI, CHOSEN_MODULE.angleKD); // 0.05
        

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        //COMMENTED VALS IN () ARE FROM BASEFALCONSWERVE, OTHER COMMENTED VALS ARE SYSID, USED VALS ARE FROM FRESTA
        public static final double DRIVE_KS = (0.32 / 12);//.18576/12; // 0.23034/12; ;
        public static final double DRIVE_KV = (1.51 / 12);//2.3317/12; // 2.6998/12; // ;
        public static final double DRIVE_KA = (0.27 / 12);//0.25916/12; // 0.29868/12; // 
        public static final SimpleMotorFeedforward DRIVE_FF = new SimpleMotorFeedforward(DRIVE_KS, DRIVE_KV, DRIVE_KA);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double MAX_SPEED = 6.;//5.;
        /** Radians per Second */
        public static final double MAX_ANGULAR_VELOCITY = 6.;

        /* Neutral Modes */
        public static final NeutralMode ANGLE_NEUTRAL_MODE = NeutralMode.Coast;
        public static final NeutralMode DRIVE_NEUTRAL_MODE = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int DRIVE_ID = 10;
            public static final int ANGLE_ID = 15;
            public static final int CANCODER_ID = 50;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(213.75);
            public static final boolean ENCODER_INVERTED = false;
            public static final SwerveModuleConstants MODULECONSTANTS = 
                new SwerveModuleConstants(DRIVE_ID, ANGLE_ID, CANCODER_ID, ANGLE_OFFSET);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { 
            public static final int DRIVE_ID = 11;
            public static final int ANGLE_ID = 16;
            public static final int CANCODER_ID = 51;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(192.129);
            public static final boolean ENCODER_INVERTED = false;
            public static final SwerveModuleConstants MODULECONSTANTS = 
                new SwerveModuleConstants(DRIVE_ID, ANGLE_ID, CANCODER_ID, ANGLE_OFFSET);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int DRIVE_ID = 12;
            public static final int ANGLE_ID = 17;
            public static final int CANCODER_ID = 52;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(256.008); // 260.332
            public static final boolean ENCODER_INVERTED = false;
            public static final SwerveModuleConstants MODULECONSTANTS = 
                new SwerveModuleConstants(DRIVE_ID, ANGLE_ID, CANCODER_ID, ANGLE_OFFSET);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { 
            public static final int DRIVE_ID = 13;
            public static final int ANGLE_ID = 18;
            public static final int CANCODER_ID = 53;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(306.474); // 194.169;
            public static final boolean ENCODER_INVERTED = false;
            public static final SwerveModuleConstants MODULECONSTANTS = 
                new SwerveModuleConstants(DRIVE_ID, ANGLE_ID, CANCODER_ID, ANGLE_OFFSET);
        }
    }

    public static final class AutoConstants { // TODO Confirm
        public static final double MAX_SPEED_MPS = 4; // 3.  // in m/s 
        public static final double MAX_ACCEL_MPS_2 = 2; // 3. // in m/s^2 
        public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(MAX_SPEED_MPS, MAX_ACCEL_MPS_2);
        public static final double MAX_ANGULAR_SPEED_R_S = Math.PI; // Math.PI / 4.; // in radians/s 
        public static final double MAX_ANGULAR_ACCEL_R_S_2 = Math.PI; // Math.PI / 4.; // in radians/s^2 
        public static final double CRAWL_SPEED = Units.inchesToMeters(5); //m/s

        public static final double KP_X_CONTROLLER = 1;//126.04; // 1. // THIS AFFECTS AUTO 
        public static final double KP_Y_CONTROLLER = 1; // THIS AFFECTS AUTO
        public static final double KD_XY_CONTROLLER = 0;//4.4556;
        public static final double KP_THETA_CONTROLLER = 0.001; //2.; // 7.; // 0.02; // THIS AFFECTS AUTO AND DRIVETOPOSE
        public static final double KD_THETA_CONTROLLER = 0; //2.; // 7.; // 0.02; // THIS AFFECTS AUTO AND DRIVETOPOSE

        public static final double KP_DRIVE_CONTROLLER = 19.336; //2.5; // THIS AFFECTS DRIVETOPOSE
    
        public static final TrapezoidProfile.Constraints K_PXY_CONSTRAINTS = // DRIVETOPOSE
            new TrapezoidProfile.Constraints(MAX_SPEED_MPS, MAX_ACCEL_MPS_2);

        public static final TrapezoidProfile.Constraints K_THETA_CONSTRAINTS = // AUTO AND DRIVETOPOSE
            new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED_R_S, MAX_ANGULAR_ACCEL_R_S_2);

        public static final ProfiledPIDController BOT_DRIVE_CONTROLLER = // DRIVETOPOSE
                new ProfiledPIDController(
                    KP_DRIVE_CONTROLLER, 0.0, KD_XY_CONTROLLER, K_PXY_CONSTRAINTS);
        
        public static final ProfiledPIDController BOT_ANGLE_CONTROLLER = // AUTO AND DRIVETOPOSE
                new ProfiledPIDController(
                    KP_THETA_CONTROLLER, 0.0, 0.0, K_THETA_CONSTRAINTS); // enableContinuousInput(-Math.PI, Math.PI);
                    
        public static final PIDConstants TRANSL_CONSTANTS = new PIDConstants(KP_X_CONTROLLER, 0, 0); // AUTO // TODO Characterize using simple motor with locked straight (clamp belts??)
        public static final PIDConstants THETA_CONSTANTS = new PIDConstants(KP_THETA_CONTROLLER, 0, KD_THETA_CONTROLLER); // AUTO // TODO Manually tune P & D for position loop
        // public static final PIDController BOT_X_CONTROLLER = new PIDController(KP_X_CONTROLLER, 0, KD_XY_CONTROLLER); // AUTO
        // public static final PIDController BOT_Y_CONTROLLER = new PIDController(KP_Y_CONTROLLER, 0, KD_XY_CONTROLLER); // AUTO
        // public static final PIDController BOT_ROT_CONTROLLER = new PIDController(0, 0, 0); // TODO
        // public static final HolonomicDriveController DRIVE_CONTROLLER = new HolonomicDriveController(
        //     BOT_Y_CONTROLLER, BOT_X_CONTROLLER, BOT_ANGLE_CONTROLLER); // AUTO

        // public static final PPSwerveControllerCommand PPSwerveController(PathPlannerTrajectory traj, Swerve swerve) {
        //     return new PPSwerveControllerCommand(
        //         traj, 
        //         swerve::getPose,
        //         SwerveConstants.SWERVE_KINEMATICS,
        //         BOT_X_CONTROLLER,
        //         BOT_Y_CONTROLLER,
        //         BOT_ROT_CONTROLLER,
        //         swerve::setModuleStates,
        //         true,
        //         swerve
        //     );
        // }

        public static final class GridLocationGroup {
            public final Pose2d greaterPose, poseLeft, poseRight;
            public final Pose2d[] poseArray;

            public GridLocationGroup(Pose2d greaterPose, Alliance alliance) {
                this.greaterPose = greaterPose;
                if(alliance.equals(Alliance.Blue)) {
                    this.poseLeft = new Pose2d(greaterPose.getX(), greaterPose.getY()+.56, greaterPose.getRotation());
                    this.poseRight = new Pose2d(greaterPose.getX(), greaterPose.getY()-.56, greaterPose.getRotation());
                }
                else {
                    this.poseLeft = new Pose2d(greaterPose.getX(), greaterPose.getY()-.56, greaterPose.getRotation());
                    this.poseRight = new Pose2d(greaterPose.getX(), greaterPose.getY()+.56, greaterPose.getRotation());
                }
                this.poseArray = new Pose2d[]{this.poseLeft, this.greaterPose, this.poseRight};
            }

            public GridLocationGroup(Pose2d greaterPose, Pose2d poseLeft, Pose2d poseRight) {
                this.greaterPose = greaterPose;
                this.poseLeft = poseLeft;
                this.poseRight = poseRight;
                this.poseArray = new Pose2d[]{this.poseLeft, this.greaterPose, this.poseRight};
            }
        }

        public static final GridLocationGroup BLUE_LEFT = new GridLocationGroup(new Pose2d(2, 4.42, Rotation2d.fromDegrees(0)), Alliance.Blue);
        public static final GridLocationGroup BLUE_CENTER = new GridLocationGroup(new Pose2d(2, 2.75, Rotation2d.fromDegrees(0)), Alliance.Blue);
        public static final GridLocationGroup BLUE_RIGHT = new GridLocationGroup(new Pose2d(2, 1.06, Rotation2d.fromDegrees(0)), Alliance.Blue);
        public static final GridLocationGroup RED_RIGHT = new GridLocationGroup(new Pose2d(14.525, 4.42, Rotation2d.fromDegrees(-0)), Alliance.Red);
        public static final GridLocationGroup RED_CENTER = new GridLocationGroup(new Pose2d(14.525, 2.75, Rotation2d.fromDegrees(-0)), Alliance.Red);
        public static final GridLocationGroup RED_LEFT = new GridLocationGroup(new Pose2d(14.525, 1.06, Rotation2d.fromDegrees(-0)), Alliance.Red);

        public static final GridLocationGroup[] GRID_LOCATIONS = {
            BLUE_LEFT, BLUE_CENTER, BLUE_RIGHT, RED_RIGHT, RED_CENTER, RED_LEFT
        };

        public static final Translation2d[] ARM_POSE_WINDOW = {
            new Translation2d(4, 0), 
            new Translation2d(12.5, 8)};
        
        public static final Translation2d[] BLUE_SUBSTATION_BOUNDS = {
            new Translation2d(9.85, 5.5),
            new Translation2d(16.5, 8)
        };

        public static final Translation2d[] RED_SUBSTATION_BOUNDS = {
            new Translation2d(0, 5.5),
            new Translation2d(6.66, 8)
        };

        // first value is double substation far from bump side, second is closer, 
        // and third is single substation
        public static final Pose2d[] BLUE_SUBS = {
            new Pose2d(15.42, 7.16, Rotation2d.fromDegrees(0)),
            new Pose2d(15.42, 6, Rotation2d.fromDegrees(0)),
            new Pose2d(13.85, 7.22, Rotation2d.fromDegrees(90))
        };

        public static final Pose2d[] RED_SUBS = {
            new Pose2d(1.1, 7.16, Rotation2d.fromDegrees(180)),
            new Pose2d(1.1, 6, Rotation2d.fromDegrees(180)),
            new Pose2d(2.31, 7.22, Rotation2d.fromDegrees(90))
        };

        public static final Translation2d[] BLUE_COMMUNITY = {
            new Translation2d(0, 0),
            new Translation2d(5, 4),
            new Translation2d(3.33, 5.5)
        };

        public static final Translation2d[] RED_COMMUNITY = {
            new Translation2d(16.5, 0),
            new Translation2d(11.66, 4),
            new Translation2d(13.2, 5.5)
        };
    }

    public static class LimelightConstants {
        public static final double MOUNT_ANGLE = 0;
        public static final double LENS_HEIGHT = Units.inchesToMeters(38.275);
        public static final double GOAL_HEIGHT = Units.inchesToMeters(14.25);
        public static final double DISTANCE_OFFSET = 0;
    }

    public static final class ArmConstants {
        public static final int LEADER_PORT = 20;
        public static final int FOLLOWER_PORT = 21;
        public static final double GEARING = 1;//240;
        public static final double VELOCITY_DENOMINATOR = 0.1;
        public static final double[] SOFT_LIMITS = {0, Units.degreesToRadians(100)};
        public static final int ENCODER_CPR = 1;
        public static final int ZERO_OFFSET = Math.round((float) (Units.degreesToRadians(-2) * GEARING * ENCODER_CPR / 2 / Math.PI)); // TODO: Find
        public static final int SWITCH_PORT = 0;
        public static final boolean SWITCH_INVERTED = true;
        public static final TrapezoidProfile.Constraints PID_CONSTRAINTS = new TrapezoidProfile.Constraints(1, 0.8); // TODO: Verify
        public static final double KP = 4.865;//7.5857;//25.059;//12.556;
        public static final double KI = 0;
        public static final double KD = 0.91242;//4.6857;//15.771;
        public static final double KS = 0.17784;//0.28737;
        public static final double KG = 0.20876;//0.31488;
        public static final double KV = 4.0448;//3.5722;
        public static final ProfiledPIDController ARM_PID = new ProfiledPIDController(KP, KI, KD, PID_CONSTRAINTS);
        public static final ArmFeedforward ARM_FF = new ArmFeedforward(KS, KG, KV);
        public static final double HIGH_SETPOINT = Units.degreesToRadians(120);
        public static final double MEDIUM_SETPOINT = Units.degreesToRadians(108);
        public static final double INTAKE_SETPOINT = Units.degreesToRadians(17);
        public static final double HOVER_SETPOINT = Units.degreesToRadians(25);
        public static final double STOW_SETPOINT = Units.degreesToRadians(-2);
        public static final double SUB_SETPOINT = Units.degreesToRadians(90);
        public static final double FF_ANGLE_OFFSET = -Units.degreesToRadians(90);

        public static final PivotingArmConstants ARM_CONSTANTS = new PivotingArmConstants(
                GEARING, VELOCITY_DENOMINATOR, FF_ANGLE_OFFSET, SOFT_LIMITS, ZERO_OFFSET, ENCODER_CPR, ARM_PID, ARM_FF);
    }

    public static class IntakeConstants {
        public static final int LEFT_MOTOR = 30;
        public static final int RIGHT_MOTOR = 31;

        public static final double THROTTLE_FORWARD = 1; // TODO
        public static final double THROTTLE_REVERSE = .25; // TODO
        public static final double THROTTLE_AUTO = -1; // TODO

        public static final int STALL_LIMIT = 20;
        public static final int FREE_LIMIT = 25;

        public static final boolean LEFT_MOTOR_INVERTED = false; // TODO
        public static final boolean RIGHT_MOTOR_INVERTED = true; // TODO

        public static final double LEFT_VELOCITY_THRESHOLD = 10; // rotations per second
        public static final double RIGHT_VELOCITY_THRESHOLD = 10;
        public static final double EJECT_RUNTIME = 2; // seconds
    }

    public static class WristConstants {
        public static final Constraints CONSTRAINTS = new Constraints(5, 5);
        public static final ProfiledPIDController PID_CONTROLLER = new ProfiledPIDController(5.5, 0, 0, CONSTRAINTS); //12pm // kD 1.9861
        public static final ArmFeedforward FF_CONTROLLER = new ArmFeedforward(0.7635, 2.0434, 0.074894);
        public static final ArmFeedforward HIGH_FF = new ArmFeedforward(0.55044, 2.2877, 0.3263);
        public static final double GEARING = 1;
        public static final int ENCODER_CPR = 1;
        public static final boolean LEFT_INVERTED = true;
        public static final boolean RIGHT_INVERTED = false;
        public static final int SWITCH_PORT = 0;
    }
    
    public static final class LEDConstants {
        public static final int PORT = 0;
        public static final int STRIP_LENGTH = 84;
    }
}
