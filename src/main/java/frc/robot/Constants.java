package frc.robot;

import org.frc5587.lib.subsystems.PivotingArmBase.PivotingArmConstants;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

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
    public static final class ArmConstants {
        public static final int MOTOR_PORT = 15;
        public static final double GEARING = 3; // TODO: Calculate
        public static final double[] SOFT_LIMITS = {0, Units.degreesToRadians(100)};
        public static final int ZERO_OFFSET = 0; // TODO: Find
        public static final int ENCODER_CPR = 42; // Change to 2048 if using Falcon
        public static final int[] SWITCH_PORTS = {0};
        public static final boolean[] SWITCHES_INVERTED = {false};
        public static final TrapezoidProfile.Constraints PID_CONSTRAINTS = new TrapezoidProfile.Constraints(1, 0.2); // TODO: Verify
        public static final ProfiledPIDController ARM_PID = new ProfiledPIDController(1, 0, 0, PID_CONSTRAINTS); // TODO: Characterize
        public static final ArmFeedforward ARM_FF = new ArmFeedforward(0, 0, 0); // TODO: Characterize
        public static final double HIGH_SETPOINT = Units.degreesToRadians(80);
        public static final double MEDIUM_SETPOINT = Units.degreesToRadians(50);
        public static final double INTAKE_SETPOINT = Units.degreesToRadians(15);
        public static final double STOW_SETPOINT = Units.degreesToRadians(0);

        public static final PivotingArmConstants ARM_CONSTANTS = new PivotingArmConstants(
                GEARING, SOFT_LIMITS, ZERO_OFFSET, ENCODER_CPR, SWITCH_PORTS, SWITCHES_INVERTED, ARM_PID, ARM_FF);
    }
}
