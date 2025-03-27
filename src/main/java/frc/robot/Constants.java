package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.generated.TunerConstants;

public class Constants {
        public class ARM {
                public static final int CAN_ID = 20;

                public static final double MOTOR_ARB_F = 0.05;
                public static final double MAX_MOTION_ALLOWED_ERROR_PERCENT = 0.03;

                public static final double MOTOR_P = 1.5;
                public static final double MOTOR_I = 0;
                public static final double MOTOR_D = 1;
                public static final double MOTOR_F = 0;

                public static final SparkBaseConfig MOTOR_CONFIG = new SparkMaxConfig()
                                .idleMode(IdleMode.kBrake)
                                .inverted(false)
                                .openLoopRampRate(.25)
                                .smartCurrentLimit(40, 40)
                                .voltageCompensation(12);

                public static final ClosedLoopConfig CLOSED_LOOP_CONFIG = MOTOR_CONFIG.closedLoop
                                .pidf(MOTOR_P, MOTOR_I, MOTOR_D, MOTOR_F)
                                .outputRange(-1, 1);

                public static final MAXMotionConfig MAX_MOTION_CONFIG = CLOSED_LOOP_CONFIG.maxMotion
                                .allowedClosedLoopError(MAX_MOTION_ALLOWED_ERROR_PERCENT)
                                .maxAcceleration(2680)
                                .maxVelocity(5600);

                public static final double ENCODER_UPPER_LIMIT = -18; // TODO: Update
                public static final double ENCODER_LOWER_LIMIT = 0.6; // TODO: Update

                public static final double DEADBAND = 0.1;
        }

        public class ELEVATOR {
                public static final int CAN_ID = 15;

                public static final double MOTOR_ARB_F = 0.1;
                public static final double MAX_MOTION_ALLOWED_ERROR_PERCENT = 0.03;

                public static final double MOTOR_P = 1.75;
                public static final double MOTOR_I = 0;
                public static final double MOTOR_D = 2.75;
                public static final double MOTOR_F = 0;

                public static final SparkBaseConfig MOTOR_CONFIG = new SparkMaxConfig()
                                .idleMode(IdleMode.kBrake)
                                .inverted(false)
                                .openLoopRampRate(.3)
                                .smartCurrentLimit(40, 40)
                                .voltageCompensation(12);

                public static final ClosedLoopConfig CLOSED_LOOP_CONFIG = MOTOR_CONFIG.closedLoop
                                .pidf(MOTOR_P, MOTOR_I, MOTOR_D, MOTOR_F)
                                .outputRange(-1, 1);

                public static final MAXMotionConfig MAX_MOTION_CONFIG = CLOSED_LOOP_CONFIG.maxMotion
                                .allowedClosedLoopError(MAX_MOTION_ALLOWED_ERROR_PERCENT)
                                .maxAcceleration(6000)
                                .maxVelocity(5600);

                public static final double ELEVATOR_UPPER_LIMIT = -184;
                public static final double ELEVATOR_LOWER_LIMIT = 1;

                public static final double ELEVATOR_SAFE_POS = 8;

                public static final double DEADBAND = 0.1;

                public static final double SLOW_DOWN_POSITION = -50;
                public static final double DRIVE_SLOW_DOWN_RATE = 0.4;
                public static final double DRIVE_ANGULAR_SLOW_DOWN_RATE = 0.4;
        }

        public class ALGAE {
                public static final int ARM_CAN_ID = 17;
                public static final int INTAKE_CAN_ID = 16;

                public static final SparkBaseConfig ARM_MOTOR_CONFIG = new SparkMaxConfig()
                                .idleMode(IdleMode.kBrake)
                                .inverted(false)
                                .openLoopRampRate(.25)
                                .smartCurrentLimit(40, 40)
                                .voltageCompensation(12);

                public static final double MOTOR_ARB_F = 0.1;

                public static final double MOTOR_P = 0;
                public static final double MOTOR_I = 0;
                public static final double MOTOR_D = 0;
                public static final double MOTOR_F = 0;

                public static final ClosedLoopConfig CLOSED_LOOP_CONFIG = ARM_MOTOR_CONFIG.closedLoop
                                .pidf(MOTOR_P, MOTOR_I, MOTOR_D, MOTOR_F)
                                .outputRange(-1, 1);

                public static final double MAX_MOTION_ALLOWED_ERROR_PERCENT = 0.03;

                public static final MAXMotionConfig MAX_MOTION_CONFIG = CLOSED_LOOP_CONFIG.maxMotion
                                .allowedClosedLoopError(MAX_MOTION_ALLOWED_ERROR_PERCENT)
                                .maxAcceleration(10000)
                                .maxVelocity(5000);

                public static final double ARM_RAISE_SPEED = 0.25;
                public static final double ARM_LOWER_SPEED = -0.25;

                public static final double INTAKE_SPEED = -0.15;
                public static final double OUTAKE_SPEED = 0.3;
        }

        public class ACTUATOR {
                public static final int PWM_PORT = -1;
        }

        public class CLIMBER {
                public static final int CAN_ID = 13;

                public static final SparkBaseConfig MOTOR_CONFIG = new SparkMaxConfig()
                                .idleMode(IdleMode.kBrake)
                                .inverted(false)
                                .openLoopRampRate(.3)
                                .smartCurrentLimit(40, 40)
                                .voltageCompensation(12);

                public static final double SPEED = 0.3;

        }

        public class CANDLE {
                public static int CAN_ID = 23;

                public final static int LED_COLUMNS = 33;
                public final static int LED_ROWS = 8;
                public final static int STRIP_START = LED_COLUMNS * LED_ROWS;
                public final static int STRIP_LENGTH = 48;

                public final static int LED_TOTAL = LED_COLUMNS * LED_ROWS + STRIP_LENGTH;
        }

        public class LIMELIGHT {
                public static final String LIMELIGHT_NAME_1 = "limelight";

                public static final double TA_TARGET_DISTANCE = 15;
                public static final double TA_TARGET_DISTANCE_ALLOWED_ERROR = 0.15;
        }

         
        public class SCORING {
                public static final double ELEVATOR_SPEED = 0.8;
                public static final double ARM_SPEED = 0.3;

                public static final double L4_ELEVATOR_POSITION = -44;
                public static final double L4_ARM_POSITION = -22;
                public static final double L4_ARM_LOWER_AMOUNT = 8;

                public static final double L3_ELEVATOR_POSITION = 0;
                public static final double L3_ARM_POSITION = -21.5;

                public static final double L2_ELEVATOR_POSITION = 21;
                public static final double L2_ARM_POSITION = -19;

                public static final double L1_ELEVATOR_POSITION = 0;
                public static final double L1_ARM_POSITION = -18;

                public static final double ELEVATOR_PICKUP_POSITION = 22;

        }

        public class DRIVE {
                public static final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
                public static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

                public static final double DRIVE_DEADBAND = 0.1;
                public static final double ROTATION_DEADBAND = 0.05;

                public static final double SLOW_DOWN_RATE = 0.2;
        }

        public class CAN_RANGE {
                public static final int CAN_ID = -1;
                public static final String CAN_BUS = "canivore";

                public static final CANrangeConfiguration CONFIG = new CANrangeConfiguration();

                public static final double TARGET_DISTANCE = 3; // distance from sensor to coral in meters
        }
}
