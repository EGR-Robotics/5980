package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Constants {
        public class SCORING {
                public static final double ELEVATOR_SPEED = 0.5;
                public static final double ARM_SPEED = 0.15;

                public static final double L4_ELEVATOR_POSITION = -160;
                public static final double L4_ARM_POSITION = -17;

                public static final Distance L4_ELEVATOR_HEIGHT = Units.Feet.of(-1.1);

                public static final Angle L4_ELEVATOR_ROT = Units.Rotations.of(-60);
                public static final Angle L4_ARM_ANGLE = Units.Rotations.of(-4.5);

                public static final Distance L3_ELEVATOR_HEIGHT = Units.Feet.of(-2.5);
                public static final Angle L3_ARM_ANGLE = Units.Rotations.of(-3.0);

                public static final Distance L2_ELEVATOR_HEIGHT = Units.Feet.of(-4.5);
                public static final Angle L2_ARM_ANGLE = Units.Rotations.of(-3.0);

                public static final Distance L1_ELEVATOR_HEIGHT = Units.Feet.of(-4.5);
                public static final Angle L1_ARM_ANGLE = Units.Rotations.of(-3.0);
        }

        public class APRIL_TAGS {
                public static int REEF_AB_TAGID;
                public static int REEF_CD_TAGID;
                public static int REEF_EF_TAGID;
                public static int REEF_GH_TAGID;
                public static int REEF_IJ_TAGID;
                public static int REEF_KL_TAGID;

                public static int CORAL_STATION_LEFT_TAGID;
                public static int CORAL_STATION_RIGHT_TAGID;

                // below values are in meters
                public static final double INSIDE_REEF_ZONE_THRESHOLD = 1.6;
                public static final double AUTO_ADJUST_THRESHOLD = 1.8;

                private static final double CORAL_STATION_OFFSET_HORIZONTAL = 0.3;
                private static final double CORAL_STATION_OFFSET_VERTICAL = 0.3;
                public static Translation2d CORAL_STATION_LEFT_OFFSET;
                public static Translation2d CORAL_STATION_RIGHT_OFFSET;

                public static void update(Alliance alliance) {
                        REEF_AB_TAGID = alliance == Alliance.Blue ? 18 : 7;
                        REEF_CD_TAGID = alliance == Alliance.Blue ? 19 : 8;
                        REEF_EF_TAGID = alliance == Alliance.Blue ? 20 : 9;
                        REEF_GH_TAGID = alliance == Alliance.Blue ? 21 : 10;
                        REEF_IJ_TAGID = alliance == Alliance.Blue ? 22 : 11;
                        REEF_KL_TAGID = alliance == Alliance.Blue ? 17 : 6;

                        CORAL_STATION_LEFT_TAGID = alliance == Alliance.Blue ? 13 : 1;
                        CORAL_STATION_RIGHT_TAGID = alliance == Alliance.Blue ? 12 : 2;

                        CORAL_STATION_LEFT_OFFSET = alliance == Alliance.Blue
                                        ? new Translation2d(CORAL_STATION_OFFSET_HORIZONTAL,
                                                        -CORAL_STATION_OFFSET_VERTICAL)
                                        : new Translation2d(CORAL_STATION_OFFSET_HORIZONTAL,
                                                        CORAL_STATION_OFFSET_VERTICAL);
                        CORAL_STATION_RIGHT_OFFSET = alliance == Alliance.Blue
                                        ? new Translation2d(-CORAL_STATION_OFFSET_HORIZONTAL,
                                                        -CORAL_STATION_OFFSET_VERTICAL)
                                        : new Translation2d(CORAL_STATION_OFFSET_HORIZONTAL,
                                                        CORAL_STATION_OFFSET_VERTICAL);
                }
        }

        public class ARM {
                public static final int CAN_ID = 20;

                public static final SparkBaseConfig MOTOR_CONFIG = new SparkMaxConfig()
                                .idleMode(IdleMode.kBrake)
                                .inverted(false)
                                .openLoopRampRate(.25)
                                .smartCurrentLimit(40, 40)
                                .voltageCompensation(12);

                public static final double AXIS_MAX_SPEED = 0.25;
                public static final double MOTOR_ARB_F = 0.05;

                public static final double MOTOR_P = 0.5;
                public static final double MOTOR_I = 0;
                public static final double MOTOR_D = 0;
                public static final double MOTOR_F = 0;

                public static final ClosedLoopConfig CLOSED_LOOP_CONFIG = MOTOR_CONFIG.closedLoop
                                .pidf(MOTOR_P, MOTOR_I, MOTOR_D, MOTOR_F)
                                .outputRange(-1, 1);

                public static final double MAX_MOTION_ALLOWED_ERROR_PERCENT = 0.03;

                public static final MAXMotionConfig MAX_MOTION_CONFIG = CLOSED_LOOP_CONFIG.maxMotion
                                .allowedClosedLoopError(MAX_MOTION_ALLOWED_ERROR_PERCENT)
                                .maxAcceleration(1200)
                                .maxVelocity(600);

                public static final double GEAR_RATIO = 20;
                public static final Distance OUTPUT_PULLEY_DIAMETER = Units.Inches.of(
                                2.256);

                public static final Distance HTD5_PULLEY_PITCH = Units.Millimeters.of(5);
                public static final double OUTPUT_PULLEY_NUMBER_OF_TEETH = 28;
                public static final Distance OUTPUT_PULLEY_CIRCUMFERENCE = HTD5_PULLEY_PITCH
                                .times(OUTPUT_PULLEY_NUMBER_OF_TEETH);

                public static final double ENCODER_UPPER_LIMIT = 0; // TODO: Update
                public static final double ENCODER_LOWER_LIMIT = 0; // TODO: Update
        }

        public class ELEVATOR {
                public static final int CAN_ID = 15;

                public static final SparkBaseConfig MOTOR_CONFIG = new SparkMaxConfig()
                                .idleMode(IdleMode.kBrake)
                                .inverted(false)
                                .openLoopRampRate(.3)
                                .smartCurrentLimit(40, 40)
                                .voltageCompensation(12);

                public static final double AXIS_MAX_SPEED = 0.25;
                public static final double MOTOR_ARB_F = 0.1;

                public static final double MOTOR_P = 30;
                public static final double MOTOR_I = 0;
                public static final double MOTOR_D = 0;
                public static final double MOTOR_F = 0;

                public static final ClosedLoopConfig CLOSED_LOOP_CONFIG = MOTOR_CONFIG.closedLoop
                                .pidf(MOTOR_P, MOTOR_I, MOTOR_D, MOTOR_F)
                                .outputRange(-1, 1);

                public static final double MAX_MOTION_ALLOWED_ERROR_PERCENT = 0.03;

                public static final MAXMotionConfig MAX_MOTION_CONFIG = CLOSED_LOOP_CONFIG.maxMotion
                                .allowedClosedLoopError(MAX_MOTION_ALLOWED_ERROR_PERCENT)
                                .maxAcceleration(45000)
                                .maxVelocity(22500);

                public static final double GEAR_RATIO = 15;
                public static final Distance OUTPUT_PULLEY_DIAMETER = Units.Inches.of(
                                2.256);

                public static final Distance HTD5_PULLEY_PITCH = Units.Millimeters.of(5);
                public static final double OUTPUT_PULLEY_NUMBER_OF_TEETH = 28;
                public static final Distance OUTPUT_PULLEY_CIRCUMFERENCE = HTD5_PULLEY_PITCH
                                .times(OUTPUT_PULLEY_NUMBER_OF_TEETH);

                public static final double ELEVATOR_UPPER_LIMIT = -184;
                public static final double ELEVATOR_LOWER_LIMIT = 1;

                public static final double ELEVATOR_SAFE_POS = -4;
        }

        public class ALGAE {
                public static final int ARM_CAN_ID = 17;
                public static final int VORTEX_CAN_ID = 16;

                public static final SparkBaseConfig MOTOR_CONFIG = new SparkMaxConfig()
                                .idleMode(IdleMode.kBrake)
                                .inverted(false)
                                .openLoopRampRate(.3)
                                .smartCurrentLimit(40, 40)
                                .voltageCompensation(12);

                public static final double AXIS_MAX_SPEED = 0.25;
                public static final double MOTOR_ARB_F = 0.1;

                public static final double MOTOR_P = 0;
                public static final double MOTOR_I = 0;
                public static final double MOTOR_D = 0;
                public static final double MOTOR_F = 0;

                public static final ClosedLoopConfig CLOSED_LOOP_CONFIG = MOTOR_CONFIG.closedLoop
                                .pidf(MOTOR_P, MOTOR_I, MOTOR_D, MOTOR_F)
                                .outputRange(-1, 1);

                public static final double MAX_MOTION_ALLOWED_ERROR_PERCENT = 0.03;

                public static final MAXMotionConfig MAX_MOTION_CONFIG = CLOSED_LOOP_CONFIG.maxMotion
                                .allowedClosedLoopError(MAX_MOTION_ALLOWED_ERROR_PERCENT)
                                .maxAcceleration(100)
                                .maxVelocity(5001);

                public static final double GEAR_RATIO = 15; // TODO: Update gear ratio

                public static final double ENCODER_UPPER_LIMIT = 0; // TODO: Update
                public static final double ENCODER_LOWER_LIMIT = 0; // TODO: Update

                public static final double ARM_RAISE_SPEED = 0.25;
                public static final double ARM_LOWER_SPEED = -0.25;

                public static final double INTAKE_SPEED = 0.15;
                public static final double OUTAKE_SPEED = -0.3;
        }

        public class ACTUATOR {
                public static final int PWM_PORT = 9;
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
}
