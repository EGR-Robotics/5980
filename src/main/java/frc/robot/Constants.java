package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class Constants {
        public class SCORING {
                public static final double ELEVATOR_SPEED = 1;
                public static final double ARM_SPEED = 0.15;

                public static final Distance L4_ELEVATOR_HEIGHT = Units.Feet.of(-0.5);
                public static final Angle L4_ARM_ANGLE = Units.Rotations.of(-24);
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

                public static final double MOTOR_P = 5;
                public static final double MOTOR_I = 0;
                public static final double MOTOR_D = 0;
                public static final double MOTOR_F = 0;

                public static final ClosedLoopConfig CLOSED_LOOP_CONFIG = MOTOR_CONFIG.closedLoop
                                .pidf(MOTOR_P, MOTOR_I, MOTOR_D, MOTOR_F)
                                .outputRange(-1, 1);

                public static final double MAX_MOTION_ALLOWED_ERROR_PERCENT = 0.03;

                public static final MAXMotionConfig MAX_MOTION_CONFIG = CLOSED_LOOP_CONFIG.maxMotion
                                .allowedClosedLoopError(MAX_MOTION_ALLOWED_ERROR_PERCENT)
                                .maxAcceleration(1000)
                                .maxVelocity(3000);

                public static final double GEAR_RATIO = 20;
                public static final Distance OUTPUT_PULLEY_DIAMETER = Units.Inches.of(
                                2.256);

                public static final Distance HTD5_PULLEY_PITCH = Units.Millimeters.of(5);
                public static final double OUTPUT_PULLEY_NUMBER_OF_TEETH = 28;
                public static final Distance OUTPUT_PULLEY_CIRCUMFERENCE = HTD5_PULLEY_PITCH
                                .times(OUTPUT_PULLEY_NUMBER_OF_TEETH);
        }

        public class ELEVATOR {
                public static final int CAN_ID = 15;

                public static final SparkBaseConfig MOTOR_CONFIG = new SparkMaxConfig()
                                .idleMode(IdleMode.kCoast)
                                .inverted(false)
                                .openLoopRampRate(.25)
                                .smartCurrentLimit(40, 40)
                                .voltageCompensation(12);

                public static final double AXIS_MAX_SPEED = 0.25;
                public static final double MOTOR_ARB_F = 0.05;

                public static final double MOTOR_P = 7;
                public static final double MOTOR_I = 0;
                public static final double MOTOR_D = 0;
                public static final double MOTOR_F = 0;

                public static final ClosedLoopConfig CLOSED_LOOP_CONFIG = MOTOR_CONFIG.closedLoop
                                .pidf(MOTOR_P, MOTOR_I, MOTOR_D, MOTOR_F)
                                .outputRange(-1, 1);

                public static final double MAX_MOTION_ALLOWED_ERROR_PERCENT = 0.03;

                public static final MAXMotionConfig MAX_MOTION_CONFIG = CLOSED_LOOP_CONFIG.maxMotion
                                .allowedClosedLoopError(MAX_MOTION_ALLOWED_ERROR_PERCENT)
                                .maxAcceleration(1000)
                                .maxVelocity(5000);

                public static final double GEAR_RATIO = 25;
                public static final Distance OUTPUT_PULLEY_DIAMETER = Units.Inches.of(
                                2.256);

                public static final Distance HTD5_PULLEY_PITCH = Units.Millimeters.of(5);
                public static final double OUTPUT_PULLEY_NUMBER_OF_TEETH = 28;
                public static final Distance OUTPUT_PULLEY_CIRCUMFERENCE = HTD5_PULLEY_PITCH
                                .times(OUTPUT_PULLEY_NUMBER_OF_TEETH);

                public static final Distance ELEVATOR_SAFE_HEIGHT = Units.Feet.of(-0.7);
        }

        public class ACTUATOR {
                public static final int PWM_PORT = 9;

                // Bounds for actuator extensions
                public static double MINIMUM = 300;
                public static double MAXIMUM = 1500;
        }
}
