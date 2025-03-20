package frc.robot.library;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

/**
 * 
 */
public class GenericMotorConstants {
        public static final int CAN_ID = -1;

        public static final double MOTOR_ARB_F = 0.05;
        public static final double MAX_MOTION_ALLOWED_ERROR_PERCENT = 0.03;

        public static final double MOTOR_P = 0.1;
        public static final double MOTOR_I = 0;
        public static final double MOTOR_D = 0;
        public static final double MOTOR_F = 0;

        public static final SparkBaseConfig MOTOR_CONFIG = gen();

        public static SparkBaseConfig gen() {
                return gen(5600, 5000);
        }

        public static SparkBaseConfig gen(double maxVelocity, double maxAcceleration) {
                SparkBaseConfig motorConfig = new SparkMaxConfig()
                                .idleMode(IdleMode.kBrake)
                                .inverted(false)
                                .openLoopRampRate(.25)
                                .smartCurrentLimit(40, 40)
                                .voltageCompensation(12);

                motorConfig
                        .closedLoop
                        .pidf(MOTOR_P, MOTOR_I, MOTOR_D, MOTOR_F)
                        .outputRange(-1, 1)
                        .maxMotion
                        .allowedClosedLoopError(MAX_MOTION_ALLOWED_ERROR_PERCENT)
                        .maxAcceleration(maxAcceleration)
                        .maxVelocity(maxVelocity);

                return motorConfig;
        }

        public static final double DEADBAND = 0.1;
}
