package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.generated.TunerConstants;

public class Constants {
    /*
     * Generic class to create REV SparkMax based motor configurations.
     */
    public class GENERIC_MOTOR_CONFIG {
        public static final int CAN_ID = -1;
        public static final double SPEED = 1;

        public static final SparkBaseConfig MOTOR_CONFIG = new SparkMaxConfig()
                .idleMode(IdleMode.kBrake)
                .inverted(false)
                .openLoopRampRate(.3)
                .smartCurrentLimit(40, 40)
                .voltageCompensation(12);

        public static final double MOTOR_ARB_F = 0.25;

        public static final double MOTOR_P = 0;
        public static final double MOTOR_I = 0;
        public static final double MOTOR_D = 0;
        public static final double MOTOR_F = 0;

        public static final ClosedLoopConfig CLOSED_LOOP_CONFIG = MOTOR_CONFIG.closedLoop
                .pidf(MOTOR_P, MOTOR_I, MOTOR_D, MOTOR_F)
                .outputRange(-1, 1);

        public static final double MAX_MOTION_ALLOWED_ERROR_PERCENT = 0.03;

        // Sensitivity of joystick
        public static final double DEADBAND = 0.1;
    }

    public class LIMELIGHT {
        public static final String LIMELIGHT_NAME_1 = "limelight";
    }

    public class SCORING {
        public static final double L4_ELEVATOR_POSITION = -44;
        public static final double L4_ARM_POSITION = -23;

        public static final double L3_ELEVATOR_POSITION = 0;
        public static final double L3_ARM_POSITION = -21.5;

        public static final double L2_ELEVATOR_POSITION = 108.54;
        public static final double L2_ARM_POSITION = -21.5;

        public static final double L1_ELEVATOR_POSITION = 0;
        public static final double L1_ARM_POSITION = -12;
    }

    public class ARM extends GENERIC_MOTOR_CONFIG {
        public static final int CAN_ID = 20;
        public static final double SPEED = 0.3;

        public static final double MOTOR_P = 1.5;
        public static final double MOTOR_I = 0.1;
        public static final double MOTOR_D = 1;

        public static final MAXMotionConfig MAX_MOTION_CONFIG = CLOSED_LOOP_CONFIG.maxMotion
                .allowedClosedLoopError(MAX_MOTION_ALLOWED_ERROR_PERCENT)
                .maxAcceleration(2000)
                .maxVelocity(1000);

        // public static final double ENCODER_UPPER_LIMIT = -18; // TODO: Update
        // public static final double ENCODER_LOWER_LIMIT = 0.6; // TODO: Update
    }

    public class ELEVATOR extends GENERIC_MOTOR_CONFIG {
        public static final int CAN_ID = 15;
        public static final double SPEED = 0.8;

        public static final double MOTOR_P = 0.01;
        
        public static final MAXMotionConfig MAX_MOTION_CONFIG = CLOSED_LOOP_CONFIG.maxMotion
                .allowedClosedLoopError(MAX_MOTION_ALLOWED_ERROR_PERCENT)
                .maxAcceleration(60000)
                .maxVelocity(30000);
        
        // public static final double ELEVATOR_UPPER_LIMIT = -184;
        // public static final double ELEVATOR_LOWER_LIMIT = 1;

        public static final double ELEVATOR_SAFE_POS = -4;

        public static final double SLOW_DOWN_POSITION = -50;
        public static final double DRIVE_SLOW_DOWN_RATE = 0.4;
        public static final double DRIVE_ANGULAR_SLOW_DOWN_RATE = 0.4;

    }

    public class ALGAE {
        public class INTAKE {
            public static final int CAN_ID = 16;

            public static final double INTAKE_SPEED = 0.15;
            public static final double OUTAKE_SPEED = -0.3;
        }

        public class BAR extends GENERIC_MOTOR_CONFIG {
            public static final int CAN_ID = 17;
            
            public static final double RAISE_SPEED = 0.25;
            public static final double LOWER_SPEED = -0.25;

            public static final MAXMotionConfig MAX_MOTION_CONFIG = CLOSED_LOOP_CONFIG.maxMotion
                .allowedClosedLoopError(MAX_MOTION_ALLOWED_ERROR_PERCENT)
                .maxAcceleration(1000)
                .maxVelocity(5001);

            // public static final double ENCODER_UPPER_LIMIT = 0; // TODO: Update
            // public static final double ENCODER_LOWER_LIMIT = 0; // TODO: Update
        }
    }

    public class ACTUATOR {
        public static final int PWM_PORT = 9;
    }

    public class CLIMBER extends GENERIC_MOTOR_CONFIG {
        public static final int CAN_ID = 13;
        public static final double SPEED = 0.3;

        // public static final SparkBaseConfig MOTOR_CONFIG = new SparkMaxConfig().idleMode(IdleMode.kBrake)
        //         .smartCurrentLimit(40).voltageCompensation(12);

        // public static ClosedLoopConfig CLOSED_LOOP_CONFIG = MOTOR_CONFIG.closedLoop
        //         .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        //         // Set PID values for position control
        //         .p(0.1)
        //         .outputRange(-1, 1);
        
        // public static final double MAX_MOTION_ALLOWED_ERROR_PERCENT = 0.25;
                
        public static final MAXMotionConfig MAX_MOTION_CONFIG = CLOSED_LOOP_CONFIG.maxMotion
            // Set MAXMotion parameters for position control
            .maxVelocity(2000)
            .maxAcceleration(10000)
            .allowedClosedLoopError(MAX_MOTION_ALLOWED_ERROR_PERCENT);


    }

    public class DRIVETRAIN {
        public static final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        public static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

        public static final double DRIVE_DEADBAND = 0.1;
        public static final double ROTATION_DEADBAND = 0.05;

        // Factor to multiply speed by when in slow mode
        public static final double SLOW_DOWN_RATE = 0.2;
    }
}
