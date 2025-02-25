package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

public class Constants {
    public class ELEVATOR {
        public static final int CAN_ID = 15;

        public static final SparkBaseConfig MOTOR_CONFIG = new SparkMaxConfig()
                .idleMode(IdleMode.kBrake)
                .inverted(false)
                .openLoopRampRate(.25)
                .smartCurrentLimit(40, 40)
                .voltageCompensation(12);

        public static final double AXIS_MAX_SPEED = 0.25;
        public static final double MOTOR_ARB_F = 0.05;

        public static final double GEAR_RATIO = 3.2142857143;
        public static final Distance OUTPUT_PULLEY_DIAMETER = Units.Inches.of(
                2.256);

        public static final Distance HTD5_PULLEY_PITCH = Units.Millimeters.of(5);
        public static final double OUTPUT_PULLEY_NUMBER_OF_TEETH = 28;
        public static final Distance OUTPUT_PULLEY_CIRCUMFERENCE = HTD5_PULLEY_PITCH
                .times(OUTPUT_PULLEY_NUMBER_OF_TEETH);
    }
}
