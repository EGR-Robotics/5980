package frc.robot.library;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.ELEVATOR;

/**
 * Generic motor class using relative encoders and closed loop controller
 * 
 * @param canId
 * @param motorType Brushless or brushed
 * @param motorConfig
 * @param motorARBF Arbitrary feedfoward velocity
 */
public class GenericMotor {
    public SparkMax m_motor;

    private SparkClosedLoopController m_PIDController;
    private RelativeEncoder m_encoder;

    private double m_motor_arb_f;
    private double m_target_encoder_pos;

    public GenericMotor(int canID, SparkLowLevel.MotorType motorType, SparkBaseConfig motorConfig, double motorARBF) {
        m_motor = new SparkMax(canID, motorType);

        m_motor.configure(
                motorConfig,
                SparkMax.ResetMode.kResetSafeParameters,
                SparkMax.PersistMode.kPersistParameters);

        m_PIDController = m_motor.getClosedLoopController();
        m_encoder = m_motor.getEncoder();

        m_encoder.setPosition(0);
        m_target_encoder_pos = m_encoder.getPosition();

        m_motor_arb_f = motorARBF;
    }

    /**
     * Sets target position for motor
     * 
     * @param pos Relative encoder position
     */
    public void setEncoderPosition(double pos) {
        m_PIDController.setReference(
                pos,
                ControlType.kMAXMotionPositionControl,
                ClosedLoopSlot.kSlot0,
                m_motor_arb_f,
                ArbFFUnits.kVoltage);
    }

    /**
     * @return Encoder position
     */
    public double getEncoderPosition() {
        return m_encoder.getPosition();
    }

    /**
     * Sets voltage percent to apply to motor
     * 
     * @param percentOutput
     */
    public void setSpeed(double percentOutput) {
        m_motor.set(percentOutput);
    }

    /**
     * Stop motor
     */
    public void stop() {
        m_motor.stopMotor();
    }

    /** 
     * @return Boolean whether motor has reached target encoder position
     */
    public boolean isAtTarget() {
        return ELEVATOR.MAX_MOTION_ALLOWED_ERROR_PERCENT >= Helpers.percentError(m_target_encoder_pos,
                m_encoder.getPosition());
    }
}
