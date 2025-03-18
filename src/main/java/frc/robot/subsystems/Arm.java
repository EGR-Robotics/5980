package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ARM;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Helpers;

public class Arm extends SubsystemBase {
    private SparkMax m_motor;

    private SparkClosedLoopController m_PIDController;
    private RelativeEncoder m_encoder;

    private double targetEncoderPos;

    private MutAngle m_targetRotations = Units.Rotations.mutable(Double.NaN);
    private MutAngularVelocity m_currentAngularVelocityHolder = Units.RPM.mutable(
            Double.NaN);

    public Arm() {
        m_motor = new SparkMax(
                ARM.CAN_ID,
                MotorType.kBrushless);

        m_motor.configure(
                ARM.MOTOR_CONFIG,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        m_PIDController = m_motor.getClosedLoopController();

        m_encoder = m_motor.getEncoder();
        m_encoder.setPosition(0);
    }

    public void setSpeed(double percentOutput) {
        // System.out.println(percentOutput);

        // if (m_encoder.getPosition() <= ARM.ENCODER_UPPER_LIMIT && percentOutput < 0)
        // {
        // System.out.println();
        // return;
        // }
        // else if (m_encoder.getPosition() >= ARM.ENCODER_LOWER_LIMIT && percentOutput
        // > 0)
        // return;

        m_motor.set(percentOutput);
        m_targetRotations.mut_replace(Double.NaN, Units.Rotations);

        System.out.println("Arm position: " + m_encoder.getPosition());
    }

    public double getPosition() {
        return m_encoder.getPosition();
    }

    public void setPosition(double pos) {
        targetEncoderPos = pos;

        m_PIDController.setReference(
                pos,
                ControlType.kMAXMotionPositionControl,
                ClosedLoopSlot.kSlot0,
                ARM.MOTOR_ARB_F,
                ArbFFUnits.kVoltage);
    }

    public void stop() {
        m_motor.stopMotor();
        m_targetRotations.mut_replace(Double.NaN, Units.Rotations);
    }

    public void setTargetRotations(Angle targetRotations) {
        m_targetRotations.mut_replace(targetRotations);
        m_PIDController.setReference(
                m_targetRotations.in(Units.Rotations),
                ControlType.kMAXMotionPositionControl,
                ClosedLoopSlot.kSlot0,
                ARM.MOTOR_ARB_F,
                ArbFFUnits.kVoltage);
    }

    public AngularVelocity getVelocity() {
        m_currentAngularVelocityHolder.mut_replace(
                m_encoder.getVelocity(),
                Units.RPM);
        return m_currentAngularVelocityHolder;
    }

    public boolean isAtTarget() {
        return ELEVATOR.MAX_MOTION_ALLOWED_ERROR_PERCENT > Helpers.percentError(targetEncoderPos,
                m_encoder.getPosition());
    }

    public Command hold() {
        return run(() -> m_PIDController.setReference(
                m_encoder.getPosition(),
                ControlType.kMAXMotionPositionControl,
                ClosedLoopSlot.kSlot0,
                ARM.MOTOR_ARB_F,
                ArbFFUnits.kVoltage));
    }

    @Override
    public void periodic() {
    }
}
