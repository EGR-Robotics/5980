package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.Servo;

public class AlgaeSubsystem implements Subsystem {
    private SparkMax armMotor;

    private SparkMax elevatorMotor;
    private RelativeEncoder elevatorEncoder;
    private SparkClosedLoopController elevatorController;

    // Limit switches for elevator
    private DigitalInput upElevatorLimitSwitch;
    private DigitalInput lowerElevatorLimitSwitch;
    private double curElevatorPos;

    public AlgaeSubsystem() {
        // Initialize arm motors
        armMotor = new SparkMax(-1, MotorType.kBrushless);

        // Initialize elevator motor
        elevatorMotor = new SparkMax(2, MotorType.kBrushless);
        elevatorEncoder = elevatorMotor.getEncoder();

        elevatorController = elevatorMotor.getClosedLoopController();
        elevatorEncoder.setPosition(0);
    
        curElevatorPos = elevatorEncoder.getPosition();

        // Create configuration for sparks
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12);
        
        /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
        config
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // Set PID values for position control
            .p(0.1)
            .outputRange(-1, 1)
            .maxMotion
            // Set MAXMotion parameters for position control
            .maxVelocity(2000)
            .maxAcceleration(10000)
            .allowedClosedLoopError(0.25);

        // Configure motors to use the config

        armMotor.configure(
            config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        elevatorMotor.configure(
            config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    public void goToLevel1() {
        elevatorController.setReference(5, ControlType.kMAXMotionPositionControl);
        curElevatorPos = elevatorEncoder.getPosition();
    }

    public void moveArm() {
        armMotor.set(-0.3);
    }

    public void moveElevator(boolean up) {
        if(up) {
            elevatorMotor.set(1);
        }
        else {
            elevatorMotor.set(-1);
        }

        curElevatorPos = elevatorEncoder.getPosition();
    }

    public Command goToLevel1Command() {
        return run(() -> goToLevel1());
    }   

    public Command moveArmCommand() {
        return run(() -> moveArm());
    }

    public Command moveElevatorUpCommand() {
        return run(() -> moveElevator(true));
    }

    public Command moveElevatorDownCommand() {
        return run(() -> moveElevator(false));
    }
    
    public Command holdElevatorPositionCommand() {
        return run(() -> elevatorController.setReference(curElevatorPos, ControlType.kMAXMotionPositionControl));
    }

    public Command stopArm() {
        return run(() -> armMotor.set(0));
    }
}
