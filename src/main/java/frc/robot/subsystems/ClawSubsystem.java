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

public class ClawSubsystem implements Subsystem {
    private SparkMax motor;
    private RelativeEncoder encoder;
    private SparkClosedLoopController controller;

    private Servo push;

    private DigitalInput limitSwitch;

    private double curPos;

    public ClawSubsystem() {
        push = new Servo(1);
        push.setAlwaysHighMode();

        limitSwitch = new DigitalInput(1);

        motor = new SparkMax(2, MotorType.kBrushless);
        encoder = motor.getEncoder();

        controller = motor.getClosedLoopController();
        encoder.setPosition(0);
    
        curPos = encoder.getPosition();


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

        motor.configure(
            config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    public void goToLevel1() {
        controller.setReference(5, ControlType.kMAXMotionPositionControl);
        curPos = encoder.getPosition();
    }

    public void move(boolean up) {
        if(up) {
            motor.set(0.3);
        }
        else {
            motor.set(-0.3);
        }

        curPos = encoder.getPosition();
    }

    public void drop() {
        System.out.println(push.getAngle());
        push.setAngle(0);
    }

    public Command level1() {
        return run(() -> goToLevel1());
    }   

    public Command moveUp() {
        return run(() -> move(true));
    }

    public Command moveDown() {
        return run(() -> move(false));
    }

    public Command hold() {
        return run(() -> controller.setReference(curPos, ControlType.kMAXMotionPositionControl));
    }

    public Command dropCommand() {
        return run(() -> drop());
    }
}
