// package frc.robot.subsystems;

// import com.revrobotics.AbsoluteEncoder;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
// import com.revrobotics.spark.config.LimitSwitchConfig.Type;
// import com.revrobotics.spark.config.SparkBaseConfig;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.ClosedLoopSlot;
// import com.revrobotics.spark.SparkClosedLoopController;
// import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
// import com.revrobotics.spark.SparkLimitSwitch;

// import edu.wpi.first.math.filter.Debouncer;


// import edu.wpi.first.units.Units;
// import edu.wpi.first.units.measure.AngularVelocity;
// import edu.wpi.first.units.measure.Distance;
// import edu.wpi.first.units.measure.MutAngle;
// import edu.wpi.first.units.measure.MutAngularVelocity;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Subsystem;

// import com.revrobotics.spark.SparkBase.ResetMode;

// import edu.wpi.first.wpilibj.Servo;

// public class ClawSubsystem implements Subsystem {
//     // private DigitalInput m_hall_effect;
//     // private Debouncer m_debouncer;

//     private SparkMax m_motor_elevator;

//     private SparkClosedLoopController m_elevator_PIDController;
//     private RelativeEncoder m_elevator_encoder;

//     private MutAngle m_elevator_targetRotations = Units.Rotations.mutable(Double.NaN);
//     private MutAngularVelocity m_currentAngularVelocityHolder = Units.RPM.mutable(
//             Double.NaN);
//     private MutAngle m_currentRotationsHolder = Units.Rotations.mutable(
//             Double.NaN);

//     public ClawSubsystem() {
//         SparkBaseConfig MOTOR_CONFIG_LEFT = new SparkMaxConfig()
//                 .idleMode(IdleMode.kBrake)
//                 .inverted(false)
//                 .openLoopRampRate(.25)
//                 .smartCurrentLimit(40, 40)
//                 .voltageCompensation(12);

//         m_motorLeft = new SparkMax(
//                 Constants.CAN_ID.ELEVATOR_LEFT_MOTOR,
//                 MotorType.kBrushless);

//         m_motorLeft.configure(
//                 Constants.ELEVATOR.MOTOR_CONFIG_LEFT,
//                 ResetMode.kResetSafeParameters,
//                 PersistMode.kPersistParameters);
//         m_motorRight.configure(
//                 Constants.ELEVATOR.MOTOR_CONFIG_RIGHT,
//                 ResetMode.kResetSafeParameters,
//                 PersistMode.kPersistParameters);

//         m_PIDController = m_motorLeft.getClosedLoopController();

//         m_encoder = m_motorLeft.getEncoder();

//         m_hall_effect = new DigitalInput(
//                 Constants.DIGITAL_INPUT.ELEVATOR_CENTER_HALL_EFFECT_SENSOR_ID);
//         m_debouncer = new Debouncer(Constants.ELEVATOR.DEBOUNCE_TIME_SECONDS);
//     }

//     public void setSpeed(double percentOutput) {
//         m_motorLeft.set(percentOutput);
//         m_targetRotations.mut_replace(Double.NaN, Units.Rotations);
//     }

//     public void setAxisSpeed(double speed) {
//         speed *= ELEVATOR.AXIS_MAX_SPEED;
//         m_motorLeft.set(speed);
//         m_targetRotations.mut_replace(Double.NaN, Units.Rotations);
//     }

//     public void stop() {
//         m_motorLeft.stopMotor();
//         m_targetRotations.mut_replace(Double.NaN, Units.Rotations);
//     }

//     private void setTargetRotations(Angle targetRotations) {
//         m_targetRotations.mut_replace(targetRotations);
//         m_PIDController.setReference(
//                 m_targetRotations.in(Units.Rotations),
//                 ControlType.kMAXMotionPositionControl,
//                 ClosedLoopSlot.kSlot0,
//                 ELEVATOR.LEFT_MOTOR_ARB_F,
//                 ArbFFUnits.kVoltage);
//     }

//     public void setTargetDistance(Distance targetDistance) {
//         Angle rotations = Units.Rotations.of(
//                 targetDistance
//                         .div(ELEVATOR.OUTPUT_PULLEY_CIRCUMFERENCE)
//                         .times(ELEVATOR.GEAR_RATIO)
//                         .magnitude());
//         setTargetRotations(rotations);
//     }

//     public AngularVelocity getVelocity() {
//         m_currentAngularVelocityHolder.mut_replace(
//                 m_encoder.getVelocity(),
//                 Units.RPM);
//         return m_currentAngularVelocityHolder;
//     }

//     private Angle getRotations() {
//         m_currentRotationsHolder.mut_replace(
//                 m_encoder.getPosition(),
//                 Units.Rotations);
//         return m_currentRotationsHolder;
//     }

//     public Distance getDistance() {
//         return (ELEVATOR.OUTPUT_PULLEY_CIRCUMFERENCE.times(
//                 getRotations().div(ELEVATOR.GEAR_RATIO).in(Units.Rotations)));
//     }

//     private boolean isAtTargetRotations() {
//         return m_targetRotations.isNear(
//                 getRotations(),
//                 ELEVATOR.MAX_MOTION_ALLOWED_ERROR_PERCENT);
//     }

//     public boolean isAtTarget() {
//         return isAtTargetRotations();
//     }

//     public boolean isAtZero() {
//         return m_debouncer.calculate(m_hall_effect.get());
//     }

//     public void setZero() {
//         m_encoder.setPosition(0);
//     }

//     @Override
//     public void periodic() {
//         // SmartDashboard.putNumber("Elevator RPM", m_encoder.getVelocity());
//     }

//     // private SparkMax armMotor;
//     // private RelativeEncoder armEncoder;
//     // private SparkClosedLoopController armController;
//     // private SparkLimitSwitch forwardArmLimit;
//     // private SparkLimitSwitch reverseArmLimit;

//     // private SparkMax elevatorMotor;
//     // private RelativeEncoder elevatorEncoder;
//     // private SparkClosedLoopController elevatorController;
//     // private SparkLimitSwitch forwardElevatorLimit;
//     // private SparkLimitSwitch reverseElevatorLimit;

//     // // Limit switches for elevator
//     // private DigitalInput upElevatorLimitSwitch;
//     // private DigitalInput lowerElevatorLimitSwitch;

//     // // Limit switches for arm
//     // private DigitalInput upArmLimitSwitch;
//     // private DigitalInput lowerArmLimitSwitch;

//     // // Actuator to push out the coral
//     // private Servo push;

//     // private double curArmPos;
//     // private double curElevatorPos;

//     // private double currentVelocity = 0;

//     // public ClawSubsystem() {
//     // // Initialize actuator
//     // push = new Servo(7);
//     // push.setAlwaysHighMode();

//     // // Initialize arm motors
//     // armMotor = new SparkMax(20, MotorType.kBrushless);
//     // armEncoder = armMotor.getEncoder();
//     // forwardArmLimit = armMotor.getForwardLimitSwitch();
//     // reverseArmLimit = armMotor.getReverseLimitSwitch();

//     // armController = armMotor.getClosedLoopController();

//     // // Initialize elevator motor
//     // elevatorMotor = new SparkMax(15, MotorType.kBrushless);
//     // elevatorEncoder = elevatorMotor.getEncoder();
//     // forwardElevatorLimit = elevatorMotor.getForwardLimitSwitch();
//     // reverseElevatorLimit = elevatorMotor.getReverseLimitSwitch();

//     // elevatorController = elevatorMotor.getClosedLoopController();

//     // curElevatorPos = elevatorEncoder.getPosition();

//     // // Create configuration for sparks
//     // SparkMaxConfig config = new SparkMaxConfig();

//     // // config.limitSwitch
//     // // .forwardLimitSwitchType(Type.kNormallyOpen)
//     // // .forwardLimitSwitchEnabled(true)
//     // // .reverseLimitSwitchType(Type.kNormallyOpen)
//     // // .reverseLimitSwitchEnabled(true);

//     // // config.softLimit
//     // // .forwardSoftLimit(-20)
//     // // .forwardSoftLimitEnabled(true)
//     // // .reverseSoftLimit(0.1)
//     // // .reverseSoftLimitEnabled(true);

//     // config
//     // .closedLoop
//     // .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
//     // .p(0.1)
//     // .i(0)
//     // .d(0)
//     // .outputRange(-1, 1)
//     // .p(0.0001, ClosedLoopSlot.kSlot1)
//     // .i(0, ClosedLoopSlot.kSlot1)
//     // .d(0, ClosedLoopSlot.kSlot1)
//     // .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
//     // .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

//     // // config
//     // // .smartCurrentLimit(40)
//     // // .closedLoopRampRate(0.1)
//     // // .closedLoop
//     // // .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
//     // // .outputRange(-1, 1);
//     // // config
//     // // .closedLoop
//     // // .p(0.1)
//     // // .i(0)
//     // // .d(0)
//     // // .outputRange(-1, 1);

//     // // Configure motors to use the config
//     // armMotor.configure(config, ResetMode.kResetSafeParameters,
//     // PersistMode.kNoPersistParameters);
//     // elevatorMotor.configure(config, ResetMode.kResetSafeParameters,
//     // PersistMode.kNoPersistParameters);

//     // armEncoder.setPosition(0);
//     // elevatorEncoder.setPosition(0);
//     // }

//     // public void setVelocity(double targetVelocity, double rampRate, SparkMax
//     // motor, Boolean up) {
//     // new Thread(() -> {
//     // while (Math.abs(targetVelocity - currentVelocity) > 0.1) { // Small threshold
//     // to stop ramping
//     // if(up){
//     // if (targetVelocity > currentVelocity) {
//     // currentVelocity += rampRate;// Change in speed per cycle
//     // } else {
//     // currentVelocity -= rampRate;
//     // }
//     // }
//     // else{
//     // if (targetVelocity < currentVelocity) {
//     // currentVelocity -= rampRate;// Change in speed per cycle
//     // } else {
//     // currentVelocity += rampRate;
//     // }

//     // }

//     // motor.set(currentVelocity); // currentVelocity/ Max RPM

//     // try {
//     // Thread.sleep(50); // Small delay for smooth ramping
//     // } catch (InterruptedException e) {
//     // e.printStackTrace();
//     // }
//     // }

//     // motor.set(targetVelocity); // Final adjustment
//     // }).start();
//     // }

//     // public void goToLevel1() {
//     // double elevatorPos = 120;
//     // elevatorController.setReference(elevatorPos, ControlType.kPosition,
//     // ClosedLoopSlot.kSlot1);
//     // curElevatorPos = elevatorPos;

//     // double armPos = -15;
//     // armController.setReference(armPos, ControlType.kPosition,
//     // ClosedLoopSlot.kSlot1);
//     // curArmPos = armPos;
//     // }

//     // public void moveArm(boolean up) {
//     // if (up) {
//     // setVelocity(.15, .05, armMotor, true);
//     // } else {
//     // setVelocity(-.15, .05, armMotor, false);
//     // }

//     // curArmPos = armEncoder.getPosition();
//     // System.out.println("arm pos: " + curArmPos);
//     // }

//     // public void moveElevator(boolean up) {
//     // if (up) {
//     // setVelocity(.45, .05, elevatorMotor, true);
//     // } else {
//     // setVelocity(-.45, .05, elevatorMotor, false);
//     // }

//     // curElevatorPos = elevatorEncoder.getPosition();
//     // System.out.println("moved elevator pos: " + curElevatorPos);
//     // }

//     // public void drop() {
//     // push.setAngle(0);
//     // }

//     // public Command goToLevel1Command() {
//     // return run(() -> goToLevel1());
//     // }

//     // public Command moveArmUpCommand() {
//     // return run(() -> moveArm(true));
//     // }

//     // public Command moveArmDownCommand() {
//     // return run(() -> moveArm(false));
//     // }

//     // public Command moveElevatorUpCommand() {
//     // return run(() -> moveElevator(true));
//     // }

//     // public Command moveElevatorDownCommand() {
//     // return run(() -> moveElevator(false));
//     // }

//     // public Command holdArmPositionCommand() {
//     // return run(() -> {
//     // System.out.println("holding arm pos: " + curArmPos);
//     // armController.setReference(curArmPos, ControlType.kPosition,
//     // ClosedLoopSlot.kSlot0);
//     // });
//     // }

//     // public Command holdElevatorPositionCommand() {
//     // return run(() -> {
//     // elevatorController.setReference(curElevatorPos, ControlType.kPosition,
//     // ClosedLoopSlot.kSlot0);
//     // System.out.println("holding elevator pos: " + curElevatorPos);
//     // });
//     // }

//     // public Command holdCommand() {
//     // return run(() -> {
//     // elevatorController.setReference(curElevatorPos, ControlType.kPosition,
//     // ClosedLoopSlot.kSlot0);
//     // armController.setReference(curArmPos, ControlType.kPosition,
//     // ClosedLoopSlot.kSlot0);
//     // });
//     // }

//     // public Command dropCoralCommand() {
//     // return run(() -> drop());
//     // }
// }
