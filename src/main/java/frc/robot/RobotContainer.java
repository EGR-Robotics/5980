package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

// Constants
import frc.robot.generated.TunerConstants;
import frc.robot.Constants.ARM;
import frc.robot.Constants.DRIVETRAIN;
import frc.robot.Constants.ELEVATOR;
// Commands
import frc.robot.commands.elevator.MoveElevator;
import frc.robot.commands.actuator.PushOut;
import frc.robot.commands.actuator.StopServo;
import frc.robot.commands.arm.MoveArm;
import frc.robot.commands.arm.StopArm;
import frc.robot.commands.elevator.StopElevator;
// import frc.robot.commands.scoring.L1;
import frc.robot.commands.scoring.L2;
import frc.robot.commands.scoring.L3;
import frc.robot.commands.scoring.L4;
import frc.robot.commands.auto.Align;
import frc.robot.commands.auto.L4AutoLower;
import frc.robot.commands.auto.ScoreElevator;
import frc.robot.commands.auto.Trough;

// Subsystems
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Vision;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Actuator;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Algae;

import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    // Set up swerve request bindings for necessary control of the swerve drive platform
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(DRIVETRAIN.MAX_SPEED * DRIVETRAIN.DRIVE_DEADBAND)
            .withRotationalDeadband(DRIVETRAIN.MAX_ANGULAR_RATE * DRIVETRAIN.ROTATION_DEADBAND) // Multiply by deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(DRIVETRAIN.MAX_SPEED);

    // Initialize controllers
    public final static CommandXboxController driverJoystick = new CommandXboxController(0);
    public final static CommandXboxController controllerJoystick = new CommandXboxController(1);

    // Initialize subsystems
    public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public static final Vision vision = new Vision();

    public final Climber climber = new Climber();
    public static final Actuator actuator = new Actuator();

    public static final Elevator elevator = new Elevator();
    public static final Arm arm = new Arm();
    public static final Algae algae = new Algae();

    // Initialize auto chooser
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // Register named commands for auto
        NamedCommands.registerCommand("zero", new InstantCommand(
                () -> elevator.setEncoderPosition(0), elevator));

        NamedCommands.registerCommand("slightLower", new L4AutoLower());
        NamedCommands.registerCommand("level4", new frc.robot.commands.auto.L4());

        NamedCommands.registerCommand("trough", new Trough());
        NamedCommands.registerCommand("pickup", new ScoreElevator());

        NamedCommands.registerCommand("align", new Align());

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Configure bindings from controller to commands
        configureBindings();
    }

    private void configureBindings() {
        // Elevator Commands

        elevator.setDefaultCommand(
                new InstantCommand(
                        () -> {
                            if (controllerJoystick.getLeftY() > ELEVATOR.DEADBAND) {
                                (new MoveElevator(true)).execute();
                            } else if (controllerJoystick.getLeftY() < -ELEVATOR.DEADBAND) {
                                (new MoveElevator(false)).execute();
                            } else {
                                (new StopElevator()).execute();
                                elevator.holdPosition();
                            }
                        }, elevator));

        controllerJoystick.a().whileTrue(new L4());
        controllerJoystick.b().whileTrue(new L3());
        controllerJoystick.y().whileTrue(new L2());

        controllerJoystick.x().whileTrue(new PushOut());
        controllerJoystick.x().onFalse(new StopServo());

        // Arm commands

        arm.setDefaultCommand(
                new InstantCommand(
                        () -> {
                            if (controllerJoystick.getRightY() > ARM.DEADBAND) {
                                (new MoveArm(true)).execute();
                            } else if (controllerJoystick.getRightY() < -ARM.DEADBAND) {
                                (new MoveArm(false)).execute();
                            } else {
                                (new StopArm()).execute();
                            }
                        }, arm));

        // Algae Bar Commands
        controllerJoystick.leftTrigger().whileTrue(algae.moveElevatorDownCommand());
        controllerJoystick.leftTrigger().onFalse(algae.holdElevatorPositionCommand());

        controllerJoystick.rightTrigger().whileTrue(algae.moveElevatorUpCommand());
        controllerJoystick.rightTrigger().onFalse(algae.holdElevatorPositionCommand());

        controllerJoystick.leftBumper().whileTrue(algae.dropAlgaeCommand());
        controllerJoystick.leftBumper().onFalse(algae.stopArm());

        controllerJoystick.rightBumper().whileTrue(algae.moveArmCommand());
        controllerJoystick.rightBumper().onFalse(algae.stopArm());

        // Drive Commands

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> {
                    double targetDriveSpeed = DRIVETRAIN.MAX_SPEED;
                    double targetAngularRate = DRIVETRAIN.MAX_ANGULAR_RATE;

                    // If elevator is raised
                    if (elevator.getEncoderPosition() <= ELEVATOR.SLOW_DOWN_POSITION) {
                        targetDriveSpeed *= ELEVATOR.DRIVE_SLOW_DOWN_RATE;
                        targetAngularRate *= ELEVATOR.DRIVE_ANGULAR_SLOW_DOWN_RATE;
                    }

                    // If the right trigger is pressed
                    if (driverJoystick.getRightTriggerAxis() == 1) {
                        targetDriveSpeed *= DRIVETRAIN.SLOW_DOWN_RATE;
                        targetAngularRate *= DRIVETRAIN.SLOW_DOWN_RATE;
                    }

                    // if (driverJoystick.getLeftTriggerAxis() == 1) {
                    // double kp_aim = 0.02;

                    // double tx = (LimelightHelpers.getTX(LIMELIGHT.LIMELIGHT_NAME_1) + 1);
                    // double rotationSpeed = -tx * kp_aim;

                    // SwerveRequest.RobotCentric limelightRotate = new SwerveRequest.RobotCentric()
                    // .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

                    // return
                    // limelightRotate.withVelocityX(0).withVelocityY(rotationSpeed).withRotationalRate(0);
                    // }

                    return drive
                            // Drive forward with negative Y forward
                            .withVelocityX(-driverJoystick.getLeftY() * targetDriveSpeed)
                            // Drive left with negative X (left)
                            .withVelocityY(-driverJoystick.getLeftX() * targetDriveSpeed)
                            // Drive counterclockwise with negative X (left)
                            .withRotationalRate(-driverJoystick.getRightX() * targetAngularRate);
                }));

        driverJoystick.leftTrigger().onTrue(new Align());

        driverJoystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

        // Zero out
        driverJoystick.b().onTrue(
                drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(0, 0))));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverJoystick.back().and(driverJoystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverJoystick.back().and(driverJoystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverJoystick.start().and(driverJoystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverJoystick.start().and(driverJoystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driverJoystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        // Climber Commands
        driverJoystick.x().whileTrue(climber.moveWenchUp());
        driverJoystick.x().onFalse(climber.stopWenchCommand());
        driverJoystick.y().whileTrue(climber.moveWenchDown());
        driverJoystick.y().onFalse(climber.stopWenchCommand());
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
