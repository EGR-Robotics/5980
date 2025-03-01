package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

// Constants
import frc.robot.generated.TunerConstants;
import frc.robot.Constants.ALGAE;
import frc.robot.Constants.APRIL_TAGS;
import frc.robot.Constants.CLIMBER;
// Commands
import frc.robot.commands.elevator.MoveElevator;
import frc.robot.commands.elevator.SetElevatorDistance;
import frc.robot.commands.actuator.Drop;
import frc.robot.commands.algae.MoveAlgaeArm;
import frc.robot.commands.algae.MoveIntake;
import frc.robot.commands.arm.MoveArm;
import frc.robot.commands.arm.StopArm;
// import frc.robot.commands.climber.MoveClimber;
import frc.robot.commands.elevator.StopElevator;
import frc.robot.commands.scoring.L1;
import frc.robot.commands.scoring.L2;
import frc.robot.commands.scoring.L3;
import frc.robot.commands.scoring.L4;

// Subsystems
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.VisionSubsystemOld;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Actuator;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.Climber;

import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Initialize controllers
    private final CommandXboxController driverJoystick = new CommandXboxController(0);
    private final CommandXboxController controllerJoystick = new CommandXboxController(1);

    // Initialize subsystems

    public final VisionSubsystemOld vision = new VisionSubsystemOld();
    public final ClimberSubsystem climberOld = new ClimberSubsystem();
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public static final Algae algae = new Algae();
    public static final Elevator elevator = new Elevator();
    public static final Arm arm = new Arm();
    public static final Actuator actuator = new Actuator();
    // public static final Climber climber = new Climber();

    // public static final AlgaeSubsystem algaeSubsystem = new AlgaeSubsystem();

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // Register named commands for auto
        NamedCommands.registerCommand("level4", new L4());
        // NamedCommands.registerCommand("drop", new Drop());

        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser);

        var alliance = DriverStation.getAlliance();

        APRIL_TAGS.update(alliance.get());

        // actuator.setSpeed(1);
    }

    private void configureBindings() {
        // Elevator Commands

        elevator.setDefaultCommand(
                new InstantCommand(
                        () -> {
                            if (controllerJoystick.getLeftY() > 0.1) {
                                (new MoveElevator(true)).execute();
                            } else if (controllerJoystick.getLeftY() < -0.2) {
                                (new MoveElevator(false)).execute();
                            } else {
                                elevator.holdPosition();
                                (new StopElevator()).execute();
                            }
                        }, elevator));

        // controllerJoystick.x().onTrue(new Drop());

        // controllerJoystick.x().onTrue(new InstantCommand(() -> actuator.setPosition(0)));

        controllerJoystick.a().whileTrue(new L4());
        controllerJoystick.b().onTrue(new L2());
        controllerJoystick.y().onTrue(new L3());
        // controll+erJoystick.x().onTrue(new L1());

        controllerJoystick.x().onTrue(new SetElevatorDistance(0));

        // Arm commands

        arm.setDefaultCommand(
                new InstantCommand(
                        () -> {
                            if (controllerJoystick.getRightY() > 0.2) {
                                (new MoveArm(true)).execute();
                            } else if (controllerJoystick.getRightY() < -0.2) {
                                (new MoveArm(false)).execute();
                            } else {
                                (new StopArm()).execute();
                            }
                        }, arm));

        // Algae Bar Commands

        controllerJoystick.leftTrigger().whileTrue(new MoveAlgaeArm(ALGAE.ARM_RAISE_SPEED));
        controllerJoystick.rightTrigger().whileTrue(new MoveAlgaeArm(ALGAE.ARM_LOWER_SPEED));
        // controllerJoystick.leftTrigger().onFalse(algae.zero());
        // controllerJoystick.rightTrigger().onFalse(algae.zero());



        // algae.setDefaultCommand(
        //     new InstantCommand(
        //         () -> algae.zero().execute(), algae
        //     )
        // );

        controllerJoystick.leftBumper().whileTrue(new MoveIntake(ALGAE.INTAKE_SPEED));
        // controllerJoystick.leftBumper().onFalse(algae.zero());
        
        controllerJoystick.rightBumper().whileTrue(new MoveIntake(ALGAE.OUTAKE_SPEED));
        // controllerJoystick.rightBumper().onFalse(algae.zero());

        
        // controllerJoystick.leftBumper().whileTrue(algaeSubsystem.moveElevatorUpCommand());
        // controllerJoystick.leftBumper().onFalse(algaeSubsystem.holdElevatorPositionCommand());
        
        // controllerJoystick.rightBumper().whileTrue(algaeSubsystem.moveElevatorDownCommand());
        // controllerJoystick.rightBumper().onFalse(algaeSubsystem.holdElevatorPositionCommand());

        // Drive Commands

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> {
                    double curSpeed = MaxSpeed;
                    double angularSpeed = MaxAngularRate;

                    if(elevator.getEncoderPosition() <= -50) {
                        curSpeed *= .4;
                        angularSpeed *= .4;

                        System.out.println("Modifying speeds: " + curSpeed + " at " + elevator.getEncoderPosition());
                    }

                    return drive.withVelocityX(-driverJoystick.getLeftY() * curSpeed) // Drive
                                                                                                         // forward with
                                                                                                         // negative Y
                                                                                                         // (forward)
                        .withVelocityY(-driverJoystick.getLeftX() * curSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-driverJoystick.getRightX() * angularSpeed); // Drive counterclockwise with
                                                                                          // negative X (left)
                }
                ));

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

        // Limelight commands

        // Limelight Align Commands
        // controllerJoystick.y().onTrue(
        // vision.alignCommand(drivetrain)
        // );

        // Climber Commands
        driverJoystick.x().onTrue(climberOld.moveWenchUp()); 
        driverJoystick.y().whileTrue(climberOld.moveWenchDown());
        driverJoystick.y().onFalse(climberOld.stopWenchCommand());

        // driverJoystick.x().onTrue(new MoveClimber(CLIMBER.SPEED));
        // driverJoystick.y().whileTrue(new MoveClimber(-CLIMBER.SPEED));    
    }

    public Command getAutonomousCommand() {
        // return new PathPlannerAuto("Test");
        // return new PathPlannerAuto("New Auto");
        return autoChooser.getSelected();
    }
}
