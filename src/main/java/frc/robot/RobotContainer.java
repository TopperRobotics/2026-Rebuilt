package frc.robot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.localization.LimelightSubsystem;
import frc.robot.subsystems.localization.PathfindingSubsystem;
import frc.robot.subsystems.localization.QuestNavSubsystem;
import frc.robot.subsystems.mechanisms.IntakeSubsystem;
import frc.robot.subsystems.mechanisms.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import gg.questnav.questnav.QuestNav;

import java.io.File;
import java.util.Optional;

import swervelib.SwerveInputStream;

public class RobotContainer {

    // Replace with CommandPS4Controller or CommandJoystick if needed
    final CommandXboxController driverXbox = new CommandXboxController(0);
    // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve/neo"));
    private LimelightSubsystem limelight = new LimelightSubsystem();
    private QuestNav questNav = new QuestNav();
    private QuestNavSubsystem questNavSubsystem = new QuestNavSubsystem(questNav, drivebase, limelight);
    private IntakeSubsystem intake = new IntakeSubsystem();
    private ShooterSubsystem shooter = new ShooterSubsystem();
    private PathfindingSubsystem pathfinder = new PathfindingSubsystem();

    // PID controller for target aiming
    private final ProfiledPIDController aimPIDController = new ProfiledPIDController(
            0.6, 0.01, 0.22,
            new TrapezoidProfile.Constraints(9999, 9999));

    Translation2d hopperPosition;

    // Establish a Sendable Chooser that will be able to be sent to the
    // SmartDashboard, allowing selection of desired auto
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled
     * by angular velocity.
     */
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
            () -> driverXbox.getLeftX(),
            () -> driverXbox.getLeftY() * -1)
            .withControllerRotationAxis(driverXbox::getRightX)
            .deadband(OperatorConstants.DEADBAND)
            .allianceRelativeControl(false);

    /**
     * Clone's the angular velocity input stream and converts it to a fieldRelative
     * input stream.
     */
    SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
            driverXbox::getRightY)
            .headingWhile(true);

    /**
     * Clone's the angular velocity input stream and converts it to a robotRelative
     * input stream.
     */
    SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
            .allianceRelativeControl(false);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
        DriverStation.silenceJoystickConnectionWarning(true);
        // TODO: uncomment this when using on the real robot
        // questNavSubsystem.resetTrackingToPoseFromLimelight();

        // Set the default auto (do nothing)
        autoChooser.setDefaultOption("Do Nothing", Commands.runOnce(drivebase::zeroGyroWithAlliance)
                .andThen(Commands.none()));

        // Add a simple auto option to have the robot drive forward for 1 second then
        // stop
        autoChooser.addOption("Drive Forward", Commands.runOnce(drivebase::zeroGyroWithAlliance).withTimeout(.2)
                .andThen(drivebase.driveForward().withTimeout(1)));
        // Put the autoChooser on the SmartDashboard
        SmartDashboard.putData("Auto Chooser", autoChooser);

        if (autoChooser.getSelected() == null) {
            RobotModeTriggers.autonomous().onTrue(Commands.runOnce(drivebase::zeroGyroWithAlliance));
        }

        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
                hopperPosition = Constants.FieldConstants.kLeftHopper;
            }
            if (ally.get() == Alliance.Blue) {
                hopperPosition = Constants.FieldConstants.kRightHopper;
            }
        }
        else {
            // default to blue
            hopperPosition = Constants.FieldConstants.kRightHopper;
        }
    }

    private void configureBindings() {
        Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity); // this one is
                                                                                                         // used

        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
        // Deploy the intake with the X button and start intake roller
        driverXbox.x().onTrue(intake.moveToPosition(Constants.intakeArm.deployedPosition, true));
        // Retract the intake with the Y button and stop intake roller
        driverXbox.y().onTrue(intake.moveToPosition(Constants.intakeArm.retractedPosition, false));

        // Aim at target continuously while allowing driver movement
        driverXbox.leftTrigger()
                .whileTrue(Commands.run(() -> {
                    // Get current pose and target pose
                    Rotation2d currentRotation = drivebase.getPose().getRotation();
                    Rotation2d targetRotation = drivebase.getPoseFacingHopper(
                            drivebase.getPose(),
                            Constants.FieldConstants.kRightHopper).getRotation();

                    // Calculate the shortest angular distance using Rotation2d.minus()
                    Rotation2d angleDifference = targetRotation.minus(currentRotation);

                    // Calculate angular velocity from PID controller
                    // Using the shortest angle in radians
                    double angularVelocity = aimPIDController.calculate(
                            0, // Current error is 0
                            angleDifference.getRadians() // Target is the shortest angle to rotate
                    );

                    // Get translation from driver input
                    Translation2d translation = new Translation2d(
                            driverXbox.getLeftX() * 3,
                            (driverXbox.getLeftY() * -1) * 3);

                    SmartDashboard.putNumber("Target oriented angular velocity radps", angularVelocity);
                    SmartDashboard.putNumber("Target oriented y velocity mps", translation.getX());
                    SmartDashboard.putNumber("Target oriented x velocity mps", translation.getY());

                    // Drive with calculated rotation
                    drivebase.getSwerveDrive().drive(translation, angularVelocity * 5, true, false);
                }));

        driverXbox.a().onTrue(Commands.runOnce(() -> drivebase.resetPose(drivebase.getPose()))); // for sim, comment out
                                                                                                 // when running on real
                                                                                                 // robot
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Pass in the selected auto from the SmartDashboard as our desired autnomous
        // commmand
        return autoChooser.getSelected();
    }

    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }
}