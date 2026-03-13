package frc.robot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
import frc.robot.Constants.climber;
import frc.robot.subsystems.localization.PathfindingSubsystem;
import frc.robot.subsystems.localization.VisionSubsystem;
import frc.robot.subsystems.mechanisms.ClimberSubsystem;
import frc.robot.subsystems.mechanisms.IntakeSubsystem;
import frc.robot.subsystems.mechanisms.ShooterHoodSubsystem;
import frc.robot.subsystems.mechanisms.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.AutoAimingSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import java.io.File;
import java.util.Optional;

import swervelib.SwerveInputStream;

public class RobotContainer {

        final CommandXboxController driverXbox = new CommandXboxController(2);
        private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                        "swerve/neo"));
        private IntakeSubsystem intake = new IntakeSubsystem();
        private ShooterSubsystem shooter = new ShooterSubsystem();
        private ClimberSubsystem climber = new ClimberSubsystem();
        private ShooterHoodSubsystem shooterHood = new ShooterHoodSubsystem(new Translation2d[]{Constants.FieldConstants.kLeftHopper, Constants.FieldConstants.kRightHopper}, drivebase);
        private AutoAimingSubsystem autoAim = new AutoAimingSubsystem(drivebase, new Translation2d[]{Constants.FieldConstants.kLeftHopper, Constants.FieldConstants.kRightHopper}, driverXbox);
        //private VisionSubsystem vision = new VisionSubsystem(drivebase, Constants.LIMELIGHT_FRONT_NAME, Constants.LIMELIGHT_BACK_NAME);
        private SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

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
        SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
                        .withControllerHeadingAxis(driverXbox::getRightX,
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
                NamedCommands.registerCommand("SetOdometryToLLPose", getAutonomousCommand());
                NamedCommands.registerCommand("DeployIntake", intake.moveToPosition(Constants.intakeArm.deployedPosition, false));
                NamedCommands.registerCommand("RetractIntake", intake.moveToPosition(Constants.intakeArm.retractedPosition, false));
                NamedCommands.registerCommand("RunIntakeRollers", intake.in());
                NamedCommands.registerCommand("StopIntakeRollers", intake.stop());
                NamedCommands.registerCommand("ExtendClimber", climber.moveToPosition(Constants.climber.extendedPosition));
                NamedCommands.registerCommand("RetractClimber", climber.moveToPosition(Constants.climber.retractedPosition));
                NamedCommands.registerCommand("RunShooter", shooter.shoot());
                NamedCommands.registerCommand("StopShooter", shooter.stopShooting());

                // Configure the trigger bindings
                configureBindings();

                // Set the default auto (do nothing)
                autoChooser.setDefaultOption("Do Nothing", Commands.runOnce(drivebase::zeroGyroWithAlliance)
                                .andThen(Commands.none()));

                // Put the autoChooser on the SmartDashboard
                SmartDashboard.putData("Auto Chooser", autoChooser);

                if (autoChooser.getSelected() == null) {
                        RobotModeTriggers.autonomous().onTrue(Commands.runOnce(drivebase::zeroGyroWithAlliance));
                }
        }

        private void configureBindings() {
                Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
                Command driveRobotOriented = drivebase.driveRobotOriented(driveDirectAngle);

                drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

                // Aim at target continuously while allowing driver movement
                driverXbox.leftTrigger()
                                .whileTrue(Commands.run(() -> {
                                        shooterHood.autoAdjustHood();
                                        autoAim.aimAtTarget();
                                }));

                driverXbox.rightBumper().onTrue(drivebase.centerModulesCommand());
                driverXbox.a().onTrue(shooter.shoot());
                driverXbox.a().onFalse(shooter.stopShooting());
                driverXbox.x().onTrue(Commands.run(() -> intake.toggleIntakeOutness()));
                driverXbox.povUp().onTrue(climber.moveToPosition(Constants.climber.extendedPosition));
                driverXbox.povDown().onTrue(climber.moveToPosition(Constants.climber.retractedPosition));
                driverXbox.rightTrigger().whileTrue(driveRobotOriented);
                driverXbox.povLeft().onTrue(shooterHood.moveToPosition(new Rotation2d(Math.toRadians(-20))));
                driverXbox.povRight().onTrue(shooterHood.moveToPosition(new Rotation2d(Math.toRadians(0))));
                driverXbox.y().onTrue(climber.runNegative());
                driverXbox.y().onFalse(climber.stop());
                driverXbox.leftBumper().onTrue(climber.runPositive());
                driverXbox.leftBumper().onFalse(climber.stop());
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