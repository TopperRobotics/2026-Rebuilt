package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class AutoAimingSubsystem extends SubsystemBase {

    private SwerveSubsystem drivebase;
    private Translation2d hopperPosition;
    private CommandXboxController driverXbox;

    // PID controller for target aiming
    // TODO: tune PID
    private final ProfiledPIDController aimPIDController = new ProfiledPIDController(
            0.6, 0.01, 0.22,
            new TrapezoidProfile.Constraints(9999, 9999));

    public AutoAimingSubsystem(SwerveSubsystem drivebase, Translation2d hopperPosition, CommandXboxController controller) {
        this.drivebase = drivebase;
        this.hopperPosition = hopperPosition;
        this.driverXbox = controller;
    }

    public void aimAtTarget() {
        // Get current pose and target pose
        Rotation2d currentRotation = drivebase.getPose().getRotation();
        Rotation2d targetRotation = drivebase.getPoseFacingHopper(
                drivebase.getPose(),
                hopperPosition).getRotation();

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
    }
}
