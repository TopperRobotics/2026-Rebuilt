package frc.robot.subsystems.localization;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class VisionSubsystem extends SubsystemBase {
    private SwerveSubsystem swerveSubsystem;
    private SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    private String limeLightFName; // this limelight is mounted facing forwards
    private String limeLightBName; // this limelight is mounted facing backwards

    public VisionSubsystem(SwerveSubsystem swerveSubsystem, String limeLightFName, String limeLightBName) {
        this.swerveSubsystem = swerveSubsystem;
        this.limeLightFName = limeLightFName;
        this.limeLightBName = limeLightBName;
        this.swerveDrivePoseEstimator = this.swerveSubsystem.getSwerveDrive().swerveDrivePoseEstimator;

    }

    private void updateLimelightOrientation() {
        LimelightHelpers.SetRobotOrientation(limeLightFName,
                swerveDrivePoseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation(limeLightBName,
                swerveDrivePoseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    }

    private void updateRobotPose() {
        boolean doRejectUpdate = false;
        if (LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limeLightFName) != null && (LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limeLightFName).pose.getX() != 0.0 && LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limeLightFName).pose.getY() != 0.0)) {
            SmartDashboard.putBoolean("Vision/Pose Was Null", false);
            LimelightHelpers.PoseEstimate mt2LimelightF = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limeLightFName);
            //LimelightHelpers.PoseEstimate mtLimelightF = LimelightHelpers.getBotPoseEstimate_wpiBlue(limeLightBName)
            if (Math.abs(
                    Math.toDegrees(swerveSubsystem.getSwerveDrive().getRobotVelocity().omegaRadiansPerSecond)) > 720)
            {
                doRejectUpdate = true;
            }
            if (!doRejectUpdate) { // update robot pose with measurements from both limelights
                swerveDrivePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.6, .6, 9999999));
                swerveDrivePoseEstimator.addVisionMeasurement(
                        mt2LimelightF.pose,
                        mt2LimelightF.timestampSeconds);
            }
        } else if (LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limeLightBName) != null && (LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limeLightBName).pose.getX() != 0.0 && LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limeLightBName).pose.getY() != 0.0)) {
            SmartDashboard.putBoolean("Vision/Pose Was Null", false);
            LimelightHelpers.PoseEstimate mt2LimelightB = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limeLightBName);
            if (Math.abs(
                    Math.toDegrees(swerveSubsystem.getSwerveDrive().getRobotVelocity().omegaRadiansPerSecond)) > 720)
            {
                doRejectUpdate = true;
            }
            if (!doRejectUpdate) { // update robot pose with measurements from both limelights
                swerveDrivePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.6, .6, 9999999));
                swerveDrivePoseEstimator.addVisionMeasurement(
                        mt2LimelightB.pose,
                        mt2LimelightB.timestampSeconds);
            }
        } else {
            SmartDashboard.putBoolean("Vision/Pose Was Null", true);
        }
    }

    @Override
    public void periodic() {
        // Update vision data periodically from both limelights
        updateLimelightOrientation();
        updateRobotPose();
    }

}