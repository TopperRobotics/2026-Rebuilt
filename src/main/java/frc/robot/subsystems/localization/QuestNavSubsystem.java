package frc.robot.subsystems.localization;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Constants.QuestNavConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import edu.wpi.first.wpilibj2.command.*;

public class QuestNavSubsystem extends SubsystemBase{

    QuestNav questNav;
    SwerveSubsystem swerveSubsystem;
    ApriltagLL limelight;

    public QuestNavSubsystem(QuestNav questNav, SwerveSubsystem swerve, ApriltagLL limelight) {
        this.questNav = questNav;
        this.swerveSubsystem = swerve;
        this.limelight = limelight;
    }

    public void resetTrackingToPoseFromLimelight(){
        if(limelight.isLimelightConnected()){
            questNav.setPose(limelight.getPose3d().transformBy(Constants.QuestNavConstants.ROBOT_TO_QUEST));
        } else {
            System.out.println("ll not connected");
            DriverStation.reportWarning("ll not connected", true);
        }
    }

    @Override
    public void periodic() {

        // Get the latest pose data frames from the Quest
        PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();

        // Loop over the pose data frames and send them to the pose estimator
        for (PoseFrame questFrame : questFrames) {
            // Make sure the Quest was tracking the pose for this frame
            if (questFrame.isTracking()) {
                // Get the pose of the Quest
                Pose3d questPose = questFrame.questPose3d();
                // Get timestamp for when the data was sent
                double timestamp = questFrame.dataTimestamp();

                // Transform by the mount pose to get your robot pose
                Pose3d robotPose = questPose.transformBy(QuestNavConstants.ROBOT_TO_QUEST.inverse());

                // TODO: add filtering if pose jumps a lot

                // Add the measurement to our estimator
                swerveSubsystem.getSwerveDrive().addVisionMeasurement(robotPose.toPose2d(), timestamp, QuestNavConstants.QUESTNAV_STD_DEVS);
            }
        }
    }

}
