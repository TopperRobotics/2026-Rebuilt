package frc.robot.subsystems.localization;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PathfindingSubsystem extends SubsystemBase {

    PathConstraints constraints;

    public PathfindingSubsystem(){
        constraints  = new PathConstraints(
        3.0, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));
    }

    public Command pathfindToPose(Pose2d targetPose){
        return AutoBuilder.pathfindToPose(targetPose, constraints);
    }

    public Command pathfindToPathAndFollowIt(PathPlannerPath targetPath){
        return AutoBuilder.pathfindThenFollowPath(targetPath, constraints);
    }
    
}
