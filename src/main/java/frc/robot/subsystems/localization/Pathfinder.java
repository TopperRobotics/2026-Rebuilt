package frc.robot.subsystems.localization;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Collectors;

/**
 * Utility that builds an on-the-fly PathPlanner path from the current robot pose to a target pose
 * and returns a list of Pose2d waypoints describing that path.
 *
 * Usage:
 *   - Provide a Supplier<Pose2d> that returns the robot's current field-relative pose.
 *   - Provide PathConstraints for max velocity/accel (or use PathConstraints.unlimitedConstraints()).
 *   - Optionally provide a RobotConfig if you want dense, generated trajectory poses (recommended).
 */
public class Pathfinder {
    private final Supplier<Pose2d> currentPoseSupplier;
    private final PathConstraints constraints;
    private final RobotConfig robotConfig; // optional; pass null to use fallback

    /**
     * @param currentPoseSupplier supplies the current robot pose (field-relative)
     * @param constraints path constraints (max vel, accel, etc.)
     * @param robotConfig optional robot configuration; provide null if unavailable
     */
    public Pathfinder(Supplier<Pose2d> currentPoseSupplier,
                                PathConstraints constraints,
                                RobotConfig robotConfig) {
        this.currentPoseSupplier = currentPoseSupplier;
        this.constraints = constraints;
        this.robotConfig = robotConfig;
    }

    /**
     * Generate a list of Pose2d waypoints that move the robot from the current pose to the target pose.
     * If RobotConfig was provided at construction, the method will generate a PathPlannerTrajectory
     * and return the trajectory states' poses (higher fidelity). Otherwise it will return a
     * list of Pose2d anchors derived from PathPlanner waypoints (coarser).
     *
     * @param target goal pose (field-relative). For non-holonomic drives, only translation and travel direction matter.
     * @return list of Pose2d that represent the generated path (in field coordinates)
     */
    public List<Pose2d> generateWaypointsTo(Pose2d target) {
        // 1) Build pose list: current + target.
        Pose2d start = currentPoseSupplier.get();
        List<Pose2d> poses = Arrays.asList(start, target);

        // 2) Convert poses -> bezier waypoints (helper provided by PathPlannerPath).
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poses);

        // 3) Build a PathPlannerPath (on-the-fly). No ideal starting state / goal end state for now.
        PathPlannerPath path = new PathPlannerPath(waypoints, constraints, null, null);
        path.preventFlipping = true; // useful if coordinates are already field-correct

        // 4a) Preferred: if RobotConfig is present, generate a PathPlannerTrajectory and return sampled poses.
        if (robotConfig != null) {
            // starting speeds = zero; starting rotation = current heading
            ChassisSpeeds startingSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
            Rotation2d startingRotation = start.getRotation();

            PathPlannerTrajectory traj = new PathPlannerTrajectory(path, startingSpeeds, startingRotation, robotConfig);

            // Extract poses from trajectory states
            return traj.getStates()
                       .stream()
                       .map(s -> s.pose) // PathPlannerTrajectoryState.pose is a public field
                       .collect(Collectors.toList());
        }

        // 4b) Fallback: return anchors as Pose2d, estimate headings from successive anchors.
        List<Pose2d> out = new ArrayList<>(waypoints.size());
        for (int i = 0; i < waypoints.size(); i++) {
            Waypoint wp = waypoints.get(i);
            Translation2d anchor = wp.anchor(); // translation of the waypoint

            // compute heading toward next anchor if possible; otherwise re-use previous heading or 0
            Rotation2d heading;
            if (i < waypoints.size() - 1) {
                Translation2d next = waypoints.get(i + 1).anchor();
                double dx = next.getX() - anchor.getX();
                double dy = next.getY() - anchor.getY();
                if (dx == 0.0 && dy == 0.0) {
                    heading = (out.isEmpty()) ? new Rotation2d() : out.get(out.size() - 1).getRotation();
                } else {
                    heading = Rotation2d.fromRadians(Math.atan2(dy, dx));
                }
            } else { // last point: use previous heading if exists, else compute from start->target
                if (!out.isEmpty()) {
                    heading = out.get(out.size() - 1).getRotation();
                } else {
                    double dx = target.getX() - start.getX();
                    double dy = target.getY() - start.getY();
                    heading = (dx == 0.0 && dy == 0.0)
                              ? new Rotation2d()
                              : Rotation2d.fromRadians(Math.atan2(dy, dx));
                }
            }

            out.add(new Pose2d(anchor, heading));
        }

        return out;
    }
}