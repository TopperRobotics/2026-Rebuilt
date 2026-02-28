package frc.robot.subsystems.mechanisms;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class ShooterHoodSubsystem extends SubsystemBase {

    private double distanceFromTarget = 0.000;
    private double neededHoodAngle = Constants.shooterHood.reverseSoftLimit;

    private Translation2d hopperPosition;
    private SwerveSubsystem swerveSubsystem;

    private SparkMax hoodMotor;
    private RelativeEncoder hoodMotorEncoder;
    private SparkClosedLoopController hoodPID;

    public ShooterHoodSubsystem(Translation2d hopperPosition, SwerveSubsystem swerveSubsystem) {
        this.hopperPosition = hopperPosition;
        this.swerveSubsystem = swerveSubsystem;

        hoodMotor = new SparkMax(25, MotorType.kBrushless);

        // Configure motor settings using SparkMaxConfig
        SparkMaxConfig config = new SparkMaxConfig();
        config
                .idleMode(IdleMode.kBrake);

        // Configure encoder settings
        config.encoder
                .positionConversionFactor(360.0 / Constants.shooterHood.gearRatio)
                .velocityConversionFactor((360.0 / Constants.shooterHood.gearRatio) / 60.0);

        // Configure PID controller settings
        config.closedLoop
                .pid(Constants.shooterHood.kP, Constants.shooterHood.kI, Constants.shooterHood.kD);

        config.softLimit.forwardSoftLimit(Constants.shooterHood.forwardSoftLimit);
        config.softLimit.reverseSoftLimit(Constants.shooterHood.reverseSoftLimit);

        // Apply configuration to motor
        hoodMotor.configure(
                config,
                SparkMax.ResetMode.kResetSafeParameters,
                SparkMax.PersistMode.kPersistParameters);

        // Get encoder and PID references after configuration
        hoodMotorEncoder = hoodMotor.getEncoder();
        hoodPID = hoodMotor.getClosedLoopController();

        // Reset encoder position to zero
        hoodMotorEncoder.setPosition(0);
    }

    public void setHoodPosition(double positionDegrees) {
        hoodPID.setSetpoint(positionDegrees, SparkBase.ControlType.kPosition);
    }

    public void putDistanceFromTargetOnNT() {
        SmartDashboard.putNumber("Hood/Distance From Target", distanceFromTarget);
    }

    public void retrieveNeededHoodAngleFromNT() {
        double neededAngle = SmartDashboard.getNumber("Hood/Needed Angle", Constants.shooterHood.reverseSoftLimit);
        if(neededAngle != -1){ // -1 means that at the robot's current distance from the target, no angle could be calculated to get the ball in the target
            this.neededHoodAngle = neededAngle;
        } else {
            this.neededHoodAngle = Constants.shooterHood.reverseSoftLimit;
        }
    }

    public void setDistanceFromTarget(double distance) {
        this.distanceFromTarget = distance;
    }

    public void determineDistanceFromTarget() {
        double hopperPositionX = hopperPosition.getX();
        double hopperPositionY = hopperPosition.getY();
        Pose2d robotPose2d = swerveSubsystem.getPose();
        double robotPoseX = robotPose2d.getX();
        double robotPoseY = robotPose2d.getY();

        this.distanceFromTarget = Math
                .sqrt(Math.pow(hopperPositionX - robotPoseX, 2) + Math.pow(hopperPositionY - robotPoseY, 2));
    }

    // this is needed because the Hood/Needed Angle key on nt is the needed inital
    // angle of the ball to hit the target, not what the angle of the hood needs to be
    // for example, if the robot were right up against the hub, the ball would need
    // an angle of 85 degrees, which is what the value on nt would be. at 3 m from
    // the hub,
    // the ball would need an angle of 71.7262 degrees to hit the target.
    public double convertBallAngleToHoodAngle() {
        return Math.abs(neededHoodAngle - 90); // it's really that simple, the hood angle just needs to be the
                                               // complement of the ball angle to hit the target,
        // so if the ball needs to be launched at 70 degrees, the hood needs to be at 20
        // degrees, and if the ball needs to be launched at 85 degrees,
        // the hood needs to be at 5 degrees.
    }

    // this is so the hood only adjusts itself when the shooter is running
    public void autoAdjustHood(){
        setHoodPosition(convertBallAngleToHoodAngle());
    } 

    @Override
    public void periodic() {
        determineDistanceFromTarget();
        putDistanceFromTargetOnNT();
        retrieveNeededHoodAngleFromNT();
    }
}
