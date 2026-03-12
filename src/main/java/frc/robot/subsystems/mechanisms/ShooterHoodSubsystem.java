package frc.robot.subsystems.mechanisms;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class ShooterHoodSubsystem extends SubsystemBase {

    private double distanceFromTarget = 0.000;
    private double neededHoodAngle = Constants.shooterHood.reverseSoftLimit;

    private Translation2d hopperPosition = new Translation2d();
    private SwerveSubsystem swerveSubsystem;

    private SparkMax hoodMotor;
    private RelativeEncoder hoodMotorEncoder;
    private SparkClosedLoopController hoodPID;

    public ShooterHoodSubsystem(Translation2d[] hopperPositions, SwerveSubsystem swerveSubsystem) {
        this.hopperPosition = determineClosestTranslation2d(hopperPositions, swerveSubsystem.getPose().getX(), swerveSubsystem.getPose().getY());
        this.swerveSubsystem = swerveSubsystem;

        hoodMotor = new SparkMax(7, MotorType.kBrushless);

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

    public Translation2d determineClosestTranslation2d(Translation2d[] translations, double x, double y) {
                if (translations == null || translations.length == 0) {
                        System.out.println("shooterhoodsubsystem determineclosesttranslation2d translations parameter null or empty");
                        return null;
                }

                Translation2d closest = translations[0];
                double minDistSq = Math.pow(closest.getX() - x, 2) + Math.pow(closest.getY() - y, 2);

                for (int i = 1; i < translations.length; i++) {
                        Translation2d current = translations[i];
                        double distSq = Math.pow(current.getX() - x, 2) + Math.pow(current.getY() - y, 2);
                        if (distSq < minDistSq) {
                                minDistSq = distSq;
                                closest = current;
                        }
                }

                return closest;
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

    public void autoAdjustHood(){
        //
    }

    public Command moveToPosition(Rotation2d goal) {
        return runOnce(() -> {
            setHoodPosition(goal.getDegrees());
        });
    }

    public double getHoodPosition(){
        return this.hoodMotorEncoder.getPosition();
    }

    @Override
    public void periodic() {
        // determineDistanceFromTarget();
        // putDistanceFromTargetOnNT();
        // retrieveNeededHoodAngleFromNT();
        SmartDashboard.putNumber("Shooter Hood/Current Position", hoodMotorEncoder.getPosition());
    }
}
