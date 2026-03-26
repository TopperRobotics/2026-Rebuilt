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
import frc.robot.Constants.shooterHood;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class ShooterHoodSubsystem extends SubsystemBase {

    private double distanceFromTarget = 0.000;
    private double neededHoodAngle = -20;
    private double launchSpeed = 50.96;

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

        config.smartCurrentLimit(40, 40);

        config.closedLoopRampRate(0.5);

        config.softLimit.forwardSoftLimit(0);
        config.softLimit.reverseSoftLimit(-35);

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

        //SmartDashboard.putNumber("Shooter Hood/Distance From Target", 0.0);
    }

    // this isn't working, the motor is giving up after around a second and the output current falls to 0
    // TODO: replace the breaker on the pdp
    public void setHoodPosition(double positionDegrees) {
        hoodPID.setSetpoint(positionDegrees, SparkBase.ControlType.kPosition);
    }

    public void putDistanceFromTargetOnNT() {
        SmartDashboard.putNumber("Hood/Distance From Target", distanceFromTarget);
    }

    public void retrieveNeededHoodAngleFromNT() {
        double neededAngle = SmartDashboard.getNumber("Hood/Needed Angle", -20);
        if(neededAngle != -1){ // -1 means that at the robot's current distance from the target, no angle could be calculated to get the ball in the target
            this.neededHoodAngle = neededAngle;
        } else {
            this.neededHoodAngle = 0;
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

    public void determineHoodAngle(){
        double hoodAngle = 0.5*Math.pow(Math.sin((9.8*distanceFromTarget)/(Math.pow(launchSpeed, 2))), -1);
        this.neededHoodAngle = hoodAngle;
    }

    public void autoAdjustHood(){
        //determineDistanceFromTarget();
        determineHoodAngle();
        setHoodPosition(-neededHoodAngle);
    }

    public Command moveToPosition(Rotation2d goal) {
        return runOnce(() -> {
            setHoodPosition(goal.getDegrees());
        });
    }

    public Command moveToPosition(double goalDegrees) {
        return runOnce(() -> {
            setHoodPosition(goalDegrees);
        });
    }

    public double getHoodPosition(){
        return this.hoodMotorEncoder.getPosition();
    }

    public Command stopHood(){
        return runOnce(() -> {
            hoodMotor.stopMotor();
        });
    }

    @Override
    public void periodic() {
        //this.distanceFromTarget = SmartDashboard.getNumber("Shooter Hood/Distance From Target", distanceFromTarget);
        //autoAdjustHood();
        SmartDashboard.putNumber("Shooter Hood/Needed Angle", neededHoodAngle);
        SmartDashboard.putNumber("Shooter Hood/Current Position", hoodMotorEncoder.getPosition());
        SmartDashboard.putNumber("Shooter Hood/Current", hoodMotor.getOutputCurrent());
        SmartDashboard.putNumber("Shooter Hood/Temperature", hoodMotor.getMotorTemperature());
        SmartDashboard.putNumber("Shooter Hood/PID Setpoint", hoodPID.getSetpoint());
    }
}
