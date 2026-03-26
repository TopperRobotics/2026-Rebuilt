package frc.robot.subsystems.mechanisms;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveDrive;

public class ShooterSubsystem extends SubsystemBase {

    private final double gravity = 9.81;
    private final double hoodAngle = Math.toRadians(75);
    private final double robotHopperHeightDifference = 1.777998984; // Meters (Target Height - Shooter Height)
    private final double flywheelRadius = 0.0762; // 3 inches in meters
    
    double desiredFlywheelSpeed;
    double distanceFromTarget = 0.000;
    Translation2d hopperPosition = new Translation2d();
    Translation2d[] hopperPositions;
    SwerveSubsystem swerveSubsystem;
    
    SparkFlex shooterMotor = new SparkFlex(16, MotorType.kBrushless);
    RelativeEncoder shooterMotorEncoder = shooterMotor.getEncoder();
    SparkClosedLoopController shooterPID;
    SparkMax beltMotor = new SparkMax(14, MotorType.kBrushless);
    SparkMax feederMotor = new SparkMax(15, MotorType.kBrushless);

    public ShooterSubsystem(Translation2d[] hopperPositions, SwerveSubsystem swerveSubsystem) {
        this.hopperPositions = hopperPositions;
        this.swerveSubsystem = swerveSubsystem;
        
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast);

        // Configure encoder settings
        config.encoder
            .positionConversionFactor(360.0 / 1)
            .velocityConversionFactor((360.0 / 1) / 60.0);
        
        // Configure PID controller settings
        config.closedLoop
            .pid(0.01, 0, 0);

        shooterMotor.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        feederMotor.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        beltMotor.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        

        desiredFlywheelSpeed = 50.96; 
    }

    public void run() { // bang bang
        if (shooterMotorEncoder.getVelocity() > desiredFlywheelSpeed) {
            shooterMotor.set(-1); // motor is inverted
        } else {
            shooterMotor.stopMotor();
        }
    }

    public void stop() {
        shooterMotor.stopMotor();
        beltMotor.stopMotor();
        feederMotor.stopMotor();
    }

    public Command shoot(double desiredSpeed){
        desiredFlywheelSpeed = desiredSpeed;
        return run(() -> {
            run();
        });
    }

    public Command shootAutoFullPower(){
        return shoot(-1).andThen(new WaitCommand(0.4)).andThen(runFeeder()).andThen(runConveyor());
    }

    public Command runEntireShooterReverse(){
        return runOnce(() -> {
            shooterMotor.set(-0.4);
            beltMotor.set(-0.4);
            feederMotor.set(0.4);
        });
    }

    public Command shoot(){
        return runOnce(() -> {
            run();
        });
    }

    public Command runFeeder(){
        return runOnce(() -> {
            feederMotor.set(1);
        });
    }

    public Command runConveyor(){
        return runOnce(()->{
            beltMotor.set(-0.8);
        });
    }

    public Command runConveyor(double speed){
        return runOnce(() -> {
            beltMotor.set(speed);
        });
    }

    public Command runConveyorReverse(){
        return runOnce(()->{
            beltMotor.set(1);
        });
    }

    public Command stopConveyor(){
        return runOnce(() -> {
            beltMotor.stopMotor();
        });
    }

    public Command stopShooting(){
        return runOnce(() -> {
            stop();
        });
    }

    // x is robot pose x, y is robot pose y, this method will return the closest hopper position to the robot, which will be used to determine the distance from the target
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
        Pose2d robotPose2d = swerveSubsystem.getPose();
        // Update the hopper we are actually looking at based on current robot position
        this.hopperPosition = determineClosestTranslation2d(hopperPositions, robotPose2d.getX(), robotPose2d.getY());
        
        this.distanceFromTarget = hopperPosition.getDistance(robotPose2d.getTranslation());
    }

    public void determineNeededFlywheelSpeedDistanceBased() {
        // Projectile Motion Equation for Exit Velocity (v):
        // v = sqrt( (g * d^2) / (2 * cos^2(theta) * (d * tan(theta) - h)) )
        
        double d = this.distanceFromTarget;
        double h = robotHopperHeightDifference;
        double theta = hoodAngle;

        // Calculate required exit velocity in m/s
        double numerator = gravity * Math.pow(d, 2);
        double denominator = 2 * Math.pow(Math.cos(theta), 2) * (d * Math.tan(theta) - h);

        if (denominator <= 0) {
            SmartDashboard.putBoolean("Flywheel Calulation Possible", false);
            // Robot is either too close or the math is impossible for this angle
            return;
        }
        SmartDashboard.putBoolean("Flywheel Calulation Possible", true);

        double vBall = Math.sqrt(numerator / denominator);

        // Convert vBall (m/s) to RPM using your provided formula:
        // RPM = (vBall * 2 * 60) / (2 * PI * radius)
        desiredFlywheelSpeed = -((vBall * 120.0) / (2 * Math.PI * flywheelRadius));
    }

    @Override
    public void periodic() {
        determineDistanceFromTarget();
        determineNeededFlywheelSpeedDistanceBased();
        
        // Optional: Send data to SmartDashboard to debug
        SmartDashboard.putNumber("Distance to Target", distanceFromTarget);
        SmartDashboard.putNumber("Desired RPM", desiredFlywheelSpeed);
        SmartDashboard.putNumber("Actual Flywheel Velocity", shooterMotorEncoder.getVelocity());
    }

}
