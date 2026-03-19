package frc.robot.subsystems.mechanisms;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    // bang bang
    double desiredFlywheelSpeed;
    SparkFlex shooterMotor = new SparkFlex(16, MotorType.kBrushless);
    RelativeEncoder shooterMotorEncoder = shooterMotor.getEncoder();
    SparkMax beltMotor = new SparkMax(14, MotorType.kBrushless);
    SparkMax feederMotor = new SparkMax(15, MotorType.kBrushless);

    public ShooterSubsystem() {
        this.desiredFlywheelSpeed = 50.96; // the ball needs to be launched at 8 m/s, with the formula v_ball = 1/2 RPM
                                           // * 2pi/60 * 3 (radius of wheel), which is rpm = 20 * v_ball / 60, we get 50.96 as the needed rpm
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast);
        shooterMotor.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        feederMotor.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        beltMotor.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
    }

    public void setDesiredFlywheelSpeed(double speed) {
        this.desiredFlywheelSpeed = speed;
    }

    public void run() {
        if (shooterMotorEncoder.getVelocity() < desiredFlywheelSpeed) {
            shooterMotor.set(-1);
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
        this.desiredFlywheelSpeed = desiredSpeed;
        return runOnce(() -> {
            run();
        });
    }

    public Command shoot(){
        this.desiredFlywheelSpeed = 50.96;
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
            beltMotor.set(-1);
        });
    }

    public Command stopShooting(){
        return runOnce(() -> {
            stop();
        });
    }

}
