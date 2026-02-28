package frc.robot.subsystems.mechanisms;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    // bang bang
    double desiredFlywheelSpeed;
    SparkFlex shooterMotor = new SparkFlex(12, MotorType.kBrushless);
    RelativeEncoder shooterMotorEncoder = shooterMotor.getEncoder();

    public ShooterSubsystem() {
        this.desiredFlywheelSpeed = 50.96; // the ball needs to be launched at 8 m/s, with the formula v_ball = 1/2 RPM
                                           // * 2pi/60 * 3 (radius of wheel), which is rpm = 20 * v_ball / 60, we get 50.96 as the needed rpm
    }

    public void setDesiredFlywheelSpeed(double speed) {
        this.desiredFlywheelSpeed = speed;
    }

    public void run() {
        if (shooterMotorEncoder.getVelocity() < desiredFlywheelSpeed) {
            shooterMotor.set(1);
        } else {
            shooterMotor.stopMotor();
        }
    }

}
