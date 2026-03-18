package frc.robot.subsystems.mechanisms;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

public class ClimberSubsystem extends SubsystemBase {
    private SparkMax climberMotor;
    private RelativeEncoder climberMotorEncoder;
    private SparkClosedLoopController climberMotorPID;

    public ClimberSubsystem() {
        climberMotor = new SparkMax(13, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);

        config.encoder.positionConversionFactor(360.0 / Constants.climber.gearRatio).velocityConversionFactor(360.0 / 60.0);

        config.closedLoop.pid(Constants.climber.kP, Constants.climber.kI, Constants.climber.kD);

        climberMotor.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);

        climberMotorEncoder = climberMotor.getEncoder();
        climberMotorPID = climberMotor.getClosedLoopController();

        climberMotorEncoder.setPosition(0);

    }

    public Rotation2d getPosition() {
        return Rotation2d.fromDegrees(climberMotorEncoder.getPosition());
    }

    public Rotation2d getError(Rotation2d goal) {
        Rotation2d currentPos = getPosition();
        double error = currentPos.getDegrees() - goal.getDegrees();
        return Rotation2d.fromDegrees(error);
    }

    public Command stop(){
        return run(() -> {
            climberMotor.stopMotor();
        });
    }

    public void goToPosition(Rotation2d goal) {
        climberMotorPID.setSetpoint(goal.getDegrees(), SparkBase.ControlType.kPosition);
    }

    public Command runNegative(){
        return runOnce(() -> {
            climberMotor.set(-1);
        });
    }

    public Command runPositive(){
        return runOnce(() -> {
            climberMotor.set(1);
        });
    }

    public Command moveToPosition(Rotation2d goal) {
        return runOnce(() -> {
            // Log goal position to SmartDashboard for debugging
            SmartDashboard.putNumber("Climber/Goal Position", goal.getDegrees());
        })
        .andThen(
            // Run PID controller until position is reached
            run(() -> goToPosition(goal))
                .until(() -> 
                    Math.abs(getError(goal).getDegrees()) < Constants.climber.tolerance
                )
                // Timeout safety: stop after 1 second if position not reached
                .withTimeout(1.0)
        );
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Climber/Climber Position", climberMotorEncoder.getPosition());
    }
}
