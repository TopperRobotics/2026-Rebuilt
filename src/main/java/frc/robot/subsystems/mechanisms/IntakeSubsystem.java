package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.intakeArm;

public class IntakeSubsystem extends SubsystemBase {
    
    // Motor controllers and related objects
    private SparkMax intakeArmMotor;
    private RelativeEncoder intakeArmEncoder;
    private SparkClosedLoopController intakeArmPID;
    private SparkFlex intakeMotor;

    public IntakeSubsystem() {
        // Initialize intake arm motor
        intakeArmMotor = new SparkMax(
            5,
            MotorType.kBrushless
        );
        
        // Initialize intake roller motors
        intakeMotor = new SparkFlex(4, MotorType.kBrushless);

        // Configure motor settings using SparkMaxConfig
        SparkMaxConfig config = new SparkMaxConfig();
        config
            .idleMode(IdleMode.kBrake);
        
        // Configure encoder settings
        config.encoder
            .positionConversionFactor(360.0 / Constants.intakeArm.gearRatio)
            .velocityConversionFactor((360.0 / Constants.intakeArm.gearRatio) / 60.0);
        
        // Configure PID controller settings
        config.closedLoop
            .pid(Constants.intakeArm.kP, Constants.intakeArm.kI, Constants.intakeArm.kD);
        
        // Apply configuration to motor
        intakeArmMotor.configure(
            config, 
            SparkMax.ResetMode.kResetSafeParameters, 
            SparkMax.PersistMode.kPersistParameters
        );
        
        // Get encoder and PID references after configuration
        intakeArmEncoder = intakeArmMotor.getEncoder();
        intakeArmPID = intakeArmMotor.getClosedLoopController();
        
        // Reset encoder position to zero
        intakeArmEncoder.setPosition(0);
    }
    
    // COMMANDS FOR INTAKE ROLLERS
    
    /**
     * Command to expel game piece from intake
     */
    public Command out() {
         return run(() -> {
            intakeMotor.set(-0.4); 
         });
    }
    
    /**
     * Command to intake game piece
     */
    public Command in() {
        return run(() -> {
            intakeMotor.set(0.4); 
        });
    }
    
    /**
     * Command to stop intake rollers
     */
    public Command stop() {
        return run(() -> {
            intakeMotor.stopMotor();
        });
    }

    // ARM POSITION METHODS
    
    /**
     * Get current arm position as Rotation2d
     */
    public Rotation2d getPosition() {
        return Rotation2d.fromDegrees(intakeArmEncoder.getPosition());
    }
    
    /**
     * Calculate error between current position and goal position
     * @param goal Target position
     * @return Error as Rotation2d
     */
    public Rotation2d getError(Rotation2d goal) {
        Rotation2d currentPos = getPosition();
        double error = currentPos.getDegrees() - goal.getDegrees();
        return Rotation2d.fromDegrees(error);
    }

    public Command stopArm() {
        return run(() -> {
            intakeArmMotor.stopMotor();
        });
    }

    public Command setArmPositionToNumberFromSmartDashboard() {
        double setPoint = SmartDashboard.getNumber("Intake/Desired Arm Position", 0);
        return run(() -> {
            moveToPosition(new Rotation2d(Math.toRadians(setPoint)), false);
        });
    }
    
    /**
     * Run PID controller to move arm to target position
     * @param goal Target position in degrees
     */
    public void goToPosition(Rotation2d goal) {
        intakeArmPID.setSetpoint(goal.getDegrees(), SparkBase.ControlType.kPosition);
    }
    
    /**
     * Command to move arm to target position with optional intake activation
     * @param goal Target position
     * @param startIntake Whether to run intake when position is reached
     * @return Command sequence
     */
    public Command moveToPosition(Rotation2d goal, boolean startIntake) {
        return runOnce(() -> {
            // Log goal position to SmartDashboard for debugging
            SmartDashboard.putNumber("Intake/Goal Position", goal.getDegrees());
        })
        .andThen(
            // Run PID controller until position is reached
            run(() -> goToPosition(goal))
                .until(() -> 
                    Math.abs(getError(goal).getDegrees()) < Constants.intake.tolerance
                )
                // Timeout safety: stop after 1 second if position not reached
                .withTimeout(1.0)
        )
        .andThen(
            // After reaching position, optionally start intake
            startIntake ? in() : stop()
        );
    }
    
    // PERIODIC METHOD
    
    @Override
    public void periodic() {
        // Update SmartDashboard with current arm position
        SmartDashboard.putNumber("Intake/Current Position", getPosition().getDegrees());
        SmartDashboard.putNumber("Intake/Target", intakeArmPID.getSetpoint());
        // Optional: Add more debugging info
        // SmartDashboard.putNumber("Intake/Error", getError(getPosition()).getDegrees());
        // SmartDashboard.putBoolean("Intake/At Setpoint", 
        //     Math.abs(getError(getPosition()).getDegrees()) < Constants.Swerve.intake.tolerance);
    }
}