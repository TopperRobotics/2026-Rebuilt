package frc.robot.subsystems.mechanisms;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    
    // Motor controllers and related objects
    private SparkMax intakeArmLeftMotor;
    private RelativeEncoder intakeArmLeftMotorEncoder;
    private SparkMax intakeArmRightMotor; // the right motor is inverted
    private RelativeEncoder intakeArmRightMotorEncoder;
    private SparkClosedLoopController intakeArmLeftMotorPID;
    private SparkClosedLoopController intakeArmRightMotorPID;
    private SparkMax intakeRollerMotor;
    private enum IntakeArmState {
        IN, OUT
    }
    private IntakeArmState currentIntakeArmState = IntakeArmState.OUT;

    public IntakeSubsystem() {
        // Initialize intake arm motor
        intakeArmLeftMotor = new SparkMax(
            8,
            MotorType.kBrushless
        );

        intakeArmRightMotor = new SparkMax(
            11,
            MotorType.kBrushless
        );
        
        // Initialize intake roller motors
        intakeRollerMotor = new SparkMax(12, MotorType.kBrushless);

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
        
        config.inverted(true);
        
        // Apply configuration to left motor
        intakeArmLeftMotor.configure(
            config, 
            SparkMax.ResetMode.kResetSafeParameters, 
            SparkMax.PersistMode.kPersistParameters
        );

        // Apply configuration to right motor
        intakeArmRightMotor.configure(
            config, 
            SparkMax.ResetMode.kResetSafeParameters, 
            SparkMax.PersistMode.kPersistParameters
        );
        
        // Get encoder and PID references after configuration
        intakeArmLeftMotorEncoder = intakeArmLeftMotor.getEncoder();
        intakeArmLeftMotorPID = intakeArmLeftMotor.getClosedLoopController();

        intakeArmRightMotorEncoder = intakeArmRightMotor.getEncoder();
        intakeArmRightMotorPID = intakeArmRightMotor.getClosedLoopController();
        
        // Reset encoder position to zero
        intakeArmLeftMotorEncoder.setPosition(0);
        intakeArmRightMotorEncoder.setPosition(0);
    }
    
    // COMMANDS FOR INTAKE ROLLERS
    
    /**
     * Command to expel game piece from intake
     */
    public Command out() {
         return run(() -> {
            intakeRollerMotor.set(1); 
         });
    }
    
    /**
     * Command to intake game piece
     */
    public Command in() {
        return run(() -> {
            intakeRollerMotor.set(-0.75); 
        });
    }
    
    /**
     * Command to stop intake rollers
     */
    public Command stop() {
        return run(() -> {
            intakeRollerMotor.stopMotor();
        });
    }

    // ARM POSITION METHODS
    
    /**
     * Get current arm position as Rotation2d
     */
    public Rotation2d getPositionLeft() {
        return Rotation2d.fromDegrees(intakeArmLeftMotorEncoder.getPosition());
    }

    public Rotation2d getPositionRight() {
        return Rotation2d.fromDegrees(intakeArmRightMotorEncoder.getPosition());
    }
    
    /**
     * Calculate error between current position and goal position
     * @param goal Target position
     * @return Error as Rotation2d
     */
    public Rotation2d getError(Rotation2d goal) {
        Rotation2d currentPos = getPositionLeft();
        double error = currentPos.getDegrees() - goal.getDegrees();
        return Rotation2d.fromDegrees(error);
    }

    /**
     * Stops the arm wherever it is
     * @return
     */
    public Command stopArm() {
        return run(() -> {
            intakeArmLeftMotor.stopMotor();
            intakeArmRightMotor.stopMotor();
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
        if(goal == Constants.intakeArm.deployedPosition){
            currentIntakeArmState = IntakeArmState.OUT;
        } else if (goal == Constants.intakeArm.retractedPosition){
            currentIntakeArmState = IntakeArmState.IN;
        }
        intakeArmLeftMotorPID.setSetpoint(goal.getDegrees(), SparkBase.ControlType.kPosition);
        intakeArmRightMotorPID.setSetpoint(goal.getDegrees(), SparkBase.ControlType.kPosition);
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
            //goToPosition(goal); // REMOVE THIS IS FOR DEBUGGING ONLY
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

    public Command toggleIntakeArmDeployment(){
        return runOnce(() -> {
            if(currentIntakeArmState == IntakeArmState.OUT){
                goToPosition(Constants.intakeArm.retractedPosition);
                currentIntakeArmState = IntakeArmState.IN;
            } else {
                goToPosition(Constants.intakeArm.deployedPosition);
                currentIntakeArmState = IntakeArmState.OUT;
            }
        });
    }

    public void toggleIntakeOutness(){
        if(getPositionLeft().getDegrees() > Constants.intakeArm.retractedPosition.getDegrees()){
            runOnce(() -> {moveToPosition(Constants.intakeArm.deployedPosition, false);});
        } else {
            runOnce(() -> {moveToPosition(Constants.intakeArm.retractedPosition, false);});
        }
    }
    
    // PERIODIC METHOD
    @Override
    public void periodic() {
        // Update SmartDashboard with current arm position
        SmartDashboard.putNumber("Intake/Current Position Left Motor", getPositionLeft().getDegrees());
        SmartDashboard.putNumber("Intake/Current Position Right Motor", getPositionRight().getDegrees());
        SmartDashboard.putNumber("Intake/Target Left Motor", intakeArmLeftMotorPID.getSetpoint());
        SmartDashboard.putNumber("Intake/Target Right Motor", intakeArmLeftMotorPID.getSetpoint());
        // Optional: Add more debugging info
        // SmartDashboard.putNumber("Intake/Error", getError(getPosition()).getDegrees());
        // SmartDashboard.putBoolean("Intake/At Setpoint", 
        //     Math.abs(getError(getPosition()).getDegrees()) < Constants.Swerve.intake.tolerance);
    }
}