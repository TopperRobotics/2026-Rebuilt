package frc.robot.subsystems.mechanisms;

import com.revrobotics.spark.*;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{

    int PWMPort;
    Spark shooterMotor = new Spark(1);

    public ShooterSubsystem(int PWMPort){
        this.PWMPort = PWMPort;
    }

    public ShooterSubsystem(){}

    public Command bust(){
        return this.runOnce(() -> shooterMotor.set(1));
    }
    
    public Command edge(){
        return this.runOnce(() -> shooterMotor.stopMotor());
    }
    
}
