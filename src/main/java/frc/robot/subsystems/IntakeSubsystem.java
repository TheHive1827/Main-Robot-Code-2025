package frc.robot.subsystems;



import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    // initialize the intake motor
    private final SparkMax intakeMotor;

    // Creates a new IntakeSubsystem
    public IntakeSubsystem() {
        intakeMotor = new SparkMax(IntakeConstants.IntakeMotorID, MotorType.kBrushless);
    }

    @Override
    public void periodic() {

    }

    public void runForward() {
        intakeMotor.setVoltage(0);
    }
// change voltage to desired output
    public void runBackwards() {
        intakeMotor.setVoltage(0);
    }

    public void stop() {
        intakeMotor.stopMotor();
    }

    // command for running intake
    public Command load() {
        return Commands.startEnd(this::runForward, this::stop, this);
    }

    // command for releasing algae
    public Command release() {
        return Commands.startEnd(this::runBackwards, this::stop, this);
    }
}
