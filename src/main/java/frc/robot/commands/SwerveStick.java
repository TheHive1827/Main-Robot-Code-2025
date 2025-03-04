package frc.robot.commands;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.*;

public abstract class SwerveStick extends Command {
    private final DriveSubsystem swerveStick;
    private final Supplier<Double> speedX;
    private final Supplier<Double> speedY;
    private final Supplier<Double> turningSpeedFunction;
    private final Supplier<Boolean> fieldOrientatonFunction;
    public SwerveStick(DriveSubsystem swerveStick, Supplier<Double> speedX, Supplier<Double> speedY, Supplier<Double> turningSpeedFunction, Supplier<Boolean> fieldOrientatonFunction) {
        this.swerveStick = swerveStick;
        this.speedX = speedX;
        this.speedY = speedY;
        this.turningSpeedFunction = turningSpeedFunction;
        this.fieldOrientatonFunction = fieldOrientatonFunction;
        addRequirements(swerveStick);
    }

    @Override
    public void initialize() {
        System.out.println("ElevatorJoystickCmd started!");
    }

    // @Override
    // public void execute() {

    //     double spdX = speedX.get();
    //     speedX = Math.abs(speedX) > OIConstants.kDriveDeadband ? speedX : 0.0;
    //     speedY = Math.abs(speedY) > OIConstants.kDriveDeadband ? speedY : 0.0;
    //     turningSpeed = Math.abs(turningSpeed) > OIConstants.kDriveDeadband ? turningSpeed 0.0;
    // } @TODO: RETURN TO THIS

    // @Override
    // public void end(boolean interrupted) {
    //     swerveStick.setMotor(0);
    //     System.out.println("ElevatorJoystickCmd ended!");
    // } @TODO: THIS TOO!

    @Override
    public boolean isFinished() {
        return false;
    }
}