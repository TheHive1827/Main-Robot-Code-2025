package frc.robot.commands;
import frc.robot.Configs;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

public abstract class SwerveStick extends Command {
    private final DriveSubsystem swerveStick;
    private final Supplier<Double> speedX;
    private final Supplier<Double> speedY;
    private final Supplier<Double> turningSpeedFunction;
    private final SlewRateLimiter xLimiter,yLimiter,turningLimiter;
    private final Supplier<Boolean> fieldOrientatonFunction;
    public SwerveStick(DriveSubsystem swerveStick, Supplier<Double> speedX, 
    Supplier<Double> speedY, Supplier<Double> turningSpeedFunction, Supplier<Boolean> fieldOrientatonFunction, 
    Supplier<SlewRateLimiter> xLimiter, Supplier<SlewRateLimiter> yLimiter, Supplier <SlewRateLimiter> turningLimiter) {
        this.swerveStick = swerveStick;
        this.speedX = speedX;
        this.speedY = speedY;
        this.turningSpeedFunction = turningSpeedFunction;
        this.fieldOrientatonFunction = fieldOrientatonFunction;
        this.xLimiter = new SlewRateLimiter(AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        this.yLimiter = new SlewRateLimiter(AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        this.turningLimiter = new SlewRateLimiter(0);
        addRequirements(swerveStick);
    }

    @Override
    public void initialize() {
        System.out.println("ElevatorJoystickCmd started!");
    }

    @Override
    public void execute() {

        double spdX = speedX.get();
        double spdY = speedY.get();
        double turningSPD = turningSpeedFunction.get();
        spdX = Math.abs(spdX) > OIConstants.kDriveDeadband ? spdX : 0.0;
        spdY = Math.abs(spdY) > OIConstants.kDriveDeadband ? spdY : 0.0;
        turningSPD = Math.abs(turningSPD) > OIConstants.kDriveDeadband ? turningSPD : 0.0;
        ChassisSpeeds chassisSpeeds;
        if (fieldOrientatonFunction.get()){
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                spdX, spdY, turningSPD, swerveStick.geRotation2d()
            );
        }
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds); 
     } //@TODO: RETURN TO THIS

    @Override
    public void end(boolean interrupted) {
        swerveStick.setMotor(0);
        System.out.println("ElevatorJoystickCmd ended!");
    } //@TODO: THIS TOO!

    @Override
    public boolean isFinished() {
        return false;
    }
}