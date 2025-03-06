package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;
// import swervelib.encoders.SparkMaxEncoderSwerve;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxAlternateEncoderSim;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.*;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import com.revrobotics.spark.SparkLowLevel.MotorType;


public class ElevatorSubsystem extends SubsystemBase {
  XboxController exampleXbox = new XboxController(0); // 0 is the USB Port to be used as indicated on the Driver Station
  private final SparkMax elevatorMotorLeader = new SparkMax(ElevatorConstants.ElevatorLeader, MotorType.kBrushless);
  private final SparkMax elevatorMotorFollower = new SparkMax(ElevatorConstants.ElevatorFollower, MotorType.kBrushless);
  private final RelativeEncoder motorLeaderEncoder = elevatorMotorLeader.getEncoder();
  private final RelativeEncoder motorFollowerEncoder = elevatorMotorFollower.getEncoder();
  boolean EncoderNotReached = true;
  boolean ButtonB = exampleXbox.getBButtonPressed();
  int ElevatorRawEncoderValue = 0;


  // public ElevatorSubsystem() {
  //    new Trigger(ButtonB).onTrue(Commands.runOnce(() -> mainElevatorMotorLeader.set(1)));
  //    Trigger xButton = exampleXbox.getBButtonPressed(); // Creates a new Trigger object for the `X` button on exampleCommandController
  // }


  /**
   * Example command factory method.
   *
   * @return a command


  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


  private void schedule(Command command) {
    
  }




//   public Command goUp2Command() {
//         return this.runOnce(() -> mainElevatorMotorLeader.set(1));
//   }

//   public void goUpCommand() {
//     if (ElevatorRawEncoderValue < 0) {
//         mainElevatorMotorLeader.set(-1);
//         mainElevatorMotorFollower.set(-1);
//     } else {
//         mainElevatorMotorLeader.set(0);
//         mainElevatorMotorFollower.set(0);
//   }
// }

//   public void goDownCommand() {
//     if (ElevatorRawEncoderValue < 0) {
//         mainElevatorMotorLeader.set(-1);
//         mainElevatorMotorFollower.set(-1);
//     } else {
//         mainElevatorMotorLeader.set(0);
//         mainElevatorMotorFollower.set(0);
//   }
// }

public void setMotor(double Speed){
  elevatorMotorLeader.set(Speed);
  elevatorMotorFollower.set(Speed);
}

}