// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.Constants.ElevatorConstants;
// // import swervelib.encoders.SparkMaxEncoderSwerve;
// import frc.robot.Constants.OIConstants;

// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.sim.SparkAbsoluteEncoderSim;
// import com.revrobotics.sim.SparkMaxAlternateEncoderSim;
// import com.revrobotics.spark.SparkAbsoluteEncoder;
// import com.revrobotics.spark.SparkLowLevel;
// import com.revrobotics.spark.SparkMax;
// import edu.wpi.first.wpilibj.motorcontrol.Spark;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.SparkMaxAlternateEncoder;
// import com.revrobotics.*;
// import edu.wpi.first.wpilibj.GenericHID;
// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.button.Trigger;

// import com.revrobotics.spark.SparkLowLevel.MotorType;


// public class ElevatorSubsystem extends SubsystemBase {
//     static CommandXboxController ElevatorController = new CommandXboxController(0);
//   XboxController exampleXbox = new XboxController(0); // 0 is the USB Port to be used as indicated on the Driver Station
//   private final static SparkMax elevatorMotorLeader = new SparkMax(ElevatorConstants.ElevatorLeader, MotorType.kBrushless);
//   private final static SparkMax elevatorMotorFollower = new SparkMax(ElevatorConstants.ElevatorFollower, MotorType.kBrushless);
//   private final static AbsoluteEncoder motorLeaderEncoder = elevatorMotorLeader.getAbsoluteEncoder();
//   private final static AbsoluteEncoder motorFollowerEncoder = elevatorMotorLeader.getAbsoluteEncoder();
//   boolean EncoderNotReached = true;
//   boolean ButtonB = exampleXbox.getBButtonPressed();
//   static int ElevatorRawEncoderValue = 0;



//   /**
//    * Example command factory method.
//    *
//    * @return a command


//   /**
//    * An example method querying a boolean state of the subsystem (for example, a digital sensor).
//    *
//    * @return value of some boolean subsystem state, such as a digital sensor.
//    */
// //   public boolean exampleCondition() {
// //     // Query some boolean state, such as a digital sensor.
// //     return false;
// //   }

// //   @Override
// //   public void periodic() {
// //     // This method will be called once per scheduler run
// //   }

//   @Override
//   public void simulationPeriodic() {
//     // This method will be called once per scheduler run during simulation
//   }

// //   public void configureBindings() {
// //     // Deploy the intake with the X button
// //     if (ElevatorRawEncoderValue == 0) {
// //         ElevatorController.y().whileTrue(ElevatorCommand(0.5));
// //     } else {
// //         ;
// //     }
// //     if (ElevatorRawEncoderValue == 0) {
// //         ElevatorController.y().whileTrue(ElevatorCommand(-0.5));
// //     } else {
// //         ;
// //     }
// //   }


//   public Command ElevatorCommand(double speed) {
//     return runOnce(() -> elevatorMotorLeader.set(speed))
//         .andThen(runOnce(() ->elevatorMotorFollower.set(speed)));
//     }

//     public void configureBindings() {
//         // Deploy the intake with the X button
//             ElevatorController.y().whileTrue(ElevatorCommand(0.5));
//             ElevatorController.y().whileTrue(ElevatorCommand(-0.5));
//       }

// //   public Command goUp2Command() {
// //         return this.runOnce(() -> mainElevatorMotorLeader.set(1));
// //   }

// //   public void goUpCommand() {
// //     if (ElevatorRawEncoderValue < 0) {
// //         mainElevatorMotorLeader.set(-1);
// //         mainElevatorMotorFollower.set(-1);
// //     } else {
// //         mainElevatorMotorLeader.set(0);
// //         mainElevatorMotorFollower.set(0);
// //   }
// // }

// //   public void goDownCommand() {
// //     if (ElevatorRawEncoderValue < 0) {
// //         mainElevatorMotorLeader.set(-1);
// //         mainElevatorMotorFollower.set(-1);
// //     } else {
// //         mainElevatorMotorLeader.set(0);
// //         mainElevatorMotorFollower.set(0);
// //   }
// // }


// }