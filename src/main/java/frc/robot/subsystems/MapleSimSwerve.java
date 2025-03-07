// package frc.robot.subsystems;
// import static edu.wpi.first.units.Units.*;
// import edu.wpi.first.math.Matrix;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.SwerveModulePosition;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.math.numbers.N1;
// import edu.wpi.first.math.numbers.N3;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.Constants;
// import org.ironmaple.simulation.SimulatedArena;
// import org.ironmaple.simulation.drivesims.*;
// import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
// import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

// public class MapleSimSwerve implements DriveSubsystem {
//     private final SelfControlledSwerveDriveSimulation simulatedDrive;
//     private final Field2d field2d;

//     public MapleSimSwerve() {
//         // For your own code, please configure your drivetrain properly according to the documentation
//         final DriveTrainSimulationConfig config = DriveTrainSimulationConfig.Default();

//         // Creating the SelfControlledSwerveDriveSimulation instance
//         this.simulatedDrive = new SelfControlledSwerveDriveSimulation(
//                 new SwerveDriveSimulation(config, new Pose2d(0, 0, new Rotation2d())));

//         // Register the drivetrain simulation to the simulation world
//         SimulatedArena.getInstance().addDriveTrainSimulation(simulatedDrive.getDriveTrainSimulation());

//         // A field2d widget for debugging
//         field2d = new Field2d();
//         SmartDashboard.putData("simulation field", field2d);
//     }

//     @Override
//     public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
//         this.simulatedDrive.runChassisSpeeds(
//                 new ChassisSpeeds(translation.getX(), translation.getY(), rotation),
//                 new Translation2d(),
//                 fieldRelative,
//                 true);

    
// }
