package frc.robot;

// import java.util.Random;

import org.littletonrobotics.urcl.URCL;

// import com.ctre.phoenix6.hardware.CANcoder;
// import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.Orchestra;

// import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform2d;
// import edu.wpi.first.math.geometry.Translation2d;

// import frc.robot.Const.*;

// import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.simulation.BatterySim;
// import edu.wpi.first.wpilibj.simulation.RoboRioSim;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.simulation.BatterySim;
// import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.sim.PhysicsSim;
// import frc.robot.subsystems.drivetrain.SwerveDrive;


public class Robot extends TimedRobot {
 
  private static final String CANBus = "CANivore";
  private final TalonFX m_fx = new TalonFX(1, CANBus);
    // private SwerveDrive drivetrain;
    // private Vision vision;

    // private XboxController controller;
    // // Limit max speed
    // private final double kDriveSpeed = 0.6;
    // /*___________________________________________VISION___________________________________________ */ 
    // // Rudimentary limiting of drivetrain acceleration
    // private SlewRateLimiter forwardLimiter = new SlewRateLimiter(1.0 / 0.6); // 1 / x seconds to 100%
    // private SlewRateLimiter strafeLimiter = new SlewRateLimiter(1.0 / 0.6);
    // private SlewRateLimiter turnLimiter = new SlewRateLimiter(1.0 / 0.33);

    // private Timer autoTimer = new Timer();
    // private Random rand = new Random(4512);
    // /*___________________________________________END___________________________________________ */



   
  @Override
  public void robotInit() {

/*NOTE: LOGGING EXPERIMENTAL THINGS! */
/*NOTE: These logging commands are for advantage scope. */
DataLogManager.start();
DriverStation.startDataLog(DataLogManager.getLog());

/*NOTE: These logging commands are for the CTRE Tuner X */
SignalLogger.setPath("/U/logs");
SignalLogger.start();

/*NOTE: These logging commands are for REV data */
URCL.start();
/*END LOGGING THINGS */

// /*_____SWERVE AND VISION_____*/
// drivetrain = new SwerveDrive();
// vision = new Vision();

// controller = new XboxController(0);
// /*END */

    var talonFXConfigs = new TalonFXConfiguration();


    // set slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    talonFXConfigs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    talonFXConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -40;

    talonFXConfigs.Voltage.PeakForwardVoltage = 8;
    talonFXConfigs.Voltage.PeakReverseVoltage = -8;

    // set Motion Magic settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    m_fx.getConfigurator().apply(talonFXConfigs);
    m_fx.getConfigurator().apply(motionMagicConfigs);

    // m_fllr.setControl(new Follower(m_fx.getDeviceID(), false));
  }

  @Override
  public void robotPeriodic() {

//     /*VISION AND SWERVE */
//  drivetrain.periodic();

//         // Correct pose estimate with vision measurements
//         var visionEst = vision.getEstimatedGlobalPose();
//         visionEst.ifPresent(
//                 est -> {
//                     var estPose = est.estimatedPose.toPose2d();
//                     // Change our trust in the measurement based on the tags we can see
//                     var estStdDevs = vision.getEstimationStdDevs(estPose);

//                     drivetrain.addVisionMeasurement(
//                             est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
//                 });

//         // Apply a random offset to pose estimator to test vision correction
//         if (controller.getBButtonPressed()) {
//             var trf =
//                     new Transform2d(
//                             new Translation2d(rand.nextDouble() * 4 - 2, rand.nextDouble() * 4 - 2),
//                             new Rotation2d(rand.nextDouble() * 2 * Math.PI));
//             drivetrain.resetPose(drivetrain.getPose().plus(trf), false);
//         }

//         // Log values to the dashboard
//         drivetrain.log();
//         /*END */
  }



  @Override
  public void autonomousInit() {
// /*___________________________________________VISION___________________________________________ */  

//         autoTimer.restart();
//         var pose = new Pose2d(1, 1, new Rotation2d());
//         drivetrain.resetPose(pose, true);
//         vision.resetSimPose(pose);
// /*___________________________________________END___________________________________________ */

    /*_____FUNNY_____ */
    Orchestra m_orchestra = new Orchestra();

    // Add a single device to the orchestra
    m_orchestra.addInstrument(m_fx);
    
    // Attempt to load the chrp
    var status = m_orchestra.loadMusic("/funny/seven.chrp");
    
    m_orchestra.play();
    
    if (!status.isOK()) {
       // log error
    }
    /*END FUNNY */
  }

  @Override
  public void autonomousPeriodic() {
  //   /*___________________________________________VISION___________________________________________ */  

  //           // translate diagonally while spinning
  //           if (autoTimer.get() < 10) {
  //             drivetrain.drive(0.5, 0.5, 0.5, false);
  //         } else {
  //             autoTimer.stop();
  //             drivetrain.stop();
  //         }
  //  /*___________________________________________END___________________________________________ */
       
  }


  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    double targetRPM = 100; // ONLY FOR TESTING!
    // double targetRPM = SmartDashboard.getNumber("RPM", 0);
    SmartDashboard.putNumber("RPM", targetRPM);
    double rotationsPerSecond = targetRPM / 60; // converting from rotations per minute to per second.
    m_fx.setControl(new VelocityVoltage(rotationsPerSecond));
    /*MOTION MAGIC */
    // m_fx.setControl(new MotionMagicVelocityVoltage(rotationsPerSecond));
  
  //     /*___________________________________________VISION___________________________________________ */  
  //        // We will use an "arcade drive" scheme to turn joystick values into target robot speeds
  //       // We want to get joystick values where pushing forward/left is positive
  //       double forward = -controller.getLeftY() * kDriveSpeed;
  //       if (Math.abs(forward) < 0.1) forward = 0; // deadband small values
  //       forward = forwardLimiter.calculate(forward); // limit acceleration
  //       double strafe = -controller.getLeftX() * kDriveSpeed;
  //       if (Math.abs(strafe) < 0.1) strafe = 0;
  //       strafe = strafeLimiter.calculate(strafe);
  //       double turn = -controller.getRightX() * kDriveSpeed;
  //       if (Math.abs(turn) < 0.1) turn = 0;
  //       turn = turnLimiter.calculate(turn);

  //       // Convert from joystick values to real target speeds
  //       forward *= Const.Swerve.kMaxLinearSpeed;
  //       strafe *= Const.Swerve.kMaxLinearSpeed;
  //       turn *= Const.Swerve.kMaxLinearSpeed;

  //       // Command drivetrain motors based on target speeds
  //       drivetrain.drive(forward, strafe, turn, true);
  //  /*___________________________________________END___________________________________________ */

  }

  @Override
  public void disabledInit() {}
  @Override
  public void disabledPeriodic() {

    // drivetrain.stop();
  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {
    PhysicsSim.getInstance().addTalonFX(m_fx, 0.001);
  }

  @Override
  public void simulationPeriodic() {
    PhysicsSim.getInstance().run();
    //   /*___________________________________________VISION___________________________________________ */  

    //         // Update drivetrain simulation
    //     drivetrain.simulationPeriodic();

    //     // Update camera simulation
    //     vision.simulationPeriodic(drivetrain.getSimPose());

    //     var debugField = vision.getSimDebugField();
    //     debugField.getObject("EstimatedRobot").setPose(drivetrain.getPose());
    //     debugField.getObject("EstimatedRobotModules").setPoses(drivetrain.getModulePoses());

    //     // Calculate battery voltage sag due to current draw
    //     RoboRioSim.setVInVoltage(
    //             BatterySim.calculateDefaultBatteryLoadedVoltage(drivetrain.getCurrentDraw()));
    // /*___________________________________________END___________________________________________ */

}
}