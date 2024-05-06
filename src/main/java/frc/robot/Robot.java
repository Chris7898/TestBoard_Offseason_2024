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
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.05; // Add 0.05 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0; // no output for error derivative

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

  }

  @Override
  public void robotPeriodic() {

 
  }



  @Override
  public void autonomousInit() {


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

  }


  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    double targetRPM = 100; // NOTE: This is only for testing to see if the motor is cooked or Chris is just mid at programming.
    
    // double targetRPM = SmartDashboard.getNumber("RPM", 0);
    SmartDashboard.putNumber("RPM", targetRPM);
    double rotationsPerSecond = targetRPM / 60; // converting from rotations per minute to per second.
    m_fx.setControl(new VelocityVoltage(rotationsPerSecond));

    /*MOTION MAGIC */
    // m_fx.setControl(new MotionMagicVelocityVoltage(rotationsPerSecond));

  }

  @Override
  public void disabledInit() {}
  @Override
  public void disabledPeriodic() {
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
    

}
}