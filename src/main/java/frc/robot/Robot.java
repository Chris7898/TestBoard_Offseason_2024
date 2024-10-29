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
import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
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
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;




public class Robot extends TimedRobot {
 
  private static final String CANBus = "CANivore";
  private final TalonFX m_fx = new TalonFX(5, CANBus);

  private static final int deviceID = 6;
  private CANSparkMax m_motor;
  private SparkPIDController m_pidController;
  private RelativeEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;


  @Override
  public void robotInit()
  {
    m_motor = new CANSparkMax(deviceID, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();
    m_pidController = m_motor.getPIDController();
    m_encoder = m_motor.getEncoder();

    kP = 6e-5; 
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 0.000015; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);



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
motionMagicConfigs.MotionMagicCruiseVelocity = 70; // Target cruise velocity of 70 rps
motionMagicConfigs.MotionMagicAcceleration = 50; // Target acceleration of 50 rps/s
motionMagicConfigs.MotionMagicJerk = 1300; // Target jerk of 1300 rps/s/s (0.1 seconds)

m_fx.getConfigurator().apply(talonFXConfigs);
m_fx.getConfigurator().apply(motionMagicConfigs);
  }

  @Override
  public void robotPeriodic()
  {
    var Actual_RPM = m_fx.getVelocity();
    SmartDashboard.putNumber("Actual RPM", Actual_RPM.getValue());
  }
  
  @Override
  public void disabledInit()
  {

  }

  @Override
  public void disabledPeriodic()
  {

  }

  @Override
  public void autonomousInit()
  {
  
  }

  @Override
  public void autonomousPeriodic()
  {
  }

  @Override
  public void teleopInit()
  {
    SmartDashboard.putNumber("RPM", 0);

   
  }

  @Override
  public void teleopPeriodic()
  {
    double targetRPM = SmartDashboard.getNumber("Kraken RPM", 0);
    
    if (targetRPM >= 5000) {
       targetRPM = 0;
    }

        double rotationsPerSecond = targetRPM / 60; // converting from rotations per minute to per second.

    if (targetRPM <= 5000) {
       m_fx.setControl(new MotionMagicVelocityVoltage(rotationsPerSecond));
    }
    
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_pidController.setP(p); kP = p; }
    if((i != kI)) { m_pidController.setI(i); kI = i; }
    if((d != kD)) { m_pidController.setD(d); kD = d; }
    if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    
    double setPoint = SmartDashboard.getNumber("NEO RPM", 0);
    m_pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
    
    SmartDashboard.putNumber("SetPoint", setPoint);
    SmartDashboard.putNumber("ProcessVariable", m_encoder.getVelocity());
  }

  @Override
  public void testInit()
  {
    
  }

  @Override
  public void testPeriodic()
  {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic()
  {}
}

