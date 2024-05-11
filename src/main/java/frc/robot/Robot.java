package frc.robot;

import org.littletonrobotics.urcl.URCL;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.io.File;
import java.io.IOException;
import swervelib.parser.SwerveParser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
 
  // private static final String CANBus = "CANivore";
  // private final TalonFX m_fx = new TalonFX(1, CANBus);
  private static Robot   instance;
  private        Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Timer disabledTimer;

  public Robot()
  {
    instance = this;
  }

  public static Robot getInstance()
  {
    return instance;
  }

   
  @Override
  public void robotInit() {


DataLogManager.start();
DriverStation.startDataLog(DataLogManager.getLog());

SignalLogger.setPath("/U/logs");
SignalLogger.start();

URCL.start();


    // var talonFXConfigs = new TalonFXConfiguration();
    // var slot0Configs = talonFXConfigs.Slot0;
    // slot0Configs.kS = 0.05; // Add 0.05 V output to overcome static friction
    // slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    // slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
    // slot0Configs.kI = 0; // no output for integrated error
    // slot0Configs.kD = 0; // no output for error derivative

    // talonFXConfigs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    // talonFXConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -40;

    // talonFXConfigs.Voltage.PeakForwardVoltage = 8;
    // talonFXConfigs.Voltage.PeakReverseVoltage = -8;

    // // set Motion Magic settings
    // var motionMagicConfigs = talonFXConfigs.MotionMagic;
    // motionMagicConfigs.MotionMagicCruiseVelocity = 70; // Target cruise velocity of 70 rps
    // motionMagicConfigs.MotionMagicAcceleration = 50; // Target acceleration of 50 rps/s
    // motionMagicConfigs.MotionMagicJerk = 1300; // Target jerk of 1300 rps/s/s (0.1 seconds)

    // m_fx.getConfigurator().apply(talonFXConfigs);
    // m_fx.getConfigurator().apply(motionMagicConfigs);

  }

  @Override
  public void robotPeriodic() {

 
  }



  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {   

  }


  @Override
  public void teleopInit() {
    SmartDashboard.putNumber("RPM", 0);
  }

  @Override
  public void teleopPeriodic() {
    // double targetRPM = SmartDashboard.getNumber("RPM", 0);
    // double rotationsPerSecond = targetRPM / 60; // converting from rotations per minute to per second.
    // // m_fx.setControl(new VelocityVoltage(rotationsPerSecond));

    // /*MOTION MAGIC */
    // m_fx.setControl(new MotionMagicVelocityVoltage(rotationsPerSecond));

  }

  @Override
  public void disabledInit() {}
  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}