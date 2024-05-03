package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class RobotContainer {

    public void autonomousInit() {
        CommandScheduler.getInstance().cancelAll();
        // Schedule autonomous command here
    }

    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
        // Schedule teleop command here
    }

  

  public Command getAutonomousCommand() {
    return null;
        
    }
}