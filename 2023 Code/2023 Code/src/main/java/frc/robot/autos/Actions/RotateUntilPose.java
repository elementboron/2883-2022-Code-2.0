package frc.robot.autos.Actions;

import java.awt.event.ActionEvent;
import java.beans.PropertyChangeListener;
import java.util.Set;

import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.LimelightReader;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Action to wait for a given amount of time To use this Action, call runAction(new WaitAction(your_time))
 */
public class RotateUntilPose implements Command {

    private final Translation2d NoTranslation = new Translation2d(0, 0);

    private Swerve s_Swerve;

    public RotateUntilPose() {
        this.s_Swerve = s_Swerve;
    }

    @Override
    public void initialize(){};

    @Override
    public void execute() {
        s_Swerve.drive( NoTranslation,10.0, false, false );
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return null;
    }

    @Override
    public boolean isFinished() {

        return true;
        //return LimelightReader.Instance().botPose.exists();
    }
}