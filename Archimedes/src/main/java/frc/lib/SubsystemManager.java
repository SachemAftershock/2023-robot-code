package frc.lib;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/**
 * Class to Manage All Subsystems
 * 
 * @author Shreyas Prasad
 */
public class SubsystemManager {
    private List<AftershockSubsystem> mAllSubsystems;

    /**
     * Subsystem Manager Constructor
     */
    public SubsystemManager(AftershockSubsystem... allSubsystems) {
        mAllSubsystems = Arrays.asList(allSubsystems);
    }

    /**
     * Initializes all Subsystems
     */
    public void initialize() {
        mAllSubsystems.forEach((subsystem) -> {
            System.out.println(subsystem + " " + Timer.getFPGATimestamp());
            subsystem.initialize();
        });
    }

    /**
     * Outputs Telemetry of all Subsystems
     */
    public void outputTelemetry() {
        mAllSubsystems.forEach(AftershockSubsystem::outputTelemetry);
    }

    /**
     * Checks all Subsystems and Reports all Subsystems that fail systems check
     * 
     * @return true if all subsystems pass systems check; false otherwise
     */
    public boolean checkSystems() {
        boolean ret = true;
        for (AftershockSubsystem subsystem : mAllSubsystems) {
            boolean subsystemPassedCheck = subsystem.checkSystem();
            if (!subsystemPassedCheck) {
                DriverStation.reportWarning(subsystem.getName() + " FAILED SYSTEM CHECK", false);
            }
            ret &= subsystemPassedCheck;
        }
        return ret;
    }

    /**
     * Gets list of all Subsystems
     * 
     * @return List of all Subsystems
     */
    public List<AftershockSubsystem> getSubsystems() {
        return mAllSubsystems;
    }
}