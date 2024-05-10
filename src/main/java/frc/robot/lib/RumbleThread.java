package frc.robot.lib;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RumbleThread {

    private static RumbleThread sInstance = null;
    private Notifier mNotifier;
    private RumbleType mType = RumbleType.kBothRumble;
    private CommandXboxController[] mControllers;

    public enum ControllersToRumble {
        DRIVER,
        CODRIVER,
        ALL
    }

    // Get subsystem
    public static RumbleThread getInstance() {
        if (sInstance == null) sInstance = new RumbleThread();
        return sInstance;
    }

    private RumbleThread() {
        mNotifier = new Notifier(this::stopAllRumbles);
        mNotifier.stop();
    }

    public void bindControllers(CommandXboxController... controllers) {
        mControllers = controllers;
    }

    private void stopAllRumbles() {

        if(mControllers == null) return;
        for(CommandXboxController c: mControllers) {
            c.getHID().setRumble(mType, 0);
        }
        
    }

    public void setRumble(RumbleType type, double amount, double rumbleTime, ControllersToRumble controllersToRumble) {

        if(mControllers == null) return;
        stopAllRumbles();

        switch (controllersToRumble) {
            case DRIVER:
                if(mControllers.length > 0) mControllers[0].getHID().setRumble(type, amount);
                break;
                
            case CODRIVER:
                if(mControllers.length > 1 && mControllers[1] != null) mControllers[1].getHID().setRumble(type, amount);
                break;
        
            default:
                for (CommandXboxController c : mControllers) { c.getHID().setRumble(type, amount); }
                break;
        }

        mNotifier.startSingle(rumbleTime);

    }
}
