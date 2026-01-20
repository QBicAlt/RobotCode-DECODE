package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.util.ElapsedTime;
import dev.nextftc.core.commands.Command;

public class waitForShots extends Command {

    private final double safetyTimeout;
    private final double clearDurationRequirement; // How long it must be empty to be "sure"

    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime clearTimer = new ElapsedTime();
    private boolean isClear = false;

    // Default constructor: 2.5s safety, must be clear for 0.2s to finish
    public waitForShots() {
        this(2.5, 0.2);
    }

    public waitForShots(double timeout, double clearDuration) {
        this.safetyTimeout = timeout;
        this.clearDurationRequirement = clearDuration;
    }

    @Override
    public void start() {
        timer.reset();
        clearTimer.reset();
        isClear = false;
    }

    @Override
    public void update() {
        boolean hasBall = LauncherOuttakeFuckingThing.INSTANCE.hasBall();

        if (hasBall) {
            // If we see a ball, reset the "clear" timer
            clearTimer.reset();
            isClear = false;
        } else {
            // If we don't see a ball, let the timer run
            isClear = true;
        }
    }

    @Override
    public boolean isDone() {
        // FINISH IF:
        // 1. The sensor has been clear continuously for the requirement time (0.2s)
        // 2. OR We hit the safety timeout (prevent infinite loop)

        boolean isEmptyConfirmed = isClear && (clearTimer.seconds() > clearDurationRequirement);
        boolean timedOut = timer.seconds() > safetyTimeout;

        return isEmptyConfirmed || timedOut;
    }
}