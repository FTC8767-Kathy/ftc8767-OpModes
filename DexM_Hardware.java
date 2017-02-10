package org.firstinspires.ftc.teamcode.opmodes8767;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DexM_Hardware {

    DriveTrain driveTrain = null;
    Collector collector = null;
    Launcher launcher = null;
    ButtonPusher beaconTrigger = null;
    Sweeper sweeper = null;
   CapLift capLift = null;
    CapLiftLock capLiftLock = null;

    private ElapsedTime period  = new ElapsedTime();

    LinearOpMode opMode;

    public DexM_Hardware(){   // constructor

    }

    public void init(LinearOpMode opMode) {
        this.opMode = opMode;

        driveTrain = new DriveTrain(opMode);
        collector = new Collector(opMode);
        launcher = new Launcher(opMode);
        beaconTrigger = new ButtonPusher(opMode);
        sweeper = new Sweeper(opMode);
        capLift = new CapLift(opMode);
        capLiftLock = new CapLiftLock(opMode);
    }

    // for Teleop only - free resources briefly during each cycle
    public void waitForTick(long periodMs) {
        long  remaining = periodMs - (long)period.milliseconds();
        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
        period.reset();
    }
}
