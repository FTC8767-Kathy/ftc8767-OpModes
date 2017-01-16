package org.firstinspires.ftc.teamcode.opmodes8767;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DexM_Hardware {

    DriveTrain DriveTrain = null;
    Collector collector = null;
    Launcher launcher = null;

    private ElapsedTime period  = new ElapsedTime();

    LinearOpMode opMode;

    public DexM_Hardware(){

    }

    public void init(LinearOpMode opMode) {

        this.opMode = opMode;
        DriveTrain = new DriveTrain(opMode);
        collector = new Collector(opMode);
        launcher = new Launcher(opMode);

    }

    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

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
