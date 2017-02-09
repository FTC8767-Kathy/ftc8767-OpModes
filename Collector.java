package org.firstinspires.ftc.teamcode.opmodes8767;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Collector {

    private DcMotor collector = null;
    double COLLECTOR_SPEED = 1;

    LinearOpMode opMode;

    public ElapsedTime runtime = new ElapsedTime();


    public Collector(LinearOpMode opMode) {   // constructor

        this.opMode = opMode;

        collector = opMode.hardwareMap.dcMotor.get("motor_collector");

        collector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void collect(double collectPower) {
        collector.setPower(collectPower);
    }

    public void stop() {
        collector.setPower(0);
    }

    public void collectTime(double collectSeconds) {
        opMode.telemetry.addData("Collecting for", collectSeconds + "seconds");
        opMode.telemetry.update();

        runtime.reset();
        while (opMode.opModeIsActive() && (runtime.seconds() < collectSeconds)) {
            collect(COLLECTOR_SPEED); //collectTime (Collects the ball)
        }
        stop();
    }

}