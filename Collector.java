package org.firstinspires.ftc.teamcode.opmodes8767;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Collector {

    public DcMotor collector = null;
    double CollectorSpeed = .75;

    LinearOpMode opMode;

    public Collector(LinearOpMode opMode){

        this.opMode = opMode;

        collector = opMode.hardwareMap.dcMotor.get("motor_collector");

        collector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void Collect () {

        collector.setPower(CollectorSpeed);

    }

    public void Abort () {

        collector.setPower(-CollectorSpeed);

    }

    public void Stop () {

        collector.setPower(0);

    }

    public void SpeedTest () {

        if (opMode.gamepad2.dpad_left) {
            collector.setPower(.35);
        }

        if (opMode.gamepad2.dpad_right) {
            collector.setPower(.45);
        }

        if (opMode.gamepad2.dpad_up) {
            collector.setPower(.5);
        }

        if (opMode.gamepad2.dpad_down) {
            collector.setPower(.4);
        }

    }

}