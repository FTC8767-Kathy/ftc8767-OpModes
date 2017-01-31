package org.firstinspires.ftc.teamcode.opmodes8767;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Collector {

    private DcMotor collector = null;
    double COLLECTOR_SPEED = .75;

    LinearOpMode opMode;

    public ElapsedTime runtime = new ElapsedTime();


    public Collector(LinearOpMode opMode){   // constructor

        this.opMode = opMode;

        collector = opMode.hardwareMap.dcMotor.get("motor_collector");

        collector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void collect(double collectPower){
        collector.setPower(collectPower);
    }

    public void reverse(){
        collector.setPower(-COLLECTOR_SPEED);
    }

    public void stop(){
        collector.setPower(0);
    }

    public void collectTime(double collectSeconds){
        opMode.telemetry.addData("Collecting for", collectSeconds + "seconds");
        opMode.telemetry.update();

        runtime.reset();
        while (opMode.opModeIsActive() && (runtime.seconds() < collectSeconds)){
            collect(COLLECTOR_SPEED); //collectTime (Collects the ball)
        }
        stop();
    }

    public void speedTest() {
        if (opMode.gamepad2.dpad_left){
            collector.setPower(.35);
        }

        if (opMode.gamepad2.dpad_right){
            collector.setPower(.45);
        }

        if (opMode.gamepad2.dpad_up){
            collector.setPower(.5);
        }

        if (opMode.gamepad2.dpad_down){
            collector.setPower(.4);
        }

    }
}