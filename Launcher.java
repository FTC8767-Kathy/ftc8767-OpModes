package org.firstinspires.ftc.teamcode.opmodes8767;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Launcher {

    private DcMotor launcher = null;
    private ElapsedTime runtime = new ElapsedTime();

    final static double AUTO_LAUNCH_TIME = 1.0;

    LinearOpMode opMode;

    public Launcher(LinearOpMode opMode){   // constructor
        this.opMode = opMode;

        launcher = opMode.hardwareMap.dcMotor.get("motor_launcher");
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void launch(){
        launcher.setPower(-1);
    }

    public void stop(){
        launcher.setPower(0);
    }

    public void autoLaunch(){
        opMode.telemetry.addData("Auto launching", "Please wait");
        opMode.telemetry.update();

        runtime.reset();

        while(opMode.opModeIsActive() && runtime.seconds() < AUTO_LAUNCH_TIME) {
            launch();
        }
        stop();
    }
}
