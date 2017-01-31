package org.firstinspires.ftc.teamcode.opmodes8767;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Launcher {

    private DcMotor launcher = null;
    private ElapsedTime runtime = new ElapsedTime();

    final static double AUTO_LAUNCH_TIME = 1.2;
//    final static int AUTO_LAUNCH_TARGET = (int) (1120 * 3) - 325; // 3360 (1120 for AM Motor)

    LinearOpMode opMode;

    public Launcher(LinearOpMode opMode){   // constructor
        this.opMode = opMode;

        launcher = opMode.hardwareMap.dcMotor.get("motor_launcher");
        launcher.setDirection(DcMotor.Direction.REVERSE);

        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void launch(){
        launcher.setPower(.75);
    }

    public void stop(){
        launcher.setPower(0);
    }

    public void autoLaunch(){
//        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        launcher.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        launcher.setTargetPosition(AUTO_LAUNCH_TARGET);

        opMode.telemetry.addData("Auto launching", "Please wait");
        launcherTelemetry();
        opMode.telemetry.update();

//        launch();

        runtime.reset();

        while(opMode.opModeIsActive() && runtime.seconds() < AUTO_LAUNCH_TIME) {
            launch();
        }
        stop();
    }

    public void launcherTelemetry() {
//        opMode.telemetry.addData("Target", AUTO_LAUNCH_TARGET);
        opMode.telemetry.addData("Current", launcher.getCurrentPosition());
    }
}
