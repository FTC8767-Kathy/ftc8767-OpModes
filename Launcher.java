package org.firstinspires.ftc.teamcode.opmodes8767;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Launcher {

    public DcMotor launcher = null;
    private ElapsedTime Runtime = new ElapsedTime();

    LinearOpMode opMode;

    public Launcher(LinearOpMode opMode){

        this.opMode = opMode;

        launcher = opMode.hardwareMap.dcMotor.get("motor_launcher");

        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void launch(){

        launcher.setPower(-1);

    }

    public void Idle(){

        launcher.setPower(0);

    }

    public void AutoLaunch(double LaunchTime){
        Runtime.reset();

        while(Runtime.seconds() < LaunchTime) {

            launch();
        }

        Idle();

    }

}
