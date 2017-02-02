package org.firstinspires.ftc.teamcode.opmodes8767;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class CapLift {

    public DcMotor capLift = null;

    LinearOpMode opMode;

    public CapLift(LinearOpMode opMode){   // constructor
        this.opMode = opMode;

        capLift = opMode.hardwareMap.dcMotor.get("motor_CapLift");

        capLift.setDirection(DcMotor.Direction.REVERSE);
        capLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void Lift(){
        capLift.setPower(.75);
    }

    public void Lower(){
        capLift.setPower(-.75);
    }

    public void Stop(){
        capLift.setPower(0);
    }
}
