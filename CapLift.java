package org.firstinspires.ftc.teamcode.opmodes8767;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class CapLift {

    private DcMotor capLift = null;

    LinearOpMode opMode;

    public CapLift(LinearOpMode opMode){   // constructor
        this.opMode = opMode;

        capLift = opMode.hardwareMap.dcMotor.get("motor_CapLift");

        capLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void Lift(){
        capLift.setPower(-opMode.gamepad2.left_stick_y);
    }

}
