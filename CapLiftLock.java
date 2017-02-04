package org.firstinspires.ftc.teamcode.opmodes8767;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class CapLiftLock {

    private Servo capLiftLock = null;

    LinearOpMode opMode;

    public CapLiftLock(LinearOpMode opMode){ // constructor
        this.opMode = opMode;

//        capLiftLock = opMode.hardwareMap.servo.get("motor_CapLiftLock");
    }

    public void Lift(){
        capLiftLock.setPosition(0);
    }

    public void Lower(){
        capLiftLock.setPosition(.5);
    }
}