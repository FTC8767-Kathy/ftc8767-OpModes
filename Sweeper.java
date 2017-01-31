package org.firstinspires.ftc.teamcode.opmodes8767;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Sweeper {

    private Servo sweeper = null;

    LinearOpMode opMode;

    public Sweeper(LinearOpMode opMode){   // constructor

        this.opMode = opMode;

        sweeper = opMode.hardwareMap.servo.get("motor_Sweeper");
        setSweeper0Percent();

        sweeper.setDirection(Servo.Direction.FORWARD);

    }

    public void setSweeper0Percent() {

        sweeper.setPosition(0);

    }

    public void setSweeper50Percent() {

        sweeper.setPosition(0.5);

    }

    public void setSweeper70Percent() {

        sweeper.setPosition(0.7);

    }

    public void sweeperTelemetry() {

        opMode.telemetry.addData("Sweeper Position: ", sweeper.getPosition());

    }

}