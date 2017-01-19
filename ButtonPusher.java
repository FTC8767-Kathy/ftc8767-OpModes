package org.firstinspires.ftc.teamcode.opmodes8767;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ButtonPusher {

    private Servo pusherLeft = null;
    private Servo pusherRight = null;

    private ColorSensor colorSensor;    // Hardware Device Object

    LinearOpMode opMode;

    public ElapsedTime runtime = new ElapsedTime();

    final static int RED = 1;
    final static int BLUE = 2;

    final double START_LEFT_POSITION = 1.0;
    final double START_RIGHT_POSITION = 0.0;

    final double PUSH_LEFT_POSITION = 0.0;
    final double PUSH_RIGHT_POSITION = 1.0;

    final int COLOR_SEPARATION = 7;

    public ButtonPusher(LinearOpMode opMode){   // constructor
        this.opMode = opMode;

        pusherLeft = opMode.hardwareMap.servo.get("pusherLeft");
        pusherRight = opMode.hardwareMap.servo.get("pusherRight");

        pusherLeft.setPosition(START_LEFT_POSITION);
        pusherRight.setPosition(START_RIGHT_POSITION);

        colorSensor = opMode.hardwareMap.colorSensor.get("colorSensor");
        colorSensor.enableLed(true);                        // ****** should it be on or not? ******
    }

   public void pushCorrectButton (int allianceColor){ // this assumes color sensor is on the right
       int beaconColor = getBeaconColor();
       if (beaconColor == allianceColor){
           pushRightButton();
       }
       else{
           pushLeftButton();
       }
       opMode.telemetry.addData("Alliance Color", allianceColor);
       opMode.telemetry.addData("Beacon Color", beaconColor);
       opMode.telemetry.addData("Red value", colorSensor.red());
       opMode.telemetry.addData("Blue value", colorSensor.blue());
       opMode.telemetry.addData("Left servo", pusherLeft.getPosition());
       opMode.telemetry.addData("Right servo", pusherRight.getPosition());
       opMode.telemetry.update();
   }

    public void pushBothButtons(){
        pushLeftButton();
        pushRightButton();
    }

    public void retractBothPushers(){
        retractLeftPusher();
        retractRightPusher();
    }

    private int getBeaconColor(){
        if (colorSensor.red() > colorSensor.blue()){
            return RED;
        }
        else {
            return BLUE;
        }
    }

    public void pushLeftButton() {
        pusherLeft.setPosition(PUSH_LEFT_POSITION);
    }

    public void pushRightButton() {
        pusherRight.setPosition(PUSH_RIGHT_POSITION);
    }

    public void retractLeftPusher() {
        pusherLeft.setPosition(START_LEFT_POSITION);
    }

    public void retractRightPusher() {
        pusherRight.setPosition(START_RIGHT_POSITION);
    }
}