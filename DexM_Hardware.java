package org.firstinspires.ftc.teamcode.opmodes8767;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class DexM_Hardware {
    public DcMotor  Motor1   = null;
    public DcMotor  Motor2  = null;
    public DcMotor  Motor3   = null;
    public DcMotor  Motor4  = null;
    public DcMotor collector = null;
    public DcMotor launcher = null;
    //public Servo RightBeacon = null;
    //public Servo LeftBeacon = null;
    //public Servo RightSweeper = null;
    //public Servo LeftSweeper = null;

    public ModernRoboticsI2cGyro gyro;

    public static final double MID_SERVO = 0.5;

    HardwareMap hwMap;
    LinearOpMode opMode;
    private ElapsedTime period  = new ElapsedTime();

    public DexM_Hardware(){
    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        Motor1  = hwMap.dcMotor.get("NWMotor");
        Motor2  = hwMap.dcMotor.get("NEMotor");
        Motor3  = hwMap.dcMotor.get("SWMotor");
        Motor4  = hwMap.dcMotor.get("SEMotor");

        collector = hwMap.dcMotor.get("motor_collector");
        launcher = hwMap.dcMotor.get("motor_launcher");

        Motor1.setDirection(DcMotor.Direction.FORWARD);
        Motor2.setDirection(DcMotor.Direction.REVERSE);
        Motor3.setDirection(DcMotor.Direction.FORWARD);
        Motor4.setDirection(DcMotor.Direction.REVERSE);

        Motor1.setPower(0);
        Motor2.setPower(0);
        Motor3.setPower(0);
        Motor4.setPower(0);

        Motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        collector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        collector.setPower(0);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setPower(0);

        //RightBeacon.setPosition(0);
        //LeftBeacon.setPosition(0);
        //RightSweeper.setPosition(0);
        //LeftSweeper.setPosition(0);
    }
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        period.reset();
    }



}
