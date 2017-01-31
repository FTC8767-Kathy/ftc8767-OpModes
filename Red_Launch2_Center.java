package org.firstinspires.ftc.teamcode.opmodes8767;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Red Launch2 Center", group="DexM")
//@Disabled
public class Red_Launch2_Center extends LinearOpMode {

    public DexM_Hardware robot   = new DexM_Hardware();
    public ElapsedTime     runtime = new ElapsedTime();


    @Override
    public void runOpMode() {
        robot.init(this);

        robot.driveTrain.setAllEncoders(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        sleep(10000);

        robot.driveTrain.driveToDistanceRange(6);


        robot.driveTrain.turnLeft(25);
        robot.launcher.autoLaunch();
        sleep(1/4);  // **** ask Jonah
        robot.collector.collectTime(2);
        sleep(1);  // **** ask Jonah
        robot.launcher.autoLaunch();

        robot.driveTrain.turnRight(23);
        robot.driveTrain.encoderDrive(-55, DriveTrain.DRIVE_SPEED);   // change to find line w/ ods?



    }
}

