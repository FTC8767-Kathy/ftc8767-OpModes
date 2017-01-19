package org.firstinspires.ftc.teamcode.opmodes8767;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="DexM_Auto_Red4", group="DexM")
//@Disabled
public class DexM_Auto_Red4 extends LinearOpMode {

    public DexM_Hardware robot   = new DexM_Hardware();
    public ElapsedTime     runtime = new ElapsedTime();

    final static int RED = 1;
    final static int BLUE = 2;

    public int allianceColor = 1;   // for blue set to 2

    @Override
    public void runOpMode() {
        robot.init(this);

        robot.driveTrain.setAllEncoders(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        robot.launcher.autoLaunch();
        robot.collector.collectTime(3);
        robot.launcher.autoLaunch();

        /*  back away from wall,
            turn toward 1st beacon,
            drive to white line,
            turn square to beacon
         */
        robot.driveTrain.encoderDrive(-7);
        robot.driveTrain.turnRight(140.5);
        robot.driveTrain.encoderDrive(60);   // change to find line w/ ods?
        robot.driveTrain.turnLeft(49.5);

        // Trigger 1st beacon to correct color
        robot.beaconTrigger.pushCorrectButton(allianceColor);   // extend servo based on right side beacon color
        robot.driveTrain.driveToDistanceRange(7);

        // Prepare for 2nd beacon
        robot.beaconTrigger.retractBothPushers();
        robot.driveTrain.strafeRight(48);

        // Trigger 2nd beacon to correct color
        robot.beaconTrigger.pushCorrectButton(allianceColor);
        robot.driveTrain.driveToDistanceRange(7);

    }
}

