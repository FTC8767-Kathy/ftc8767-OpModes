package org.firstinspires.ftc.teamcode.opmodes8767;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Red 100 Points", group="DexM")
//@Disabled
public class Red_100Points extends LinearOpMode {

    public DexM_Hardware robot   = new DexM_Hardware();
    public ElapsedTime     runtime = new ElapsedTime();

    final static int RED = 1;
    final static int BLUE = 2;

    public int allianceColor = 1;   // for blue set to 2()

    @Override
    public void runOpMode() {
        robot.init(this);
        robot.driveTrain.calibrateGyro();
        robot.driveTrain.setAllEncoders(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        //test for gyro
        robot.driveTrain.gyroDrive(.75,70,0);
        robot.driveTrain.gyroTurn(0.5,180);
        robot.driveTrain.GyroStrafeRight(30);
        robot.driveTrain.GyroStrafeLeft(30);

        //EncoderDrive Test
        robot.driveTrain.encoderDrive(50,0.7);

        sleep(50000);

        //  Head toward the first beacon:
        robot.driveTrain.driveToDistanceRange(17);
        robot.driveTrain.gyroTurn(0.5,137);
        robot.driveTrain.gyroDrive(0.8,57,137);   // change to find line w/ ods?

        // Find the beacon
        robot.driveTrain.driveToLine(0.9);
        robot.driveTrain.gyroTurn(0.5,43);
        robot.driveTrain.strafeToLineLeft(.6);
        robot.driveTrain.driveToDistanceRange(15);

        // Trigger 1st beacon to correct color
        robot.beaconTrigger.pushCorrectButton(allianceColor);   // extend servo based on right side beacon color
        robot.driveTrain.driveToDistanceRange(7);

        // Prepare for 2nd beacon
        robot.beaconTrigger.retractBothPushers();
        robot.driveTrain.driveToDistanceRange(15);

        // Strafe Right
        robot.driveTrain.GyroStrafeRight(27);
        robot.driveTrain.driveToDistanceRange(15);
        robot.driveTrain.GyroStrafeRight(25);
        robot.driveTrain.strafeToLineRight(1.2);

        // Trigger 2nd beacon to correct color
        robot.beaconTrigger.pushCorrectButton(allianceColor);
        robot.driveTrain.driveToDistanceRange(7);
        robot.beaconTrigger.retractBothPushers();
        robot.driveTrain.gyroDrive(-0.7,5,90);

        //Go to Capball
        robot.driveTrain.gyroTurn(0.75,135);
        robot.driveTrain.gyroDrive(0.8,20,135);

        //Shot Ball and Collect
        //robot.launcher.autoLaunch();
        //sleep(1/4);  // **** ask Jonah
        //robot.collector.collectTime(2);
        //sleep(1);  // **** ask Jonah
        //robot.launcher.autoLaunch();

        robot.driveTrain.gyroDrive(1,30,135);

    }
}

