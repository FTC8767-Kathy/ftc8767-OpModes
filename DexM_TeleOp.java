package org.firstinspires.ftc.teamcode.opmodes8767;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="DexM_TeleOp", group="DexM")
//@Disabled
public class DexM_TeleOp extends LinearOpMode {

    DexM_Hardware robot = new DexM_Hardware();

    public boolean lastResetState = false;
    public boolean curResetState  = false;

    public DexM_TeleOp(){
    }

    @Override
    public void runOpMode() {

        robot.init(this);
//        robot.driveTrain.calibrateGyro();

        waitForStart();

        while (opModeIsActive()) {

            robot.driveTrain.driveWithControllers();

            checkForGyroCalibrateCommand();
            checkForCollectorCommands();
            checkForLauncherCommand();
            checkForSweeperCommands();
            checkForBeaconPusherCommands();

//            robot.collector.speedTest();

            updateTelemetry();

            robot.waitForTick(40);
        }
    }

    private void checkForGyroCalibrateCommand() {
        curResetState = (gamepad1.a && gamepad1.b);

        if(curResetState && !lastResetState)  {
            robot.driveTrain.gyro.resetZAxisIntegrator();
        }
        lastResetState = curResetState;
    }

    private void updateTelemetry() {
        robot.driveTrain.drivetrainTelemetry();
        robot.sweeper.sweeperTelemetry();
        robot.launcher.launcherTelemetry();
        telemetry.addData("Raw", robot.driveTrain.lightSensorLeft.getRawLightDetected());
        telemetry.update();
    }

    private void checkForBeaconPusherCommands() {
        if (robot.opMode.gamepad2.right_trigger > 0) {
            robot.beaconTrigger.pushRightButton();
        }
        else if (robot.opMode.gamepad2.right_bumper) {
            robot.beaconTrigger.retractRightPusher();
        }

        if (robot.opMode.gamepad2.left_trigger > 0) {
            robot.beaconTrigger.pushLeftButton();
        }
        else if (robot.opMode.gamepad2.left_bumper) {
            robot.beaconTrigger.retractLeftPusher();
        }
    }

    private void checkForSweeperCommands() {
        if (robot.opMode.gamepad2.dpad_up) {
            robot.sweeper.setSweeper0Percent();
        }
        else if (robot.opMode.gamepad2.dpad_right || robot.opMode.gamepad2.dpad_left) {
            robot.sweeper.setSweeper50Percent();
        }
        else if (robot.opMode.gamepad2.dpad_down) {
            robot.sweeper.setSweeper70Percent();
        }
    }

    private void checkForLauncherCommand() {
        if (gamepad2.left_stick_y < -.5) {
            robot.launcher.launch();
        }
        else {
            robot.launcher.stop();
        }
    }

    private void checkForCollectorCommands() {
/*
        if (gamepad2.a) {
            robot.collector.collect();
        }
        else if (gamepad2.x) {
            robot.collector.reverse();
        }
        else if (gamepad2.b) {
            robot.collector.stop();
        }
*/
        robot.collector.collect(gamepad2.right_stick_y);
    }
}
