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

        robot.capLiftLock.Lower();

        waitForStart();

        while (opModeIsActive()) {

            robot.driveTrain.driveWithControllers();

            checkForGyroCalibrateCommand();
            checkForCollectorCommands();
            checkForLauncherCommand();
            checkForSweeperCommands();
            checkForBeaconPusherCommands();
            checkForCapLiftCommands();
            checkForCapLiftLockCommands();

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
        if (robot.opMode.gamepad1.y) {
            robot.sweeper.setSweeper0Percent();
        }
        else if (robot.opMode.gamepad1.b) {
            robot.sweeper.setSweeper50Percent();
        }
        else if (robot.opMode.gamepad1.x) {
            robot.sweeper.setSweeper70Percent();
        }
    }

    private void checkForLauncherCommand() {
        if (gamepad2.a) {
            robot.launcher.launch();
        }
        else {
            robot.launcher.stop();
        }
    }

    private void checkForCollectorCommands() {
        robot.collector.collect(gamepad2.right_stick_y);
    }

    private void checkForCapLiftCommands () {

        robot.capLift.Lift();

    }

    private void checkForCapLiftLockCommands () {

        if (robot.opMode.gamepad2.y) {

            robot.capLiftLock.Lift();

        }

    }
}
