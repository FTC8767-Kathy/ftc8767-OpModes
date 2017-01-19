package org.firstinspires.ftc.teamcode.opmodes8767;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="DexM_TeleOp", group="DexM")
//@Disabled
public class DexM_TeleOp extends LinearOpMode {

    public DexM_TeleOp(){

    }

    DexM_Hardware robot = new DexM_Hardware();

    @Override
    public void runOpMode() {

        boolean lastResetState = false;
        boolean curResetState  = false;

        robot.init(this);
        robot.driveTrain.calibrateGyro();

        waitForStart();

        while (opModeIsActive()) {

            curResetState = (gamepad1.a && gamepad1.b);

            if(curResetState && !lastResetState)  {
                robot.driveTrain.gyro.resetZAxisIntegrator();
            }
            lastResetState = curResetState;

            robot.driveTrain.driveWithControllers();

            if (gamepad2.a) {
                robot.collector.collect();
            }
            else if (gamepad2.x) {
                robot.collector.reverse();
            }
            else if (gamepad2.b) {
                robot.collector.stop();
            }
            if (gamepad2.left_bumper) {
                robot.launcher.launch();
            }
            else {
                robot.launcher.stop();
            }

            robot.collector.speedTest();

            robot.waitForTick(40);
        }
    }
}
