package org.firstinspires.ftc.teamcode.opmodes8767;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
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

        int heading = 0;
        int angleZ = 0;
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

            heading = robot.driveTrain.gyro.getHeading();
            angleZ  = robot.driveTrain.gyro.getIntegratedZValue();

            robot.driveTrain.Drive();

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

            telemetry.addData(">", "Press A & B to reset Heading.");
            telemetry.addData("0", "Heading %03f", robot.driveTrain.heading);
            telemetry.addData("1", "Int. Ang. %03d", angleZ);
            telemetry.addData("NWSpeed: ", robot.driveTrain.NWPower);
            telemetry.addData("NESpeed: ", robot.driveTrain.NEPower);
            telemetry.addData("SWSpeed: ", robot.driveTrain.SWPower);
            telemetry.addData("SESpeed: ", robot.driveTrain.SEPower);
            telemetry.update();

            robot.waitForTick(40);
        }
    }
}
