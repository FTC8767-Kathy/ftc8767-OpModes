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
    public void runOpMode() throws InterruptedException {

        int heading = 0;
        int angleZ = 0;
        boolean lastResetState = false;
        boolean curResetState  = false;

        robot.init(this);

        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        //robot.DriveTrain.gyro.calibrate();

        while (!isStopRequested() && robot.DriveTrain.gyro.isCalibrating())  {

            sleep(50);
            idle();

        }

        telemetry.addData("Say", "Hello Driver");
        telemetry.update();

        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            curResetState = (gamepad1.a && gamepad1.b);

            if(curResetState && !lastResetState)  {

                robot.DriveTrain.gyro.resetZAxisIntegrator();

            }

            lastResetState = curResetState;

            heading = robot.DriveTrain.gyro.getHeading();
            angleZ  = robot.DriveTrain.gyro.getIntegratedZValue();

            robot.DriveTrain.Drive();

            if (gamepad2.a) {

                robot.collector.Collect();

            }

            else if (gamepad2.x) {

                robot.collector.Abort();

            }

            else if (gamepad2.b) {

                robot.collector.Stop();

            }

            if (gamepad2.left_bumper) {

                robot.launcher.launch();

            }

            else {

                robot.launcher.Idle();

            }

            robot.collector.SpeedTest();

            telemetry.addData(">", "Press A & B to reset Heading.");
            telemetry.addData("0", "Heading %03f", robot.DriveTrain.heading);
            telemetry.addData("1", "Int. Ang. %03d", angleZ);
            telemetry.addData("NWSpeed: ", robot.DriveTrain.NWPower);
            telemetry.addData("NESpeed: ", robot.DriveTrain.NEPower);
            telemetry.addData("SWSpeed: ", robot.DriveTrain.SWPower);
            telemetry.addData("SESpeed: ", robot.DriveTrain.SEPower);
            telemetry.update();

            robot.waitForTick(40);

        }

    }

}
