package org.firstinspires.ftc.teamcode.opmodes8767;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="DexM_TeleOp", group="DexM")
//@Disabled
public class DexM_TeleOp extends LinearOpMode {
    public DexM_TeleOp(){

    }

    DexM_Hardware robot = new DexM_Hardware();

    @Override
    public void runOpMode() throws InterruptedException {
        double NWPower;
        double NEPower;
        double SWPower;
        double SEPower;
        double maxFront;
        double maxRear;
//
        ModernRoboticsI2cGyro gyro;
        int heading = 0;
        int angleZ = 0;
        boolean lastResetState = false;
        boolean curResetState  = false;

        boolean launching = false;

        robot.init(hardwareMap);

        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        gyro.calibrate();

        while (!isStopRequested() && gyro.isCalibrating())  {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.update();

        telemetry.addData("Say", "Hello Driver");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            curResetState = (gamepad1.a && gamepad1.b);
            if(curResetState && !lastResetState)  {
                gyro.resetZAxisIntegrator();
            }
            lastResetState = curResetState;

            heading = gyro.getHeading();
            angleZ  = gyro.getIntegratedZValue();

            NWPower = (-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * 0.2;
            NEPower = (-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) * 0.24;
            SWPower = (-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * 0.24;
            SEPower = (-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) * 0.24;

            if(gamepad1.left_bumper){
                NWPower = .5;
            }
            if(gamepad1.right_bumper){
                NEPower = .5;
            }
            if(gamepad1.left_trigger > 0){
                SWPower = .5;
            }
            if(gamepad1.right_trigger > 0){
                SEPower = .5;
            }

/*          if(gamepad2.x){
                //robot.LeftSweeper.setPosition(.75);
            }
            else{
                //robot.LeftSweeper.setPosition(0);
            }
            if(gamepad2.b){
                //robot.RightSweeper.setPosition(.75);
            }
            else{
                //robot.RightSweeper.setPosition(0);
            }*/

            maxFront = Math.max(Math.abs(NWPower), Math.abs(NEPower));
            if (maxFront > 1.0) {
                NWPower /= maxFront;
                NEPower /= maxFront;
            }

            maxRear = Math.max(Math.abs(SWPower), Math.abs(SEPower));
            if (maxRear > 1.0) {
                SWPower /= maxRear;
                SEPower /= maxRear;
            }
            // Check for collector commands
            if (gamepad2.a) // pull in to launcher
                robot.collector.setPower(1);
            else if (gamepad2.x) // push out of launcher
                robot.collector.setPower(-1);
            else if (gamepad2.b) // stop collector
                robot.collector.setPower(0);

            // Check for automatic launch command
            if (gamepad2.right_trigger > 0 && !launching) {
                robot.launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.launcher.setTargetPosition(1120 * 3); // 3:1 ratio for AndyMark motor
                robot.launcher.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.launcher.setPower(-1);
                launching = true;
            }
            else if (launching && !robot.launcher.isBusy()) { // Auto-launch is complete
                robot.launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                launching = false;
            }


            // Check for manual launcher commands
            if (!launching) { // First make sure we are not doing an auto-launch
                if (gamepad2.left_bumper)
                    robot.launcher.setPower(-1);
                else if (gamepad2.left_trigger > 0)
                    robot.launcher.setPower(1);
                else
                    robot.launcher.setPower(0);
            }

            telemetry.addData("launching", launching);
            telemetry.addData("launch power", robot.launcher.getPower());
            telemetry.addData("launch count", robot.launcher.getCurrentPosition());

            robot.Motor1.setPower(NWPower);
            robot.Motor2.setPower(NEPower);
            robot.Motor3.setPower(SWPower);
            robot.Motor4.setPower(SEPower);

            telemetry.addData(">", "Press A & B to reset Heading.");
            telemetry.addData("0", "Heading %03d", heading);
            telemetry.addData("1", "Int. Ang. %03d", angleZ);
            telemetry.addData("NWSpeed: ", NWPower);
            telemetry.addData("NESpeed: ", NEPower);
            telemetry.addData("SWSpeed: ", SWPower);
            telemetry.addData("SESpeed: ", SEPower);
            telemetry.update();

            robot.waitForTick(40);
        }
    }
}
