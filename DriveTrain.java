package org.firstinspires.ftc.teamcode.opmodes8767;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrain {

    public ModernRoboticsI2cGyro gyro;

    public DcMotor NWMotor = null;
    public DcMotor NEMotor = null;
    public DcMotor SWMotor = null;
    public DcMotor SEMotor = null;
    double NWPower;
    double NEPower;
    double SWPower;
    double SEPower;
    //double heading = gyro.getHeading();
    double heading = TEST_HEADING;
    double IntegrateDriveAngle = 0;
    static final double COUNTS_PER_MOTOR_REV = 1440;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    static final double TEST_HEADING = 45;

    LinearOpMode opMode;

    public DriveTrain(LinearOpMode opMode){

        this.opMode = opMode;

        NWMotor = opMode.hardwareMap.dcMotor.get("NWMotor");
        NEMotor = opMode.hardwareMap.dcMotor.get("NEMotor");
        SWMotor = opMode.hardwareMap.dcMotor.get("SWMotor");
        SEMotor = opMode.hardwareMap.dcMotor.get("SEMotor");
        gyro = (ModernRoboticsI2cGyro)opMode.hardwareMap.gyroSensor.get("gyro");

        NWMotor.setDirection(DcMotor.Direction.FORWARD);
        NEMotor.setDirection(DcMotor.Direction.REVERSE);
        SWMotor.setDirection(DcMotor.Direction.FORWARD);
        SEMotor.setDirection(DcMotor.Direction.REVERSE);

        SetMotorPower(0, 0, 0, 0);

        NWMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        NEMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SWMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SEMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void Drive(){

        if (opMode.gamepad1.a) {
            NWPower = (-opMode.gamepad1.left_stick_y + opMode.gamepad1.left_stick_x + opMode.gamepad1.right_stick_x) + GridOrientationMotor1and4();
            NEPower = (-opMode.gamepad1.left_stick_y - opMode.gamepad1.left_stick_x - opMode.gamepad1.right_stick_x) + GridOrientationMotor2and3();
            SWPower = (-opMode.gamepad1.left_stick_y - opMode.gamepad1.left_stick_x + opMode.gamepad1.right_stick_x) + GridOrientationMotor2and3();
            SEPower = (-opMode.gamepad1.left_stick_y + opMode.gamepad1.left_stick_x - opMode.gamepad1.right_stick_x) + GridOrientationMotor1and4();

        }

        else {
            NWPower = (-opMode.gamepad1.left_stick_y + opMode.gamepad1.left_stick_x + opMode.gamepad1.right_stick_x);
            NEPower = (-opMode.gamepad1.left_stick_y - opMode.gamepad1.left_stick_x - opMode.gamepad1.right_stick_x);
            SWPower = (-opMode.gamepad1.left_stick_y - opMode.gamepad1.left_stick_x + opMode.gamepad1.right_stick_x);
            SEPower = (-opMode.gamepad1.left_stick_y + opMode.gamepad1.left_stick_x - opMode.gamepad1.right_stick_x);

        }

        if (opMode.gamepad1.left_bumper) {

            NWPower = .5;

        }

        else if (opMode.gamepad1.right_bumper) {

            NEPower = .5;

        }

        else if (opMode.gamepad1.left_trigger > 0) {

            SWPower = .5;

        }

        else if (opMode.gamepad1.right_trigger > 0) {

            SEPower = .5;

        }

        SetMotorPower(NWPower, NEPower, SWPower, SEPower);

        //Nomalize();

    }

    public void encoderDrive (double Inches) {

        int NWTarget;
        int NETarget;
        int SWTarget;
        int SETarget;

        NWTarget = NWMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
        NETarget = NEMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
        SWTarget = SWMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
        SETarget = SEMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);

        NWMotor.setTargetPosition(NWTarget);
        NEMotor.setTargetPosition(NETarget);
        SWMotor.setTargetPosition(SWTarget);
        SEMotor.setTargetPosition(SETarget);

        NWMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        NEMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SWMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SEMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        SetMotorPower(Math.abs(DRIVE_SPEED), Math.abs(DRIVE_SPEED), Math.abs(DRIVE_SPEED), Math.abs(DRIVE_SPEED));

        while (NWMotor.isBusy() || NEMotor.isBusy() || SWMotor.isBusy() || SEMotor.isBusy()){

            NWMotor.getCurrentPosition();
            NEMotor.getCurrentPosition();
            SWMotor.getCurrentPosition();
            SEMotor.getCurrentPosition();

        }

        SetMotorPower(0, 0, 0, 0);

        NWMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        NEMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SWMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SEMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void encoderTurn (double Degrees) {

        int NWTarget;
        int NETarget;
        int SWTarget;
        int SETarget;

        NWTarget = NWMotor.getCurrentPosition() + (int) (-Degrees * .1 * COUNTS_PER_INCH);
        NETarget = NEMotor.getCurrentPosition() + (int) (Degrees * .1 * COUNTS_PER_INCH);
        SWTarget = SWMotor.getCurrentPosition() + (int) (-Degrees * .1 * COUNTS_PER_INCH);
        SETarget = SEMotor.getCurrentPosition() + (int) (Degrees * .1 * COUNTS_PER_INCH);

        NWMotor.setTargetPosition(NWTarget);
        NEMotor.setTargetPosition(NETarget);
        SWMotor.setTargetPosition(SWTarget);
        SEMotor.setTargetPosition(SETarget);

        NWMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        NEMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SWMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SEMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        SetMotorPower(Math.abs(TURN_SPEED), Math.abs(TURN_SPEED), Math.abs(TURN_SPEED), Math.abs(TURN_SPEED));

        while (NWMotor.isBusy() || NEMotor.isBusy() || SWMotor.isBusy() || SEMotor.isBusy()){

            NWMotor.getCurrentPosition();
            NEMotor.getCurrentPosition();
            SWMotor.getCurrentPosition();
            SEMotor.getCurrentPosition();

        }

        SetMotorPower(0, 0, 0, 0);

        NWMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        NEMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SWMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SEMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void encoderStrafe (double Inches) {

        // CURRENTLY INACTIVE

        int NWTarget;
        int NETarget;
        int SWTarget;
        int SETarget;

        NWTarget = NWMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
        NETarget = NEMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
        SWTarget = SWMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
        SETarget = SEMotor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);

        NWMotor.setTargetPosition(NWTarget);
        NEMotor.setTargetPosition(NETarget);
        SWMotor.setTargetPosition(SWTarget);
        SEMotor.setTargetPosition(SETarget);

        NWMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        NEMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SWMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SEMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        SetMotorPower(Math.abs(DRIVE_SPEED), Math.abs(DRIVE_SPEED), Math.abs(DRIVE_SPEED), Math.abs(DRIVE_SPEED));

        while (NWMotor.isBusy() || NEMotor.isBusy() || SWMotor.isBusy() || SEMotor.isBusy()){

            NWMotor.getCurrentPosition();
            NEMotor.getCurrentPosition();
            SWMotor.getCurrentPosition();
            SEMotor.getCurrentPosition();

        }

        SetMotorPower(0, 0, 0, 0);

        NWMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        NEMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SWMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SEMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void Nomalize () {

        if (Math.abs(NWPower) > 1.0) {

            NWPower /= NWPower;
        }

        if (Math.abs(NEPower) > 1.0) {

            NEPower /= NEPower;

        }

        if (Math.abs(SWPower) > 1.0) {

            SWPower /= SWPower;

        }

        if (Math.abs(SEPower) > 1.0) {

            SEPower /= SEPower;

        }

    }

    public void SetMotorPower (double NWMotorPower, double NEMotorPower, double SWMotorPower, double SEMotorPower) {

        NWMotor.setPower(NWMotorPower);
        NEMotor.setPower(NEMotorPower);
        SWMotor.setPower(SWMotorPower);
        SEMotor.setPower(SEMotorPower);

    }

    public double GridOrientationMotor1and4 () {

        return IntegrateDriveAngle = ((Math.pow(heading, 2))/(Math.pow(1.623, 2)))+((Math.pow(heading-0.927, 2))/(Math.pow(1.5, 2)));

    }

    public double GridOrientationMotor2and3 () {

        return IntegrateDriveAngle = ((Math.pow(heading, 2))/(Math.pow(1.623, 2)))+((Math.pow(heading+0.927, 2))/(Math.pow(1.5, 2)));

    }

}

