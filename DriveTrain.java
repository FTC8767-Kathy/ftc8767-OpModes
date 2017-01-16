package org.firstinspires.ftc.teamcode.opmodes8767;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class DriveTrain {

    public ModernRoboticsI2cGyro gyro;

    public DcMotor NWMotor = null; // these should be made private when EncoderDrive_Gyro_Method is fixed
    public DcMotor NEMotor = null; // then maybe create getter & setter methods?
    public DcMotor SWMotor = null; //
    public DcMotor SEMotor = null; //

    private double NWPower;
    private double NEPower;
    private double SWPower;
    private double SEPower;

    private int nwTarget;
    private int neTarget;
    private int swTarget;
    private int seTarget;

    //double heading = gyro.getHeading();
    static final double TEST_HEADING = 45;  // temporary for testing?
    double heading = TEST_HEADING;          // when ready, delete these 2 & uncomment gyro.getHeading()?

    LinearOpMode opMode;

    double IntegrateDriveAngle = 0;

    static final double COUNTS_PER_MOTOR_REV = 1440;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    public DriveTrain(LinearOpMode opMode){   // constructor
        this.opMode = opMode;

        initialDriveMotorsSetup(opMode);

        gyro = (ModernRoboticsI2cGyro)opMode.hardwareMap.gyroSensor.get("gyro");
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

        // check if driver wants to run an individual motor with triggers & bumpers
        testIndividualMotors();

        // set motor power based on joysticks **** individual states WILL override joysticks
        setMotorPower(NWPower, NEPower, SWPower, SEPower);

        //normalize();
    }

    public void encoderDrive (double inches) {
        // calculate & adjust drive motor targets for number of inches desired
        int adjustTicks = (int) (inches * COUNTS_PER_INCH);
        adjustAllCurrentTargets(adjustTicks, adjustTicks, adjustTicks, adjustTicks);

        setAllEncoders(DcMotor.RunMode.RUN_USING_ENCODER);
        setAllEncoders(DcMotor.RunMode.RUN_TO_POSITION);
        setMotorPower(Math.abs(DRIVE_SPEED), Math.abs(DRIVE_SPEED), Math.abs(DRIVE_SPEED), Math.abs(DRIVE_SPEED));

        waitForDriveTargetsToReach();  // update telemetry while motors work toward targets
        setMotorPower(0, 0, 0, 0);
    }

    public void encoderTurn (double degrees) {
        // calculate & adjust drive motor targets for number of degrees desired
        int adjustTicks = (int) (degrees * .1 * COUNTS_PER_INCH);
        adjustAllCurrentTargets(-adjustTicks, adjustTicks, -adjustTicks, adjustTicks);

        setAllEncoders(DcMotor.RunMode.RUN_USING_ENCODER);
        setAllEncoders(DcMotor.RunMode.RUN_TO_POSITION);
        setMotorPower(Math.abs(TURN_SPEED), Math.abs(TURN_SPEED), Math.abs(TURN_SPEED), Math.abs(TURN_SPEED));

        waitForDriveTargetsToReach();  // update telemetry while motors work toward targets
        setMotorPower(0, 0, 0, 0);
    }

    public void encoderStrafe (double inches) {

        // CURRENTLY INACTIVE

        // calculate & adjust drive motor targets for number of inches desired
        int adjustTicks = (int) (inches * COUNTS_PER_INCH);
        adjustAllCurrentTargets(adjustTicks, adjustTicks, adjustTicks, adjustTicks);

        setAllEncoders(DcMotor.RunMode.RUN_USING_ENCODER);
        setAllEncoders(DcMotor.RunMode.RUN_TO_POSITION);
        setMotorPower(Math.abs(DRIVE_SPEED), Math.abs(DRIVE_SPEED), Math.abs(DRIVE_SPEED), Math.abs(DRIVE_SPEED));

        waitForDriveTargetsToReach();  // update telemetry while motors work toward targets
        setMotorPower(0, 0, 0, 0);
    }

    public void normalize() {
        double maxPower = Math.max(Math.abs(NWPower), Math.abs(NEPower));
        maxPower = Math.max(maxPower, Math.abs(SWPower));
        maxPower = Math.max(maxPower, Math.abs(SEPower));

        if (maxPower > 1.0){
            NWPower /= maxPower;
            NEPower /= maxPower;
            SWPower /= maxPower;
            SEPower /= maxPower;
        }
    }

    public void calibrateGyro(){
        //start calibrating the gyro
        opMode.telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        opMode.telemetry.update();

        opMode.sleep(1000); // wait 1 second for gyro to stabilize (may be movement from initializing servos)
        gyro.calibrate();

        // make sure the gyro is finished calibrating
        while (!opMode.isStopRequested() && gyro.isCalibrating())  {
            opMode.sleep(50);
        }
        opMode.telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        opMode.telemetry.update();
    }

    public void setMotorPower(double NWMotorPower, double NEMotorPower, double SWMotorPower, double SEMotorPower) {

        NWMotor.setPower(NWMotorPower);
        NEMotor.setPower(NEMotorPower);
        SWMotor.setPower(SWMotorPower);
        SEMotor.setPower(SEMotorPower);
    }

    public void setAllEncoders(DcMotor.RunMode encoderMode) {
        NWMotor.setMode(encoderMode);
        NEMotor.setMode(encoderMode);
        SWMotor.setMode(encoderMode);
        SEMotor.setMode(encoderMode);
    }

    private void adjustAllCurrentTargets(int nwAdjust, int neAdjust, int swAdjust, int seAdjust) {
        // calculate new target
        nwTarget = NWMotor.getCurrentPosition() + nwAdjust;
        neTarget = NEMotor.getCurrentPosition() + neAdjust;
        swTarget = SWMotor.getCurrentPosition() + swAdjust;
        seTarget = SEMotor.getCurrentPosition() + seAdjust;

        setAllDriveTargets(nwTarget, neTarget, swTarget, seTarget);  // set motors to new targets
    }

    private void setAllDriveTargets(int NWTarget, int NETarget, int SWTarget, int SETarget) {
        NWMotor.setTargetPosition(NWTarget);
        NEMotor.setTargetPosition(NETarget);
        SWMotor.setTargetPosition(SWTarget);
        SEMotor.setTargetPosition(SETarget);
    }

    private void waitForDriveTargetsToReach() {
        while (opMode.opModeIsActive() && (NWMotor.isBusy() || NEMotor.isBusy() || SWMotor.isBusy() || SEMotor.isBusy())){
            updateTelemetryForTargetAndCurrent();
        }
    }

    private void updateTelemetryForTargetAndCurrent() {
        opMode.telemetry.addData("NW Target & Current", nwTarget +" ; " + NWMotor.getCurrentPosition());
        opMode.telemetry.addData("NE Target & Current", neTarget +" ; " + NEMotor.getCurrentPosition());
        opMode.telemetry.addData("SW Target & Current", swTarget +" ; " + SWMotor.getCurrentPosition());
        opMode.telemetry.addData("SE Target & Current", seTarget +" ; " + SEMotor.getCurrentPosition());

        opMode.telemetry.update();
    }

    private void testIndividualMotors() {
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
    }

    private void initialDriveMotorsSetup(LinearOpMode opMode) {
        NWMotor = opMode.hardwareMap.dcMotor.get("NWMotor");
        NEMotor = opMode.hardwareMap.dcMotor.get("NEMotor");
        SWMotor = opMode.hardwareMap.dcMotor.get("SWMotor");
        SEMotor = opMode.hardwareMap.dcMotor.get("SEMotor");

        NWMotor.setDirection(DcMotor.Direction.FORWARD);
        NEMotor.setDirection(DcMotor.Direction.REVERSE);
        SWMotor.setDirection(DcMotor.Direction.FORWARD);
        SEMotor.setDirection(DcMotor.Direction.REVERSE);

        setMotorPower(0, 0, 0, 0);
        setAllEncoders(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double GridOrientationMotor1and4 () {

        return IntegrateDriveAngle = ((Math.pow(heading, 2))/(Math.pow(1.623, 2)))+((Math.pow(heading-0.927, 2))/(Math.pow(1.5, 2)));

    }

    public double GridOrientationMotor2and3 () {

        return IntegrateDriveAngle = ((Math.pow(heading, 2))/(Math.pow(1.623, 2)))+((Math.pow(heading+0.927, 2))/(Math.pow(1.5, 2)));

    }
}

