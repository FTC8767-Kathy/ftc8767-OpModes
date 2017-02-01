package org.firstinspires.ftc.teamcode.opmodes8767;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DriveTrain {

    public ModernRoboticsI2cGyro gyro;
    ModernRoboticsI2cRangeSensor rangeSensor;
    OpticalDistanceSensor lightSensorLeft;
//    OpticalDistanceSensor lightSensorRight;

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

    static final double COUNTS_PER_MOTOR_REV = 1440;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double DRIVE_SPEED = 0.75;
    static final double STRAFE_SPEED = 0.5;
    static final double STRAFE_COMPENSATE = 1.1;
    static final double TURN_SPEED = 0.45;
    static final double ULTRA_SPEED = 0.16;
    static final double FIND_LINE_SPEED = .16;
    static final double TOP_SPEED_SCALE = .5;
    final static double MINIMUN_POWER_TO_MOVE = .15;

    public DriveTrain(LinearOpMode opMode){   // constructor
        this.opMode = opMode;

        initialDriveMotorsSetup(opMode);

        gyro = (ModernRoboticsI2cGyro)opMode.hardwareMap.gyroSensor.get("gyro");
        rangeSensor = opMode.hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensor");
        lightSensorLeft = opMode.hardwareMap.get(OpticalDistanceSensor.class, "lightSensor");
    }

    public void driveWithControllers(){
        heading = gyro.getHeading();

        NWPower = (-opMode.gamepad1.left_stick_y + opMode.gamepad1.left_stick_x + opMode.gamepad1.right_stick_x);
        NEPower = (-opMode.gamepad1.left_stick_y - opMode.gamepad1.left_stick_x - opMode.gamepad1.right_stick_x);
        SWPower = (-opMode.gamepad1.left_stick_y - opMode.gamepad1.left_stick_x + opMode.gamepad1.right_stick_x);
        SEPower = (-opMode.gamepad1.left_stick_y + opMode.gamepad1.left_stick_x - opMode.gamepad1.right_stick_x);

        checkDPadForTeleop();

        // check if driver wants to run an individual motor with triggers & bumpers
        testIndividualMotors();

/*
        // scale down speed for better control
        NWPower *= TOP_SPEED_SCALE;
        NEPower *= TOP_SPEED_SCALE;
        SWPower *= TOP_SPEED_SCALE;
        SEPower *= TOP_SPEED_SCALE;
*/
        // scale drive power for each motor for better control at slower speeds
        customizeDrivePowers();

        normalize();

        // set motor power based on joysticks **** individual states WILL override joysticks
        setMotorPower(NWPower, NEPower, SWPower, SEPower);
    }

    public void encoderDrive (double inches, double inPower) {
        // calculate & adjust drive motor targets for number of inches desired
        int adjustTicks = (int) (inches * COUNTS_PER_INCH);
        adjustAllCurrentTargets(adjustTicks, adjustTicks, adjustTicks, adjustTicks);

        setAllEncoders(DcMotor.RunMode.RUN_USING_ENCODER);
        setAllEncoders(DcMotor.RunMode.RUN_TO_POSITION);
        setAllMotorPowersTheSame(Math.abs(inPower));

        waitForDriveTargetsToReach();  // update telemetry while motors work toward targets
        setAllMotorPowersTheSame(0);
    }

    public void driveToDistanceRange (double targetDistance){
        setAllEncoders(DcMotor.RunMode.RUN_USING_ENCODER);
        while (opMode.opModeIsActive()){
            double currentDistance = rangeSensor.getDistance(DistanceUnit.CM);

            if (currentDistance > targetDistance + 0.25){
                updateTelemetryForDistanceRange(targetDistance,currentDistance);
                setAllMotorPowersTheSame(ULTRA_SPEED);
            }
            else if (currentDistance < targetDistance - 0.25){
                updateTelemetryForDistanceRange(targetDistance,currentDistance);
                setAllMotorPowersTheSame(-ULTRA_SPEED);
            }
            else{
                setAllMotorPowersTheSame(0);
                break;
            }
        }
    }

    public void driveToLine(double targetLightValue){
        setAllEncoders(DcMotor.RunMode.RUN_USING_ENCODER);

        double currentLightValue = lightSensorLeft.getRawLightDetected();

        while (opMode.opModeIsActive() && currentLightValue < targetLightValue){
            opMode.telemetry.addData("Raw", lightSensorLeft.getRawLightDetected());
            opMode.telemetry.update();
            setAllMotorPowersTheSame(FIND_LINE_SPEED);
            currentLightValue = lightSensorLeft.getRawLightDetected();
        }

        setAllMotorPowersTheSame(0);
    }

    private void encoderTurn (double degrees) {
        // calculate & adjust drive motor targets for number of degrees desired
        int adjustTicks = (int) (degrees * .26 * COUNTS_PER_INCH);
        adjustAllCurrentTargets(-adjustTicks, adjustTicks, -adjustTicks, adjustTicks);

        setAllEncoders(DcMotor.RunMode.RUN_USING_ENCODER);
        setAllEncoders(DcMotor.RunMode.RUN_TO_POSITION);
        setAllMotorPowersTheSame(Math.abs(TURN_SPEED));

        waitForDriveTargetsToReach();  // update telemetry while motors work toward targets
        setAllMotorPowersTheSame(0);
    }

    public void turnLeft (double degrees){
        encoderTurn(degrees);
    }

    public void turnRight (double degrees){
        encoderTurn(-degrees);
    }

    private void encoderStrafe (double inches) {
        // calculate & adjust driveWithControllers motor targets for number of inches desired
        int adjustTicks = (int) (inches * COUNTS_PER_INCH);
        adjustAllCurrentTargets(adjustTicks, -adjustTicks, -adjustTicks, adjustTicks);

        setAllEncoders(DcMotor.RunMode.RUN_USING_ENCODER);
        setAllEncoders(DcMotor.RunMode.RUN_TO_POSITION);
        setMotorPower(Math.abs(STRAFE_SPEED), Math.abs(STRAFE_SPEED), Math.abs(STRAFE_COMPENSATE * STRAFE_SPEED), Math.abs(STRAFE_COMPENSATE * STRAFE_SPEED));

        waitForDriveTargetsToReach();  // update telemetry while motors work toward targets
        setAllMotorPowersTheSame(0);
    }

    public void strafeLeft (double inches){
        encoderStrafe(-inches);
    }

    public void strafeRight (double inches){
        encoderStrafe(inches);
    }

    public void strafeToLineRight (double targetLightValue){
        setAllEncoders(DcMotor.RunMode.RUN_USING_ENCODER);

        double currentLightValue = lightSensorLeft.getRawLightDetected();

        while (opMode.opModeIsActive() && currentLightValue < targetLightValue){
            setMotorPower(FIND_LINE_SPEED, -FIND_LINE_SPEED, -FIND_LINE_SPEED, FIND_LINE_SPEED);
            currentLightValue = lightSensorLeft.getRawLightDetected();
        }

        setAllMotorPowersTheSame(0);
    }

    public void strafeToLineLeft (double targetLightValue){
        setAllEncoders(DcMotor.RunMode.RUN_USING_ENCODER);

        double currentLightValue = lightSensorLeft.getRawLightDetected();

        while (opMode.opModeIsActive() && currentLightValue < targetLightValue){
            setMotorPower(-FIND_LINE_SPEED, FIND_LINE_SPEED, FIND_LINE_SPEED, -FIND_LINE_SPEED);
            currentLightValue = lightSensorLeft.getRawLightDetected();
        }

        setAllMotorPowersTheSame(0);
    }

    private void setAllMotorPowersTheSame(double inPower) {
        setMotorPower(inPower, inPower, inPower, inPower);
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

    public void waitForDriveTargetsToReach() {
        while (opMode.opModeIsActive() && (NWMotor.isBusy() && NEMotor.isBusy() && SWMotor.isBusy()&& SEMotor.isBusy())){
            updateTelemetryForTargetAndCurrent();
        }
    }

    public void drivetrainTelemetry() {
        opMode.telemetry.addData(">", "Press A & B to reset Heading.");
        opMode.telemetry.addData("0", "Heading %03f", heading);
        opMode.telemetry.addData("NWSpeed: ", NWPower);
        opMode.telemetry.addData("NESpeed: ", NEPower);
        opMode.telemetry.addData("SWSpeed: ", SWPower);
        opMode.telemetry.addData("SESpeed: ", SEPower);
    }

    private void updateTelemetryForTargetAndCurrent() {
        opMode.telemetry.addData("NW Target, Current & power", nwTarget + " ; " + NWMotor.getCurrentPosition() + " ; "+ NWMotor.getPower() );
        opMode.telemetry.addData("NE Target, Current & power", neTarget + " ; " + NEMotor.getCurrentPosition() + " ; "+ NEMotor.getPower() );
        opMode.telemetry.addData("SW Target, Current & power", swTarget + " ; " + SWMotor.getCurrentPosition() + " ; "+ SWMotor.getPower() );
        opMode.telemetry.addData("SE Target, Current & power", seTarget + " ; " + SEMotor.getCurrentPosition() + " ; "+ SEMotor.getPower() );

        opMode.telemetry.update();
    }

    private void updateTelemetryForDistanceRange(double targetDistance, double currentDistance) {
        opMode.telemetry.addData("Target distance", targetDistance);
        opMode.telemetry.addData("Current distance", currentDistance);
        opMode.telemetry.addData("NW Target, Current & power", nwTarget + " ; " + NWMotor.getCurrentPosition() + " ; "+ NWMotor.getPower() );
        opMode.telemetry.addData("NE Target, Current & power", neTarget + " ; " + NEMotor.getCurrentPosition() + " ; "+ NEMotor.getPower() );
        opMode.telemetry.addData("SW Target, Current & power", swTarget + " ; " + SWMotor.getCurrentPosition() + " ; "+ SWMotor.getPower() );
        opMode.telemetry.addData("SE Target, Current & power", seTarget + " ; " + SEMotor.getCurrentPosition() + " ; "+ SEMotor.getPower() );

        opMode.telemetry.update();
    }

    private void setMotorPowerVariable(double nwPower, double nePower, double swPower, double sePower) {
        NWPower = nwPower;
        NEPower = nePower;
        SWPower = swPower;
        SEPower = sePower;
    }

    private void customizeDrivePowers() {
        NWPower = customizeMotorPower(NWPower);
        NEPower = customizeMotorPower(NEPower);
        SWPower = customizeMotorPower(SWPower);
        SEPower = customizeMotorPower(SEPower);

        addMinimumPower();
    }

    private double customizeMotorPower(double initialPower) {
        if (initialPower == 0) {
            return 0;
        }

        if (initialPower > .80) {
            return initialPower;
        }
        else if (initialPower > .60) {
            return initialPower * .5;
        }
        else if (initialPower > 0){
            return initialPower * .25;
        }

        // negative power
        else {
            if (initialPower < -.80) {
                return  initialPower;
            }
            else if (initialPower < -.60) {
                return  initialPower * .5;
            }
            else {
                return  initialPower * .25;
            }
        }
    }

    private void addMinimumPower() {
        if (NWPower > 0) NWPower += MINIMUN_POWER_TO_MOVE;
        else if (NWPower < 0) NWPower -= MINIMUN_POWER_TO_MOVE;

        if (NEPower > 0) NEPower += MINIMUN_POWER_TO_MOVE;
        else if (NEPower < 0) NEPower -= MINIMUN_POWER_TO_MOVE;

        if (SWPower > 0) SWPower += MINIMUN_POWER_TO_MOVE;
        else if (SWPower < 0) SWPower -= MINIMUN_POWER_TO_MOVE;

        if (SEPower > 0) SEPower += MINIMUN_POWER_TO_MOVE;
        else if (SEPower < 0) SEPower -= MINIMUN_POWER_TO_MOVE;

    }

    private void checkDPadForTeleop() {
        if (opMode.gamepad1.dpad_up) {
            setMotorPowerVariable(TOP_SPEED_SCALE, TOP_SPEED_SCALE, TOP_SPEED_SCALE, TOP_SPEED_SCALE);
        }
        else if (opMode.gamepad1.dpad_down) {
            setMotorPowerVariable(-TOP_SPEED_SCALE, -TOP_SPEED_SCALE, -TOP_SPEED_SCALE, -TOP_SPEED_SCALE);
        }
        else if (opMode.gamepad1.dpad_left) {
            setMotorPowerVariable(-TOP_SPEED_SCALE, TOP_SPEED_SCALE, TOP_SPEED_SCALE, -TOP_SPEED_SCALE);
        }
        else if (opMode.gamepad1.dpad_right) {
            setMotorPowerVariable(TOP_SPEED_SCALE, -TOP_SPEED_SCALE, -TOP_SPEED_SCALE, TOP_SPEED_SCALE);
        }
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

    private void normalize() {
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

    private void initialDriveMotorsSetup(LinearOpMode opMode) {
        NWMotor = opMode.hardwareMap.dcMotor.get("NWMotor");
        NEMotor = opMode.hardwareMap.dcMotor.get("NEMotor");
        SWMotor = opMode.hardwareMap.dcMotor.get("SWMotor");
        SEMotor = opMode.hardwareMap.dcMotor.get("SEMotor");

        NWMotor.setDirection(DcMotor.Direction.FORWARD);
        NEMotor.setDirection(DcMotor.Direction.REVERSE);
        SWMotor.setDirection(DcMotor.Direction.FORWARD);
        SEMotor.setDirection(DcMotor.Direction.REVERSE);

        setAllMotorPowersTheSame(0);
        setAllEncoders(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Method to drive on a fixed compass bearing (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the opmode running.
     *
     * @param speed    Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive(double speed,
                          double distance,
                          double angle) {

        int newLeftTarget;
        int newRightTarget;
        int adjustTicks;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            adjustTicks = (int) (distance * DriveTrain.COUNTS_PER_INCH);
            adjustAllCurrentTargets(adjustTicks, adjustTicks, adjustTicks, adjustTicks);

            setAllEncoders(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Math.abs(speed);
            setAllMotorPowersTheSame(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opMode.opModeIsActive() && (NWMotor.isBusy() && NEMotor.isBusy() && SWMotor.isBusy()&& SEMotor.isBusy())){

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if any one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                NWMotor.setPower(leftSpeed);
                SWMotor.setPower(leftSpeed);
                NEMotor.setPower(rightSpeed);
                SEMotor.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d:%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Actual", "%7d:%7d", NWMotor.getCurrentPosition(),
                        NEMotor.getCurrentPosition(), SEMotor.getCurrentPosition(), SWMotor.getCurrentPosition());
                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            setAllMotorPowersTheSame(0);

            // Turn off RUN_TO_POSITION
            NEMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            NWMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            SEMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            SWMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

}





