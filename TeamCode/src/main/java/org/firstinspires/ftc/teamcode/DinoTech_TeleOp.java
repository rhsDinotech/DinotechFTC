package org.firstinspires.ftc.teamcode;

//import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.Device;
//import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.Range;
//import com.qualcomm.robotcore.util.Range;

//import org.firstinspires.ftc.robotcore.internal.network.DeviceNameManager;


@TeleOp(name="DrivingMode", group="Linear Opmode")

public class DinoTech_TeleOp extends LinearOpMode {

    // LeftDrive: Hub 1 Motor 0
    private DcMotor LeftDrive = null;

    // RightDrive:  Hub 2 Motor 0
    private DcMotor RightDrive = null;

    // Arm:  Hub 1 Motor 1
    private DcMotor Arm = null;

    // Pulley:  Hub 1 Motor 2
    private DcMotor Pulley = null;

    //Elevator: Hub2 Motor 1
    private DcMotor Elevator = null;

    // Grabber: Hub 1 Servo 0
    private Servo Grabber = null;
    //  End Instantiate Hardware Instances

    private int armtarget = 0;


    // setting for servo open/close acceleration
    //    -- increase / decrease for movement speed
    //    -- change +/- to reverse direction for right / left triggers
    private double servoScaling = 1;

    // Setup a variable for each drive wheel to save power level for telemetry
    double leftPower;
    double rightPower;


    private ElapsedTime runtime = new ElapsedTime();


    public void runOpMode() {


        /*  Instantiate Hardware Instances */
        LeftDrive = hardwareMap.get(DcMotor.class, "LeftDrive");
        RightDrive = hardwareMap.get(DcMotor.class, "RightDrive");


        // Elevator: Hub 2 Motor 2
        Elevator = hardwareMap.get(DcMotor.class, "ElevatorMotor");
        Arm = hardwareMap.get(DcMotor.class, "Arm");
        Pulley = hardwareMap.get(DcMotor.class, "Pulley");
        Grabber = hardwareMap.get(Servo.class, "Grabber");
        telemetry.update();


        // Initialize  Motor Directions
        LeftDrive.setDirection(DcMotor.Direction.REVERSE);
        RightDrive.setDirection(DcMotor.Direction.FORWARD);
        Elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // Initialize and alert driver
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            int leftPos = LeftDrive.getCurrentPosition();
            int rightPos = RightDrive.getCurrentPosition();
            int elvPos = Elevator.getCurrentPosition();
            int pulleyPos = Pulley.getCurrentPosition();
            double grabberPos = Grabber.getPosition();



            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.left_stick_x;
            leftPower = Range.clip(drive + turn, -1.0, 1.0);
            rightPower = Range.clip(drive - turn, -1.0, 1.0);

            // Send calculated power to wheels
            LeftDrive.setPower(leftPower);
            RightDrive.setPower(rightPower);

            // Define Elevator Var
            //telemetry.addData("Elevator:  ",  "%f", Elevator.getCurrentPosition());


            double ElvPower = -gamepad1.right_stick_y;

            //Send Calculated Power to Elevator
            Elevator.setPower(ElvPower);

            //Send Telemetry to controller
            //telemetry.addData("Elevator: ", "%f", Elevator.getCurrentPosition());


            // Define Arm Var
            Arm.setPower(.25);
            armtarget += gamepad2.left_stick_y;

            if (armtarget < 0) {
                armtarget = 0;
            }
            if (armtarget > 215) {
                armtarget = 215;
            }
            Arm.setTargetPosition(armtarget);

            // Define Pulley Var
            double PulleyPower = gamepad2.right_stick_y;

            // Send Calculated Power to Pulley
            Pulley.setPower(PulleyPower);

            // Define Grabber Var
            double Open = gamepad2.right_trigger;
            double Close = gamepad2.left_trigger;
            double ServoPos = Grabber.getPosition();

            //Send Telemetry to controller
            //telemetry.addData("Grabber1 ", "%f", Grabber.getPosition());


            //Send
            if (Open > 0 && Grabber.getPosition() < 1) {
                ServoPos += servoScaling;
                Grabber.setPosition(ServoPos);
            }
            if (Close > 0 && Grabber.getPosition() > 0); {
                ServoPos -= servoScaling;
                Grabber.setPosition(ServoPos);
            }
            telemetry.addData("Grabber2: ", "%f", Grabber.getPosition());


            //Show the elapsed game time and wheel power.
            // telemetry.addData("Status", "Run Time: " + runtime.toString());
            // telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            // telemetry.addData("Elevator", "Elevator(%.2f)", ElvPower);
            // telemetry.addData("Arm", "Arm(%.2f)", ArmPower);
            // telemetry.addData("Pulley", "Pulley(%.2f)", PulleyPower);

            telemetry.addData("ArmTarget", armtarget);
            telemetry.addData("LeftPos:  ", leftPos);
            telemetry.addData("RightPos:  ", rightPos);
            telemetry.addData("Elevator", elvPos);
            telemetry.addData( "Pulley", pulleyPos);
            telemetry.addData("ArmPos",Arm.getCurrentPosition());


            telemetry.update();
        }
    }
}
