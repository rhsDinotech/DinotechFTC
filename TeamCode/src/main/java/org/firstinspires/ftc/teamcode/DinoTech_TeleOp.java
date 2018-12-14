package org.firstinspires.ftc.teamcode;

//import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.Device;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;

//import org.firstinspires.ftc.robotcore.internal.network.DeviceNameManager;


@TeleOp(name="DrivingMode", group="Linear Opmode")

public class DinoTech_TeleOp extends LinearOpMode {
    /**
     * public DinoTech_TeleOp() {
     * super();
     * }
     **/


    // LeftDrive: Hub 1 Motor 0
    private DcMotor LeftDrive = null;

    // RightDrive:  Hub 2 Motor 0
    private DcMotor RightDrive = null;

    // Arm:  Hub 1 Motor 1
    private DcMotor Arm = null;

    // Pulley:  Hub 1 Motor 2
    private DcMotor Pulley = null;

    // Grabber: Hub 1 Servo 0
    private Servo Grabber = null;
    /**  End Instantiate Hardware Instances **/

    // setting for servo open/close acceleration
    //    -- increase / decrease for movement speed
    //    -- change +/- to reverse direction for right / left triggers
    private double servoScaling = .5;


    private ElapsedTime runtime = new ElapsedTime();



    public void runOpMode() {


        // Initialize hardware variables
        LeftDrive = hardwareMap.get(DcMotor.class, "LeftDrive");
        RightDrive = hardwareMap.get(DcMotor.class, "RightDrive");
        /*  Instantiate Hardware Instances */ /**  Instantiate Hardware Instances **/ // Elevator: Hub 2 Motor 2
        DcMotor elevatorMotor = hardwareMap.get(DcMotor.class, "ElevatorMotor");
        Arm = hardwareMap.get(DcMotor.class, "Arm");
        Pulley = hardwareMap.get(DcMotor.class, "Pulley");
        Grabber = hardwareMap.get(Servo.class, "Grabber");
        telemetry.update();


        // Initialize  Motor Directions
        LeftDrive.setDirection(DcMotor.Direction.REVERSE);
        RightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Initialize and alert driver
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.left_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            // Send calculated power to wheels
            LeftDrive.setPower(leftPower);
            RightDrive.setPower(rightPower);

            // Define Elevator Var
            double ElvPower = gamepad1.right_stick_y;

            //Send Calculated Power to Elevator
            elevatorMotor.setPower(ElvPower);

            // Define Arm Var
            double ArmPower = gamepad2.left_stick_y;

            // Send Calculated Power to Arm
            Arm.setPower(ArmPower);

            // Define Pulley Var
            double PulleyPower = gamepad2.right_stick_y;

            // Send Calculated Power to Pulley
            Pulley.setPower(PulleyPower);

            // Define Grabber Var
            double Open = gamepad2.right_trigger;
            double Close = gamepad2.left_trigger;
            double ServoPos = Grabber.getPosition();
            //Send
            if (Open > 0) {
                ServoPos += servoScaling;
                Grabber.setPosition(ServoPos);
            }
            if (Close > 0) {
                ServoPos -= servoScaling;
                Grabber.setPosition(ServoPos);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Elevator", "Elevator(%.2f)", ElvPower);
            telemetry.addData("Arm", "Arm(%.2f)", ArmPower);
            telemetry.addData("Pulley", "Pulley(%.2f)", PulleyPower);
            telemetry.update();
        }
    }
}
