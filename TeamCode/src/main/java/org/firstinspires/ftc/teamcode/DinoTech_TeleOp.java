package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Device;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.network.DeviceNameManager;


@TeleOp(name="DrivingMode", group="Linear Opmode")

public class DinoTech_TeleOp extends LinearOpMode {
    /**
     * public DinoTech_TeleOp() {
     * super();
     * }
     **/

    // Elevator: Hub 2 Motor 2
    private DcMotor ElevatorMotor = null;

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

    private ElapsedTime runtime = new ElapsedTime();

    private int servoScaling = 1;

    public void runOpMode() {
        // Initialize and alert driver
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        LeftDrive = hardwareMap.get(DcMotor.class, "LeftDrive");
        RightDrive = hardwareMap.get(DcMotor.class, "RightDrive");
        ElevatorMotor = hardwareMap.get(DcMotor.class, "ElevatorMotor");
        Arm = hardwareMap.get(DcMotor.class, "Arm");
        Pulley = hardwareMap.get(DcMotor.class, "Pulley");
        Grabber = hardwareMap.get(Servo.class, "Grabber");


        telemetry.update();


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        LeftDrive.setDirection(DcMotor.Direction.REVERSE);
        RightDrive.setDirection(DcMotor.Direction.FORWARD);

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

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            LeftDrive.setPower(leftPower);
            RightDrive.setPower(rightPower);

            // Define Elevator Var
            double ElvPower = gamepad1.right_stick_y;

            //Send Calculated Power to Elevator
            ElevatorMotor.setPower(ElvPower);

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
