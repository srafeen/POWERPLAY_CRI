package org.firstinspires.ftc.teamcode.GameOpModes;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controllers.GamepadController;
import org.firstinspires.ftc.teamcode.SubSystems.BlinkinDisplay;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.SubSystems.Magazine;
import org.firstinspires.ftc.teamcode.SubSystems.MajorArm;
import org.firstinspires.ftc.teamcode.SubSystems.MinorArm;
import org.firstinspires.ftc.teamcode.SubSystems.Spinner;

/**
 * Ultimate Goal TeleOp mode <BR>
 *
 * This code defines the TeleOp mode is done by Hazmat Robot for Freight Frenzy<BR>
 *
 */
@TeleOp(name = "TeleOp", group = "00-Teleop")
public class TeleOpMode extends LinearOpMode {

    public GamepadController gamepadController;
    public DriveTrain driveTrain;
    public Intake intake;
    public Elevator elevator;
    public Magazine magazine;
    public Spinner spinner;
    public MajorArm majorArm;
    public MinorArm minorArm;
    public BlinkinDisplay blinkinDisplay;

    //public Vuforia Vuforia1;
    public Pose2d startPose = GameField.ORIGINPOSE;

    public ElapsedTime gameTimer = new ElapsedTime(MILLISECONDS);;

    @Override
    /**
     * Constructor for passing all the subsystems in order to make the subsystem be able to use
     * and work/be active
     */
    public void runOpMode() throws InterruptedException {

        /* Create Subsystem Objects*/
        driveTrain = new DriveTrain(hardwareMap);
        intake = new Intake(hardwareMap);
        elevator = new Elevator(hardwareMap);
        magazine = new Magazine(hardwareMap);
        spinner = new Spinner(hardwareMap);
        majorArm = new MajorArm(hardwareMap);
        minorArm = new MinorArm(hardwareMap);
        blinkinDisplay = new BlinkinDisplay(hardwareMap);

        /* Create Controllers */
        gamepadController = new GamepadController(gamepad1, gamepad2, driveTrain, intake, elevator,
                magazine, spinner, majorArm, minorArm, blinkinDisplay);

        GameField.playingAlliance= GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE;
        /* Get last position after Autonomous mode ended from static class set in Autonomous */
        if ( GameField.poseSetInAutonomous == true) {
            driveTrain.getLocalizer().setPoseEstimate(GameField.currentPose);
        } else {
            driveTrain.getLocalizer().setPoseEstimate(startPose);
        }

        GameField.debugLevel = GameField.DEBUG_LEVEL.NONE;
        //GameField.debugLevel = GameField.DEBUG_LEVEL.MAXIMUM;


        /* Set Initial State of any subsystem when TeleOp is to be started*/

        majorArm.moveMajorArmParkingPosition();
        elevator.moveElevatorLevel0Position();

        blinkinDisplay.setPatternBlack();
        /* Wait for Start or Stop Button to be pressed */
        waitForStart();
        gameTimer.reset();

        /* If Stop is pressed, exit OpMode */
        if (isStopRequested()) return;

        /*If Start is pressed, enter loop and exit only when Stop is pressed */
        while (!isStopRequested()) {

            if (GameField.debugLevel != GameField.DEBUG_LEVEL.NONE) {
                printDebugMessages();
                telemetry.update();
            }

            while (opModeIsActive()) {
                gamepadController.runByGamepadControl();

                if (gameTimer.time() > 80000 && gameTimer.time() < 90000) {
                    blinkinDisplay.setPatternEndGame();
                }

                if (GameField.debugLevel != GameField.DEBUG_LEVEL.NONE) {
                    printDebugMessages();
                    telemetry.update();
                }

            }

        }
        GameField.poseSetInAutonomous = false;
    }

    /**
     * Method to add debug messages. Update as telemetry.addData.
     * Use public attributes or methods if needs to be called here.
     */
    public void printDebugMessages(){
        telemetry.setAutoClear(true);
        telemetry.addData("DEBUG_LEVEL is : ", GameField.debugLevel);
        telemetry.addData("Robot ready to start","");

        if (GameField.debugLevel == GameField.DEBUG_LEVEL.MAXIMUM) {

            telemetry.addData("GameField.playingAlliance : ", GameField.playingAlliance);
            telemetry.addData("GameField.poseSetInAutonomous : ", GameField.poseSetInAutonomous);
            telemetry.addData("GameField.currentPose : ", GameField.currentPose);
            telemetry.addData("startPose : ", startPose);

            //****** Drive debug ******
            telemetry.addData("Drive Mode : ", driveTrain.driveMode);
            telemetry.addData("PoseEstimate :", driveTrain.poseEstimate);
            telemetry.addData("Battery Power", driveTrain.getBatteryVoltage(hardwareMap));

            telemetry.addData("Major Arm Position : ", majorArm.getMajorArmPosition());
            telemetry.addData("Major Claw State : ", majorArm.getMajorClawState());
            telemetry.addData("Major Arm Position Count : ", majorArm.getMajorArmPositionCount());
            telemetry.addData("Major Wrist Position : ", majorArm.majorWristServo.getPosition());

            telemetry.addData("Intake State : ", intake.getIntakeMotorState());
            telemetry.addData("Intake Motor Power : ", intake.getIntakeMotorPower());

            telemetry.addData("Elevator State : ", elevator.getElevatorState());
            telemetry.addData("Elevator encoder count : ", elevator.elevatorMotor.getCurrentPosition());
            telemetry.addData("Elevator Position Count : ", elevator.getElevatorPositionCount());

            telemetry.addData("Magazine State : ", magazine.getMagazineServoState());
            telemetry.addData("Magazine Color Sensor State : ", magazine.getMagazineColorSensorState());
            telemetry.addData("Magazine Color Sensor Distance :", "%.3f", magazine.getMagazineColorSensorDistance());
            telemetry.addData("Magazine Auto State : ", gamepadController.autoMagazine);

            telemetry.addData("Spinner State : ", spinner.getSpinnerMotorState());
            telemetry.addData("Spinner Motor Power : ", spinner.getSpinnerMotorPower());

            telemetry.addData("Minor Arm Position : ", minorArm.getMinorServoState());
            telemetry.addData("Minor Claw State : ", minorArm.getMinorClawState());

            telemetry.addData("Blinkin Pattern : ", blinkinDisplay.currentPattern);
            telemetry.addData("Game Timer : ", gameTimer.time());
        }

        telemetry.update();

    }
}
