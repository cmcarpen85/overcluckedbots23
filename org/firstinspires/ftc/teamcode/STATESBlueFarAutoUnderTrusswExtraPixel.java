package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "STATESBlueFarAutoUnderTrusswExtraPixel (Blocks to Java)")
public class STATESBlueFarAutoUnderTrusswExtraPixel extends LinearOpMode {

  private Servo outtakePivotServoAsServo;
  private Servo toggleServoAsServo;
  private DistanceSensor propDetectionLeft;
  private DistanceSensor propDetectionRightAsDistanceSensor;
  private Servo intakePivotServoAsServo;
  private BNO055IMU imuAsBNO055IMU;
  private Servo wristServoAsServo;
  private DcMotor backLeftMotor;
  private DcMotor backRightMotorAsDcMotor;
  private DcMotor frontLeftMotor;
  private DcMotor frontRightMotorAsDcMotor;
  private DcMotor leftLiftMotor;
  private DcMotor rightLiftMotorAsDcMotor;
  private DcMotor intakeMotorAsDcMotor;
  private CRServo passoffServoAsCRServo;
  private CRServo passOffServo2;

  int error2;
  double errorAngle;
  double newRampTime;
  int FLStartingEncoderPosition;
  double OldTargetAngle;
  float MeasuredAngle;
  int numTicks;
  int FRStartingEncoderPosition;
  double TargetAngle;
  double currentPower;
  double desiredWristAngle0;
  double lastRampTime;
  double averageFps;
  double desiredWristAngle1;
  double desiredWristAngle2;
  double desiredWristAngle3;
  double desiredWristAngle4;
  double desiredWristAngle5;
  double desiredWristAngle6;
  double scoringCenter;
  double scoringCentral;
  double scoringSecondary;
  int row1;
  int row0;
  int row2;
  int row3;
  int row4;
  int row5;
  int row6;
  int row7;
  int row8;
  int row9;
  int row10;
  int row11;
  int row12;
  int outtakeReady4PickupHeight;
  double rampPowerInterval;
  double GyroAdjustment;
  String detectedProp;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    outtakePivotServoAsServo = hardwareMap.get(Servo.class, "outtakePivotServoAsServo");
    toggleServoAsServo = hardwareMap.get(Servo.class, "toggleServoAsServo");
    propDetectionLeft = hardwareMap.get(DistanceSensor.class, "propDetectionLeft");
    propDetectionRightAsDistanceSensor = hardwareMap.get(DistanceSensor.class, "propDetectionRightAsDistanceSensor");
    intakePivotServoAsServo = hardwareMap.get(Servo.class, "intakePivotServoAsServo");
    imuAsBNO055IMU = hardwareMap.get(BNO055IMU.class, "imuAsBNO055IMU");
    wristServoAsServo = hardwareMap.get(Servo.class, "wristServoAsServo");
    backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
    backRightMotorAsDcMotor = hardwareMap.get(DcMotor.class, "backRightMotorAsDcMotor");
    frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
    frontRightMotorAsDcMotor = hardwareMap.get(DcMotor.class, "frontRightMotorAsDcMotor");
    leftLiftMotor = hardwareMap.get(DcMotor.class, "leftLiftMotor");
    rightLiftMotorAsDcMotor = hardwareMap.get(DcMotor.class, "rightLiftMotorAsDcMotor");
    intakeMotorAsDcMotor = hardwareMap.get(DcMotor.class, "intakeMotorAsDcMotor");
    passoffServoAsCRServo = hardwareMap.get(CRServo.class, "passoffServoAsCRServo");
    passOffServo2 = hardwareMap.get(CRServo.class, "passOffServo2");

    initialize();
    waitForStart();
    // Put initialization blocks here.
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        outtakePivotServoAsServo.setPosition(0.483);
        toggleServoAsServo.setPosition(0.48);
        sleep(300);
        callRowHeight(1, 0.5);
        DriveFWDwRamp(36.75, 1, 0.035, 16.8);
        TurnRight(0, 0.5);
        outtakePivotServoAsServo.setPosition(0.36);
        propDetection();
        telemetry.addData("Left", propDetectionLeft.getDistance(DistanceUnit.CM));
        telemetry.addData("Right", propDetectionRightAsDistanceSensor.getDistance(DistanceUnit.CM));
        telemetry.update();
        if (detectedProp.equals("Left")) {
          Drive_FWD(-6, 0.5);
          TurnW_ramp(-87.5, 1);
          callRowHeight(0.25, 0.35);
          sleep(250);
          outtakePivotServoAsServo.setPosition(0.396);
          sleep(250);
          Drive_FWD(0.5, 0.4);
          TurnRight(0, 0.35);
          sleep(100);
          releasePixel(false, true, false);
          sleep(100);
          outtakePivotServoAsServo.setPosition(0.36);
          Drive_FWD(-2, 0.5);
          outtakePivotServoAsServo.setPosition(0.39);
          callRowHeight(1, 0.35);
          TurnRight(0, 0.5);
          sleep(300);
          pivotAngle(0);
          sleep(100);
          releasePixel(false, false, true);
          sleep(100);
          callRowHeight(0.5, 0.5);
          setWristAngle(desiredWristAngle0);
          FourbarPosition("3");
          Drive_FWD(-13, 0.5);
          runIntake("In");
          TurnRight(0, 0.5);
          passoffConveyor("In");
          Drive_FWD(-2, 0.35);
          sleep(1500);
          runIntake("Out");
          sleep(200);
          passoffConveyor("In");
          sleep(150);
          passoffConveyor("In");
          FourbarPosition("Rest");
          callRowHeight(0.5, 0.5);
          Drive_FWD(13, 0.5);
          TurnRight(0, 0.5);
          StrafeRight(-25, 0.5);
          runIntake("Rest");
          TurnRight(0, 0.5);
          // Process ends
          // Scoring on board from far pos.
          sleep(150);
          DriveFWDwRamp(86.5, 1, 0.035, 35);
          sleep(100);
          pivotAngle(0);
          passoffConveyor("Rest");
          callRowHeight(0, 0.5);
          sleep(250);
          StrafeRight(12.5, 0.5);
          callRowHeight(2, 0.75);
          TurnRight(0, 0.4);
          pivotAngle(1);
          sleep(375);
          setWristAngle(desiredWristAngle1);
          Drive_FWD(10, 0.3);
          TurnRight(0, 0.5);
          releasePixel(true, true, false);
          toggleServoAsServo.setPosition(scoringSecondary);
          Drive_FWD(-2, 0.2);
          TurnRight(0, 0.5);
          StrafeRight(-18, 0.5);
          setWristAngle(desiredWristAngle0);
          sleep(250);
          pivotAngle(0);
          sleep(250);
          callRowHeight(0, 0.5);
          TurnRight(85, 0.5);
        } else if (detectedProp.equals("Right")) {
          Drive_FWD(-4.5, 0.5);
          sleep(100);
          StrafeRight(10, 0.5);
          callRowHeight(0.25, 0.5);
          TurnRight(0, 0.5);
          sleep(100);
          outtakePivotServoAsServo.setPosition(0.396);
          sleep(250);
          releasePixel(false, true, false);
          sleep(250);
          outtakePivotServoAsServo.setPosition(0.36);
          sleep(250);
          callRowHeight(1, 0.5);
          outtakePivotServoAsServo.setPosition(0.39);
          Drive_FWD(-2, 0.5);
          TurnW_ramp(-88, 1);
          FourbarPosition("3");
          pivotAngle(0);
          sleep(100);
          Drive_FWD(-3, 0.5);
          TurnRight(0, 0.5);
          Drive_FWD(-1, 0.3);
          TurnRight(0, 0.5);
          setWristAngle(desiredWristAngle0);
          passoffConveyor("In");
          runIntake("In");
          Drive_FWD(-1, 0.35);
          sleep(1500);
          runIntake("Out");
          passoffConveyor("In");
          sleep(150);
          passoffConveyor("In");
          FourbarPosition("Rest");
          callRowHeight(0.5, 0.5);
          Drive_FWD(19, 0.5);
          TurnRight(0, 0.5);
          StrafeRight(-21.5, 0.5);
          runIntake("Rest");
          TurnRight(0, 0.5);
          sleep(150);
          DriveFWDwRamp(86.5, 1, 0.035, 35);
          TurnRight(0, 0.5);
          passoffConveyor("Rest");
          setWristAngle(desiredWristAngle0);
          callRowHeight(0, 0.6);
          sleep(250);
          StrafeRight(28, 0.5);
          callRowHeight(2, 0.75);
          sleep(250);
          TurnRight(0, 0.4);
          pivotAngle(1);
          sleep(250);
          setWristAngle(desiredWristAngle6);
          Drive_FWD(6, 0.4);
          TurnRight(0, 0.5);
          releasePixel(true, false, false);
          toggleServoAsServo.setPosition(scoringCentral);
          Drive_FWD(-3, 0.2);
          TurnRight(0, 0.5);
          StrafeRight(-9, 0.75);
          TurnRight(0, 0.5);
          Drive_FWD(3, 0.2);
          TurnRight(0, 0.5);
          toggleServoAsServo.setPosition(scoringSecondary);
          Drive_FWD(-4, 0.2);
          setWristAngle(desiredWristAngle0);
          sleep(250);
          pivotAngle(0);
          StrafeRight(-11, 0.6);
          callRowHeight(0, 0.5);
          TurnRight(85, 0.5);
        } else {
          Drive_FWD(-4, 0.4);
          // Process for drop on spikemark
          callRowHeight(0.25, 0.35);
          sleep(250);
          outtakePivotServoAsServo.setPosition(0.396);
          sleep(250);
          releasePixel(false, true, false);
          sleep(100);
          outtakePivotServoAsServo.setPosition(0.36);
          Drive_FWD(-2.5, 0.35);
          outtakePivotServoAsServo.setPosition(0.39);
          callRowHeight(1, 0.35);
          TurnW_ramp(-88, 1);
          sleep(300);
          pivotAngle(0);
          sleep(100);
          releasePixel(false, false, true);
          sleep(100);
          callRowHeight(0.5, 0.35);
          setWristAngle(desiredWristAngle0);
          FourbarPosition("3");
          Drive_FWD(-20.5, 0.35);
          runIntake("In");
          TurnRight(0, 0.35);
          passoffConveyor("In");
          sleep(1300);
          runIntake("Out");
          passoffConveyor("In");
          sleep(150);
          passoffConveyor("In");
          FourbarPosition("Rest");
          callRowHeight(0.5, 0.5);
          Drive_FWD(13, 0.5);
          TurnRight(0, 0.5);
          StrafeRight(-23, 0.5);
          runIntake("Rest");
          TurnRight(0, 0.5);
          sleep(150);
          DriveFWDwRamp(86.5, 1, 0.035, 35);
          TurnRight(0, 0.4);
          passoffConveyor("Rest");
          callRowHeight(0, 0.75);
          sleep(250);
          StrafeRight(20, 0.5);
          callRowHeight(2, 0.75);
          sleep(250);
          TurnRight(0, 0.4);
          pivotAngle(1);
          sleep(375);
          setWristAngle(desiredWristAngle6);
          Drive_FWD(13, 0.3);
          toggleServoAsServo.setPosition(scoringCentral);
          Drive_FWD(-2, 0.2);
          TurnRight(0, 0.5);
          StrafeRight(-4, 0.5);
          TurnRight(0, 0.5);
          Drive_FWD(1, 0.2);
          TurnRight(0, 0.5);
          toggleServoAsServo.setPosition(scoringSecondary);
          Drive_FWD(-2, 0.2);
          setWristAngle(desiredWristAngle0);
          StrafeRight(-12, 0.5);
          pivotAngle(0);
          TurnRight(85, 0.5);
          callRowHeight(0, 0.5);
        }
        telemetry.update();
        sleep(30000);
      }
    }
    waitForStart();
  }

  /**
   * We need to change the heights-these are the ones from last year
   */
  private void FourbarPosition(String IntakeRest12) {
    // Remember to change the servo after configuring robot!
    if (IntakeRest12.equals("Intake")) {
      intakePivotServoAsServo.setPosition(0.525);
    } else if (IntakeRest12.equals("Rest")) {
      intakePivotServoAsServo.setPosition(0.445);
    } else if (IntakeRest12.equals("1")) {
      intakePivotServoAsServo.setPosition(0.501);
    } else if (IntakeRest12.equals("2")) {
      intakePivotServoAsServo.setPosition(0.495);
    } else if (IntakeRest12.equals("3")) {
      intakePivotServoAsServo.setPosition(0.4975);
    }
  }

  /**
   * Describe this function...
   */
  private void pivotAngle(int pivotOrientation) {
    if (pivotOrientation == 0) {
      outtakePivotServoAsServo.setPosition(0.4872);
    } else if (pivotOrientation == 1) {
      outtakePivotServoAsServo.setPosition(0.4);
    }
  }

  /**
   * Describe this function...
   */
  private void initialize() {
    int INTAKE_TICKS_PER_REVOLUTION;
    int liftRightPosition;
    int liftLeftPosition;
    int baseHeight;
    boolean detectTapeLeft;
    boolean detectTapeRight;

    imuAsBNO055IMU.initialize(new BNO055IMU.Parameters());
    // Verify all initalizations before "TargetAngle=0"
    INTAKE_TICKS_PER_REVOLUTION = 28;
    TargetAngle = 0;
    GyroAdjustment = 0.02;
    liftRightPosition = 0;
    liftLeftPosition = 0;
    baseHeight = 0;
    row0 = 0;
    row1 = 225;
    row2 = 310;
    row3 = 400;
    row4 = 480;
    row5 = 565;
    row6 = 645;
    row7 = 725;
    outtakeReady4PickupHeight = 65;
    // Disabled because does not go to that height
    // The stuff for wrist function
    wristServoAsServo.setPosition(0.5105);
    // The stuff for toggle function
    scoringCenter = 0.497;
    scoringSecondary = 0.513;
    scoringCentral = 0.475;
    toggleServoAsServo.setPosition(scoringCenter);
    desiredWristAngle0 = 0.5115;
    desiredWristAngle1 = 0.605;
    desiredWristAngle2 = 0.575;
    desiredWristAngle3 = 0.545;
    desiredWristAngle4 = 0.4875;
    desiredWristAngle5 = 0.452;
    desiredWristAngle6 = 0.415;
    // The stuff for pivot function
    outtakePivotServoAsServo.setPosition(0.4872);
    backLeftMotor.setTargetPosition(0);
    backRightMotorAsDcMotor.setTargetPosition(0);
    frontLeftMotor.setTargetPosition(0);
    frontRightMotorAsDcMotor.setTargetPosition(0);
    leftLiftMotor.setTargetPosition(0);
    rightLiftMotorAsDcMotor.setTargetPosition(0);
    intakeMotorAsDcMotor.setTargetPosition(0);
    leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightLiftMotorAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backRightMotorAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontRightMotorAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    intakeMotorAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
    frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
    rightLiftMotorAsDcMotor.setDirection(DcMotor.Direction.REVERSE);
    backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    backRightMotorAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    frontRightMotorAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightLiftMotorAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    intakeMotorAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backRightMotorAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    frontRightMotorAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    intakeMotorAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    FourbarPosition("Rest");
    detectTapeLeft = false;
    detectTapeRight = false;
  }

  /**
   * Describe this function...
   */
  private void passoffConveyor(String passoffConveyor2) {
    if (passoffConveyor2.equals("In")) {
      passoffServoAsCRServo.setDirection(CRServo.Direction.REVERSE);
      passOffServo2.setDirection(CRServo.Direction.FORWARD);
      passoffServoAsCRServo.setPower(1);
      passOffServo2.setPower(0.5);
    } else if (passoffConveyor2.equals("Out")) {
      passoffServoAsCRServo.setDirection(CRServo.Direction.FORWARD);
      passOffServo2.setDirection(CRServo.Direction.REVERSE);
      passoffServoAsCRServo.setPower(1);
      passOffServo2.setPower(0.5);
    } else if (passoffConveyor2.equals("Rest")) {
      passoffServoAsCRServo.setDirection(CRServo.Direction.REVERSE);
      passOffServo2.setDirection(CRServo.Direction.FORWARD);
      passoffServoAsCRServo.setPower(0);
      passOffServo2.setPower(0);
    } else if (passoffConveyor2.equals("getReady")) {
      passoffServoAsCRServo.setDirection(CRServo.Direction.FORWARD);
      passOffServo2.setDirection(CRServo.Direction.REVERSE);
      passoffServoAsCRServo.setPower(1);
      passOffServo2.setPower(0.5);
      sleep(150);
      passoffServoAsCRServo.setPower(0);
      passOffServo2.setPower(0);
    }
  }

  /**
   * Describe this function...
   */
  private void callRowHeight(double rowNumber, double VSPower) {
    // Make subfunction as a safety function
    if (rowNumber == 1) {
      leftLiftMotor.setTargetPosition(row1);
      rightLiftMotorAsDcMotor.setTargetPosition(row1);
      rightLiftMotorAsDcMotor.setPower(VSPower);
      leftLiftMotor.setPower(VSPower);
    } else if (rowNumber == 0) {
      leftLiftMotor.setTargetPosition(row0);
      rightLiftMotorAsDcMotor.setTargetPosition(row0);
      rightLiftMotorAsDcMotor.setPower(VSPower);
      leftLiftMotor.setPower(VSPower);
    } else if (rowNumber == 2) {
      rightLiftMotorAsDcMotor.setTargetPosition(row2);
      leftLiftMotor.setTargetPosition(row2);
      rightLiftMotorAsDcMotor.setPower(VSPower);
      leftLiftMotor.setPower(VSPower);
    } else if (rowNumber == 3) {
      rightLiftMotorAsDcMotor.setTargetPosition(row3);
      leftLiftMotor.setTargetPosition(row3);
      rightLiftMotorAsDcMotor.setPower(VSPower);
      leftLiftMotor.setPower(VSPower);
    } else if (rowNumber == 4) {
      rightLiftMotorAsDcMotor.setTargetPosition(row4);
      leftLiftMotor.setTargetPosition(row4);
      rightLiftMotorAsDcMotor.setPower(VSPower);
      leftLiftMotor.setPower(VSPower);
    } else if (rowNumber == 5) {
      rightLiftMotorAsDcMotor.setTargetPosition(row5);
      leftLiftMotor.setTargetPosition(row5);
      rightLiftMotorAsDcMotor.setPower(VSPower);
      leftLiftMotor.setPower(VSPower);
    } else if (rowNumber == 6) {
      rightLiftMotorAsDcMotor.setTargetPosition(row6);
      leftLiftMotor.setTargetPosition(row6);
      rightLiftMotorAsDcMotor.setPower(VSPower);
      leftLiftMotor.setPower(VSPower);
    } else if (rowNumber == 7) {
      rightLiftMotorAsDcMotor.setTargetPosition(row7);
      leftLiftMotor.setTargetPosition(row7);
      rightLiftMotorAsDcMotor.setPower(VSPower);
      leftLiftMotor.setPower(VSPower);
    } else if (rowNumber == 8) {
      rightLiftMotorAsDcMotor.setTargetPosition(row8);
      leftLiftMotor.setTargetPosition(row8);
      rightLiftMotorAsDcMotor.setPower(VSPower);
      leftLiftMotor.setPower(VSPower);
    } else if (rowNumber == 9) {
      rightLiftMotorAsDcMotor.setTargetPosition(row9);
      leftLiftMotor.setTargetPosition(row9);
      rightLiftMotorAsDcMotor.setPower(VSPower);
      leftLiftMotor.setPower(VSPower);
    } else if (rowNumber == 10) {
      rightLiftMotorAsDcMotor.setTargetPosition(row10);
      leftLiftMotor.setTargetPosition(row10);
      rightLiftMotorAsDcMotor.setPower(VSPower);
      leftLiftMotor.setPower(VSPower);
    } else if (rowNumber == 11) {
      rightLiftMotorAsDcMotor.setTargetPosition(row11);
      leftLiftMotor.setTargetPosition(row11);
      rightLiftMotorAsDcMotor.setPower(VSPower);
      leftLiftMotor.setPower(VSPower);
    } else if (rowNumber == 12) {
      rightLiftMotorAsDcMotor.setTargetPosition(row12);
      leftLiftMotor.setTargetPosition(row12);
      rightLiftMotorAsDcMotor.setPower(VSPower);
      leftLiftMotor.setPower(VSPower);
    } else if (rowNumber == 0.5) {
      rightLiftMotorAsDcMotor.setTargetPosition(outtakeReady4PickupHeight);
      leftLiftMotor.setTargetPosition(outtakeReady4PickupHeight);
      rightLiftMotorAsDcMotor.setPower(VSPower);
      leftLiftMotor.setPower(VSPower);
    } else if (rowNumber == 0.75) {
      rightLiftMotorAsDcMotor.setTargetPosition(110);
      leftLiftMotor.setTargetPosition(110);
      rightLiftMotorAsDcMotor.setPower(VSPower);
      leftLiftMotor.setPower(VSPower);
    } else if (rowNumber == 0.25) {
      rightLiftMotorAsDcMotor.setTargetPosition(77);
      leftLiftMotor.setTargetPosition(77);
      rightLiftMotorAsDcMotor.setPower(VSPower);
      leftLiftMotor.setPower(VSPower);
    }
  }

  /**
   * Describe this function...
   */
  private void error() {
    error2 = frontLeftMotor.getTargetPosition() - frontLeftMotor.getCurrentPosition();
  }

  /**
   * Describe this function...
   */
  private void angleError() {
    errorAngle = TargetAngle - ReturnMeasuredAngle();
  }

  /**
   * We need to add the sensor inputs
   */
  private void propDetection() {
    double leftDetect;
    double midDetect;
    double rightDetect;

    // We need to add the sensor inputs
    leftDetect = propDetectionLeft.getDistance(DistanceUnit.CM);
    midDetect = propDetectionRightAsDistanceSensor.getDistance(DistanceUnit.CM);
    rightDetect = propDetectionRightAsDistanceSensor.getDistance(DistanceUnit.CM);
    if (leftDetect < 13) {
      detectedProp = "Left";
    } else if (rightDetect < 13) {
      detectedProp = "Right";
    } else {
      detectedProp = "Mid";
    }
  }

  /**
   * Describe this function...
   */
  private void rampUpRampDown(int desiredPosition, int maxDrivePower, double RampDistance, double rampPower) {
    newRampTime = getRuntime() - lastRampTime;
    numTicks = numTicks + 1;
    averageFps = (newRampTime + averageFps) / numTicks;
    rampPowerInterval = (newRampTime / 0.06) * rampPower;
    if (Math.abs(error2) <= (RampDistance * 0.5 + RampDistance) * 43) {
      if (currentPower > 0.08) {
        currentPower = currentPower - rampPowerInterval * 0.95;
      } else {
        currentPower = 0.08;
      }
    } else {
      if (currentPower < 0.2) {
        currentPower = currentPower + rampPowerInterval * 2;
      } else if (currentPower < maxDrivePower) {
        currentPower = currentPower + rampPowerInterval * 1;
      }
    }
    lastRampTime = getRuntime();
  }

  /**
   * Describe this function...
   */
  private void yield() {
    String passoffConveyor2;
    double rowNumber;
    double VSPower;
    int callRowHeightStartTime;
    int callRowHeightStartTime2;
    double rowNumber2;
    double VSPower2;
    int passoffConveyerStartTime;

    if (getRuntime() > callRowHeightStartTime) {
      callRowHeight(rowNumber, VSPower);
      callRowHeightStartTime = 0;
    }
    if (getRuntime() > callRowHeightStartTime2) {
      callRowHeight(rowNumber2, VSPower2);
      callRowHeightStartTime2 = 0;
    }
    if (getRuntime() > passoffConveyerStartTime) {
      passoffConveyor(passoffConveyor2);
      passoffConveyerStartTime = 0;
    }
  }

  /**
   * Describe this function...
   */
  private void Drive_FWD(double Distance, double Power) {
    FLStartingEncoderPosition = frontLeftMotor.getCurrentPosition();
    FRStartingEncoderPosition = frontRightMotorAsDcMotor.getCurrentPosition();
    frontLeftMotor.setTargetPosition((int) (FLStartingEncoderPosition + 43 * Distance));
    frontRightMotorAsDcMotor.setTargetPosition((int) (FRStartingEncoderPosition + 43 * Distance));
    if (Distance > 0) {
      // Running Forward
      while (frontLeftMotor.getCurrentPosition() < frontLeftMotor.getTargetPosition()) {
        if (TargetAngle > 0) {
          // If we are moving forward and aiming angle>0
          if (ReturnMeasuredAngle() > TargetAngle || ReturnMeasuredAngle() < -90) {
            backLeftMotor.setPower(Power - GyroAdjustment);
            frontLeftMotor.setPower(Power - GyroAdjustment);
            backRightMotorAsDcMotor.setPower(Power);
            frontRightMotorAsDcMotor.setPower(Power);
          } else {
            backLeftMotor.setPower(Power);
            frontLeftMotor.setPower(Power);
            backRightMotorAsDcMotor.setPower(Power - GyroAdjustment);
            frontRightMotorAsDcMotor.setPower(Power - 0.05);
          }
        } else {
          // If we are moving forward and aiming angle<0
          if (ReturnMeasuredAngle() > TargetAngle && ReturnMeasuredAngle() < 90) {
            backLeftMotor.setPower(Power - GyroAdjustment);
            frontLeftMotor.setPower(Power - GyroAdjustment);
            backRightMotorAsDcMotor.setPower(Power);
            frontRightMotorAsDcMotor.setPower(Power);
          } else {
            backLeftMotor.setPower(Power);
            frontLeftMotor.setPower(Power);
            backRightMotorAsDcMotor.setPower(Power - GyroAdjustment);
            frontRightMotorAsDcMotor.setPower(Power - GyroAdjustment);
          }
        }
      }
    } else {
      // Running Backward
      while (frontLeftMotor.getCurrentPosition() > frontLeftMotor.getTargetPosition()) {
        if (TargetAngle > 0) {
          // If we are moving backward and aiming angle>0
          if (ReturnMeasuredAngle() > TargetAngle || MeasuredAngle < -90) {
            backLeftMotor.setPower(-Power);
            frontLeftMotor.setPower(-Power);
            backRightMotorAsDcMotor.setPower(-Power + GyroAdjustment);
            frontRightMotorAsDcMotor.setPower(-Power + GyroAdjustment);
          } else {
            backLeftMotor.setPower(-Power + GyroAdjustment);
            frontLeftMotor.setPower(-Power + GyroAdjustment);
            backRightMotorAsDcMotor.setPower(-Power);
            frontRightMotorAsDcMotor.setPower(-Power);
          }
        } else {
          // If we are moving backwardz and aiming angle<0
          if (ReturnMeasuredAngle() > TargetAngle && ReturnMeasuredAngle() < 90) {
            backLeftMotor.setPower(-Power);
            frontLeftMotor.setPower(-Power);
            backRightMotorAsDcMotor.setPower(-Power + GyroAdjustment);
            frontRightMotorAsDcMotor.setPower(-Power + GyroAdjustment);
          } else {
            backLeftMotor.setPower(-Power + GyroAdjustment);
            frontLeftMotor.setPower(-Power + GyroAdjustment);
            backRightMotorAsDcMotor.setPower(-Power);
            frontRightMotorAsDcMotor.setPower(-Power);
          }
        }
      }
    }
    backLeftMotor.setPower(0);
    backRightMotorAsDcMotor.setPower(0);
    frontLeftMotor.setPower(0);
    frontRightMotorAsDcMotor.setPower(0);
    frontRightMotorAsDcMotor.setTargetPosition(frontRightMotorAsDcMotor.getCurrentPosition());
  }

  /**
   * Describe this function...
   */
  private void rampUpRampDownAng(double desiredAngle, double maxDrivePower, double RampPow, int RampAngle) {
    double ROC;

    ROC = (maxDrivePower * 2) / desiredAngle;
    if (Math.abs(errorAngle) >= Math.abs(TargetAngle) - RampAngle) {
      if (currentPower < maxDrivePower) {
        currentPower = currentPower + RampPow;
      }
    } else if (Math.abs(errorAngle) <= RampAngle + 5) {
      if (currentPower > 0.1) {
        currentPower = currentPower - RampPow * 2.5;
      }
    } else {
      if (currentPower > 0.1) {
        currentPower = currentPower;
      }
    }
  }

  /**
   * Describe this function...
   */
  private void dropOnSpikemark() {
    callRowHeight(1, 0.5);
    sleep(1000);
    outtakePivotServoAsServo.setPosition(0.36);
    sleep(1000);
    wristServoAsServo.setPosition(0.38);
    sleep(1000);
    outtakePivotServoAsServo.setPosition(0.485);
    sleep(1000);
    callRowHeight(0.75, 0.5);
    // We deleted realesepixel, because we wanted to do different steps throughout auto
  }

  /**
   * Describe this function...
   */
  private void DriveFWDwRamp(double Distance, int Power, double rampPower, double RampDistance) {
    FLStartingEncoderPosition = frontLeftMotor.getCurrentPosition();
    FRStartingEncoderPosition = frontRightMotorAsDcMotor.getCurrentPosition();
    currentPower = 0.1;
    frontLeftMotor.setTargetPosition((int) (FLStartingEncoderPosition + 43 * Distance));
    frontRightMotorAsDcMotor.setTargetPosition((int) (FRStartingEncoderPosition + 43 * Distance));
    lastRampTime = getRuntime();
    if (Distance > 0) {
      // Running Forward
      while (frontLeftMotor.getCurrentPosition() < frontLeftMotor.getTargetPosition()) {
        yield();
        telemetry.addData("current position", frontLeftMotor.getCurrentPosition());
        telemetry.addData("current power", currentPower);
        telemetry.addData("RMPpowerint", rampPowerInterval);
        error();
        telemetry.addData("NumTicks", numTicks);
        telemetry.addData("DesiredPower", Power);
        telemetry.addData("error", error2);
        telemetry.addData("averageFps", averageFps);
        telemetry.addData("time", newRampTime);
        telemetry.addData("Target Angle", TargetAngle);
        telemetry.addData("measured angle", ReturnMeasuredAngle());
        rampUpRampDown(frontLeftMotor.getTargetPosition(), Power, RampDistance, rampPower);
        telemetry.update();
        if (TargetAngle > 0) {
          // If we are moving forward and aiming angle>0
          if (ReturnMeasuredAngle() > TargetAngle || ReturnMeasuredAngle() < -90) {
            backLeftMotor.setPower(currentPower - GyroAdjustment);
            frontLeftMotor.setPower(currentPower - GyroAdjustment);
            backRightMotorAsDcMotor.setPower(currentPower);
            frontRightMotorAsDcMotor.setPower(currentPower);
          } else {
            backLeftMotor.setPower(currentPower);
            frontLeftMotor.setPower(currentPower);
            backRightMotorAsDcMotor.setPower(currentPower - GyroAdjustment);
            frontRightMotorAsDcMotor.setPower(currentPower - GyroAdjustment);
          }
        } else {
          // If we are moving forward and aiming angle<0
          if (ReturnMeasuredAngle() > TargetAngle && ReturnMeasuredAngle() < 90) {
            backLeftMotor.setPower(currentPower - GyroAdjustment);
            frontLeftMotor.setPower(currentPower - GyroAdjustment);
            backRightMotorAsDcMotor.setPower(currentPower);
            frontRightMotorAsDcMotor.setPower(currentPower);
          } else {
            backLeftMotor.setPower(currentPower);
            frontLeftMotor.setPower(currentPower);
            backRightMotorAsDcMotor.setPower(currentPower - GyroAdjustment);
            frontRightMotorAsDcMotor.setPower(currentPower - GyroAdjustment);
          }
        }
      }
      lastRampTime = 0;
      numTicks = 0;
    } else {
      // Running Backward
      while (frontLeftMotor.getCurrentPosition() > frontLeftMotor.getTargetPosition()) {
        yield();
        telemetry.addData("current position", frontLeftMotor.getCurrentPosition());
        telemetry.addData("current power", currentPower);
        telemetry.addData("NumTicks", numTicks);
        error();
        telemetry.addData("error", error2);
        rampUpRampDown(frontLeftMotor.getTargetPosition(), Power, RampDistance, rampPower);
        telemetry.update();
        if (TargetAngle > 0) {
          // If we are moving backward and aiming angle>0
          if (ReturnMeasuredAngle() > TargetAngle && ReturnMeasuredAngle() < -90) {
            backLeftMotor.setPower(-currentPower);
            frontLeftMotor.setPower(-currentPower);
            backRightMotorAsDcMotor.setPower(-currentPower + GyroAdjustment);
            frontRightMotorAsDcMotor.setPower(-currentPower + GyroAdjustment);
          } else {
            backLeftMotor.setPower(-currentPower + GyroAdjustment);
            frontLeftMotor.setPower(-currentPower + GyroAdjustment);
            backRightMotorAsDcMotor.setPower(-currentPower);
            frontRightMotorAsDcMotor.setPower(-currentPower);
          }
        } else {
          // If we are moving backwardz and aiming angle<0
          if (ReturnMeasuredAngle() > TargetAngle && ReturnMeasuredAngle() < 90) {
            backLeftMotor.setPower(-currentPower);
            frontLeftMotor.setPower(-currentPower);
            backRightMotorAsDcMotor.setPower(-currentPower + GyroAdjustment);
            frontRightMotorAsDcMotor.setPower(-currentPower + GyroAdjustment);
          } else {
            backLeftMotor.setPower(-currentPower + GyroAdjustment);
            frontLeftMotor.setPower(-currentPower + GyroAdjustment);
            backRightMotorAsDcMotor.setPower(-currentPower);
            frontRightMotorAsDcMotor.setPower(-currentPower);
          }
        }
      }
      lastRampTime = 0;
      numTicks = 0;
    }
    backLeftMotor.setPower(0);
    backRightMotorAsDcMotor.setPower(0);
    frontLeftMotor.setPower(0);
    frontRightMotorAsDcMotor.setPower(0);
    error();
    telemetry.addData("Target Angle", TargetAngle);
    telemetry.addData("measured angle", ReturnMeasuredAngle());
    telemetry.addData("current position", frontLeftMotor.getCurrentPosition());
    telemetry.addData("error", error2);
    telemetry.addData("averageFps", averageFps);
    telemetry.addData("done", 1);
    telemetry.update();
    frontRightMotorAsDcMotor.setTargetPosition(frontRightMotorAsDcMotor.getCurrentPosition());
    frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition());
  }

  /**
   * Describe this function...
   */
  private void setWristAngle(double my_0123456) {
    if (my_0123456 == desiredWristAngle0) {
      wristServoAsServo.setPosition(desiredWristAngle0);
    } else if (my_0123456 == desiredWristAngle1) {
      wristServoAsServo.setPosition(desiredWristAngle1);
    } else if (my_0123456 == desiredWristAngle2) {
      wristServoAsServo.setPosition(desiredWristAngle2);
    } else if (my_0123456 == desiredWristAngle3) {
      wristServoAsServo.setPosition(desiredWristAngle3);
    } else if (my_0123456 == desiredWristAngle4) {
      wristServoAsServo.setPosition(desiredWristAngle4);
    } else if (my_0123456 == desiredWristAngle5) {
      wristServoAsServo.setPosition(desiredWristAngle5);
    } else if (my_0123456 == desiredWristAngle6) {
      wristServoAsServo.setPosition(desiredWristAngle6);
    }
  }

  /**
   * Describe this function...
   */
  private void TurnRight(int Angle, double Power) {
    OldTargetAngle = TargetAngle;
    if (OldTargetAngle + Angle > 180) {
      TargetAngle = (OldTargetAngle + Angle) - 360;
    } else if (OldTargetAngle + Angle < -180) {
      TargetAngle = OldTargetAngle + Angle + 360;
    } else {
      TargetAngle = OldTargetAngle + Angle;
    }
    FLStartingEncoderPosition = frontLeftMotor.getCurrentPosition();
    FRStartingEncoderPosition = frontRightMotorAsDcMotor.getCurrentPosition();
    frontLeftMotor.setTargetPosition((int) (FLStartingEncoderPosition + 10.6 * Angle));
    frontRightMotorAsDcMotor.setTargetPosition((int) (FRStartingEncoderPosition - 10.6 * Angle));
    if (Angle == 0) {
    } else if (Angle > 0) {
      // Turning Right
      while (frontLeftMotor.getCurrentPosition() < frontLeftMotor.getTargetPosition() && opModeIsActive()) {
        backLeftMotor.setPower(Power);
        backRightMotorAsDcMotor.setPower(-Power);
        frontLeftMotor.setPower(Power);
        frontRightMotorAsDcMotor.setPower(-Power);
      }
    } else {
      // Turning Left
      while (frontLeftMotor.getCurrentPosition() > frontLeftMotor.getTargetPosition() && opModeIsActive()) {
        backLeftMotor.setPower(-Power);
        backRightMotorAsDcMotor.setPower(Power);
        frontLeftMotor.setPower(-Power);
        frontRightMotorAsDcMotor.setPower(Power);
      }
    }
    backLeftMotor.setPower(0);
    backRightMotorAsDcMotor.setPower(0);
    frontLeftMotor.setPower(0);
    frontRightMotorAsDcMotor.setPower(0);
    sleep(100);
    if (TargetAngle > 0) {
      // TargetAngle >0
      if (ReturnMeasuredAngle() > TargetAngle + 1 || MeasuredAngle < -90) {
        while ((ReturnMeasuredAngle() > TargetAngle + 1 || MeasuredAngle < -90) && opModeIsActive()) {
          frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
          frontRightMotorAsDcMotor.setPower(0.1);
          backRightMotorAsDcMotor.setPower(0.1);
          frontLeftMotor.setPower(-0.1);
          backLeftMotor.setPower(-0.1);
        }
      } else if (ReturnMeasuredAngle() < TargetAngle - 1 && MeasuredAngle > -90) {
        while (ReturnMeasuredAngle() < TargetAngle - 1 && opModeIsActive()) {
          frontRightMotorAsDcMotor.setPower(-0.1);
          backRightMotorAsDcMotor.setPower(-0.1);
          frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
          frontLeftMotor.setPower(0.1);
          backLeftMotor.setPower(0.1);
        }
      }
    } else {
      // TargetAngle <0
      if (ReturnMeasuredAngle() > TargetAngle + 1 && MeasuredAngle < 90) {
        // If Turned a little right
        while (ReturnMeasuredAngle() > TargetAngle + 1 && opModeIsActive()) {
          frontRightMotorAsDcMotor.setPower(0.1);
          backRightMotorAsDcMotor.setPower(0.1);
          frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
          frontLeftMotor.setPower(-0.1);
          backLeftMotor.setPower(-0.1);
        }
      } else if (ReturnMeasuredAngle() < TargetAngle - 1 || MeasuredAngle > 90) {
        // If Turned a little left
        while ((ReturnMeasuredAngle() < TargetAngle - 1 || MeasuredAngle > 90) && opModeIsActive()) {
          frontRightMotorAsDcMotor.setPower(-0.1);
          backRightMotorAsDcMotor.setPower(-0.1);
          frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
          frontLeftMotor.setPower(0.1);
          backLeftMotor.setPower(0.1);
        }
      }
    }
    frontRightMotorAsDcMotor.setPower(0);
    backRightMotorAsDcMotor.setPower(0);
    frontLeftMotor.setPower(0);
    backLeftMotor.setPower(0);
    frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  }

  /**
   * Describe this function...
   */
  private void TurnW_ramp(double Angle, int Power) {
    double scaleFactor;
    double normalMeasuredAngle;

    OldTargetAngle = TargetAngle;
    currentPower = 0.05;
    scaleFactor = 250 * (Power / Angle);
    if (OldTargetAngle + Angle > 180) {
      TargetAngle = (OldTargetAngle + Angle) - 360;
    } else if (OldTargetAngle + Angle < -180) {
      TargetAngle = OldTargetAngle + Angle + 360;
    } else {
      TargetAngle = OldTargetAngle + Angle;
    }
    FLStartingEncoderPosition = frontLeftMotor.getCurrentPosition();
    FRStartingEncoderPosition = frontRightMotorAsDcMotor.getCurrentPosition();
    frontLeftMotor.setTargetPosition((int) (FLStartingEncoderPosition + 10.6 * Angle));
    frontRightMotorAsDcMotor.setTargetPosition((int) (FRStartingEncoderPosition - 10.6 * Angle));
    if (Angle == 0) {
    } else if (Angle > 0) {
      // Turning Right
      while (ReturnMeasuredAngle() < TargetAngle && opModeIsActive()) {
        telemetry.addData("Angle", ReturnMeasuredAngle());
        telemetry.addData("Desired Angle", TargetAngle);
        telemetry.addData("current power", currentPower);
        angleError();
        telemetry.addData("error", errorAngle);
        telemetry.update();
        rampUpRampDownAng(TargetAngle, Power, 0.03, 30);
        backLeftMotor.setPower(currentPower);
        backRightMotorAsDcMotor.setPower(-currentPower);
        frontLeftMotor.setPower(currentPower);
        frontRightMotorAsDcMotor.setPower(-currentPower);
      }
    } else {
      // Turning Left
      while (ReturnMeasuredAngle() > TargetAngle && opModeIsActive()) {
        telemetry.addData("Angle", ReturnMeasuredAngle());
        telemetry.addData("Desired Angle", TargetAngle);
        telemetry.addData("current power", currentPower);
        angleError();
        telemetry.addData("error", errorAngle);
        telemetry.update();
        rampUpRampDownAng(TargetAngle, Power, 0.03, 30);
        backLeftMotor.setPower(-currentPower);
        backRightMotorAsDcMotor.setPower(currentPower);
        frontLeftMotor.setPower(-currentPower);
        frontRightMotorAsDcMotor.setPower(currentPower);
      }
    }
    backLeftMotor.setPower(0);
    backRightMotorAsDcMotor.setPower(0);
    frontLeftMotor.setPower(0);
    frontRightMotorAsDcMotor.setPower(0);
    telemetry.addData("Angle", ReturnMeasuredAngle());
    angleError();
    telemetry.addData("error", errorAngle);
    telemetry.update();
    if (TargetAngle > 0) {
      normalMeasuredAngle = ReturnMeasuredAngle();
      if (ReturnMeasuredAngle() < 0) {
        normalMeasuredAngle = ReturnMeasuredAngle() + 360;
      }
      // TargetAngle >0
      if (normalMeasuredAngle > TargetAngle + 1) {
        while (normalMeasuredAngle > TargetAngle + 1 && opModeIsActive()) {
          telemetry.addData("TargetAngle", TargetAngle);
          telemetry.addData("ReturMeasuredAngle", ReturnMeasuredAngle());
          telemetry.addData("Normalangle", normalMeasuredAngle);
          telemetry.update();
          frontRightMotorAsDcMotor.setPower(0.1);
          backRightMotorAsDcMotor.setPower(0.1);
          frontLeftMotor.setPower(-0.1);
          backLeftMotor.setPower(-0.1);
          normalMeasuredAngle = ReturnMeasuredAngle();
          if (ReturnMeasuredAngle() < 0) {
            normalMeasuredAngle = ReturnMeasuredAngle() + 360;
          }
        }
      } else if (normalMeasuredAngle < TargetAngle - 1) {
        while (normalMeasuredAngle < TargetAngle - 1 && opModeIsActive()) {
          telemetry.addData("TargetAngle", TargetAngle);
          telemetry.addData("ReturMeasuredAngle", ReturnMeasuredAngle());
          telemetry.addData("Normalangle", normalMeasuredAngle);
          telemetry.update();
          frontRightMotorAsDcMotor.setPower(-0.1);
          backRightMotorAsDcMotor.setPower(-0.1);
          frontLeftMotor.setPower(0.1);
          backLeftMotor.setPower(0.1);
          normalMeasuredAngle = ReturnMeasuredAngle();
          if (ReturnMeasuredAngle() < 0) {
            normalMeasuredAngle = ReturnMeasuredAngle() + 360;
          }
        }
      }
    } else {
      // TargetAngle <0
      if (normalMeasuredAngle > TargetAngle + 1) {
        // If Turned a little right
        while (normalMeasuredAngle > TargetAngle + 1 && opModeIsActive()) {
          telemetry.addData("TargetAngle", TargetAngle);
          telemetry.addData("ReturMeasuredAngle", ReturnMeasuredAngle());
          telemetry.addData("Normalangle", normalMeasuredAngle);
          telemetry.update();
          frontRightMotorAsDcMotor.setPower(0.1);
          backRightMotorAsDcMotor.setPower(0.1);
          frontLeftMotor.setPower(-0.1);
          backLeftMotor.setPower(-0.1);
          normalMeasuredAngle = ReturnMeasuredAngle();
          if (ReturnMeasuredAngle() > 0) {
            normalMeasuredAngle = ReturnMeasuredAngle() - 360;
          }
        }
      } else if (normalMeasuredAngle < TargetAngle - 1) {
        // If Turned a little left
        while (normalMeasuredAngle < TargetAngle - 1 && opModeIsActive()) {
          telemetry.addData("TargetAngle", TargetAngle);
          telemetry.addData("ReturMeasuredAngle", ReturnMeasuredAngle());
          telemetry.addData("Normalangle", normalMeasuredAngle);
          telemetry.update();
          frontRightMotorAsDcMotor.setPower(-0.1);
          backRightMotorAsDcMotor.setPower(-0.1);
          frontLeftMotor.setPower(0.1);
          backLeftMotor.setPower(0.1);
          normalMeasuredAngle = ReturnMeasuredAngle();
          if (ReturnMeasuredAngle() > 0) {
            normalMeasuredAngle = ReturnMeasuredAngle() - 360;
          }
        }
      }
    }
    backLeftMotor.setPower(0);
    backRightMotorAsDcMotor.setPower(0);
    frontLeftMotor.setPower(0);
    frontRightMotorAsDcMotor.setPower(0);
    sleep(200);
    telemetry.addData("TargetAngle", TargetAngle);
    telemetry.addData("ReturMeasuredAngle", ReturnMeasuredAngle());
    telemetry.update();
  }

  /**
   * Describe this function...
   */
  private void StrafeRight(double Distance, double Power) {
    FLStartingEncoderPosition = frontLeftMotor.getCurrentPosition();
    FRStartingEncoderPosition = frontRightMotorAsDcMotor.getCurrentPosition();
    frontLeftMotor.setTargetPosition((int) (FLStartingEncoderPosition + 49 * Distance));
    frontRightMotorAsDcMotor.setTargetPosition((int) (FRStartingEncoderPosition - 49 * Distance));
    if (Distance > 0) {
      // Running to the Right
      while (frontLeftMotor.getCurrentPosition() < frontLeftMotor.getTargetPosition()) {
        backLeftMotor.setPower(-Power);
        backRightMotorAsDcMotor.setPower(Power);
        frontLeftMotor.setPower(Power);
        frontRightMotorAsDcMotor.setPower(-Power);
      }
    } else {
      // Running to the Left
      while (frontLeftMotor.getCurrentPosition() > frontLeftMotor.getTargetPosition()) {
        backLeftMotor.setPower(Power);
        backRightMotorAsDcMotor.setPower(-Power);
        frontLeftMotor.setPower(-Power);
        frontRightMotorAsDcMotor.setPower(Power);
      }
    }
    backLeftMotor.setPower(0);
    backRightMotorAsDcMotor.setPower(0);
    frontLeftMotor.setPower(0);
    frontRightMotorAsDcMotor.setPower(0);
  }

  /**
   * Describe this function...
   */
  private void runIntake(String runIntakeInOut) {
    if (runIntakeInOut.equals("In")) {
      intakeMotorAsDcMotor.setPower(1);
      intakeMotorAsDcMotor.setTargetPosition(2000);
    } else if (runIntakeInOut.equals("Out")) {
      intakeMotorAsDcMotor.setPower(-1);
      intakeMotorAsDcMotor.setTargetPosition(-2000);
    } else if (runIntakeInOut.equals("Rest")) {
      intakeMotorAsDcMotor.setPower(0);
    }
  }

  /**
   * Describe this function...
   */
  private double ReturnMeasuredAngle() {
    MeasuredAngle = -imuAsBNO055IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    return MeasuredAngle;
  }

  /**
   * Describe this function...
   */
  private void releasePixel(boolean central, boolean secondary, boolean center) {
    // TODO: Enter the type for variable named my_0123456
    UNKNOWN_TYPE my_0123456;

    if (center == true) {
      toggleServoAsServo.setPosition(scoringCenter);
    } else if (central == true && secondary == true) {
      if (my_0123456 == (desiredWristAngle2 || desiredWristAngle5)) {
        toggleServoAsServo.setPosition(scoringCentral);
        sleep(500);
        toggleServoAsServo.setPosition(scoringSecondary);
        sleep(500);
        toggleServoAsServo.setPosition(scoringCenter);
      } else if (my_0123456 == (desiredWristAngle3 || desiredWristAngle4)) {
        toggleServoAsServo.setPosition(scoringCentral);
        sleep(500);
        toggleServoAsServo.setPosition(scoringSecondary);
        sleep(500);
        toggleServoAsServo.setPosition(scoringCenter);
      } else {
        toggleServoAsServo.setPosition(scoringCentral);
        sleep(500);
        toggleServoAsServo.setPosition(scoringSecondary);
        sleep(500);
        toggleServoAsServo.setPosition(scoringCenter);
      }
      central = false;
      secondary = false;
    } else if (central == true) {
      toggleServoAsServo.setPosition(scoringCentral);
      sleep(500);
      toggleServoAsServo.setPosition(scoringCenter);
      central = false;
    } else if (secondary == true) {
      toggleServoAsServo.setPosition(scoringSecondary);
      sleep(500);
      toggleServoAsServo.setPosition(scoringCenter);
      secondary = false;
    }
  }

  /**
   * Describe this function...
   */
  private void compressing() {
    setWristAngle(desiredWristAngle0);
    sleep(500);
    pivotAngle(0);
    sleep(1000);
    callRowHeight(0, 0.5);
  }
}