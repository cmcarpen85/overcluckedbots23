package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name = "STATESTeleOp2023 (Blocks to Java)")
public class STATESTeleOp2023 extends LinearOpMode {

  private BNO055IMU imuAsBNO055IMU;
  private Servo climberReleaseServo;
  private Servo droneReleaseServo;
  private Servo wristServoAsServo;
  private Servo toggleServoAsServo;
  private Servo outtakePivotServoAsServo;
  private DcMotor climberWinchMotor;
  private DcMotor backLeftMotor;
  private DcMotor backRightMotorAsDcMotor;
  private DcMotor frontLeftMotor;
  private DcMotor frontRightMotorAsDcMotor;
  private DcMotor leftLiftMotor;
  private DcMotor rightLiftMotorAsDcMotor;
  private DcMotor intakeMotorAsDcMotor;
  private Servo intakePivotServoAsServo;
  private CRServo passoffServoAsCRServo;
  private CRServo passOffServo2;

  int rowNumber;
  boolean SlightlyLoweredTrue_False;
  int scoringStep;
  double PercentX;
  double ReadyToGrabPixelsStep;
  int piviotStep;
  int wristServo;
  String RobotMode;
  double ReadyToGrabPixelsTimer;
  int EnteringTheManipulatorStep;
  double PercentY;
  boolean slowModeLatch;
  int MissedPickStep;
  int releasePixelsStep;
  boolean Snap2ZeroLatch;
  double desiredWristAngle0;
  double desiredWristAngle1;
  double desiredWristAngle2;
  double desiredWristAngle3;
  double desiredWristAngle4;
  double desiredWristAngle5;
  double desiredWristAngle6;
  boolean latchWristPosition1;
  double scoringCenter;
  double scoringCentral;
  double scoringSecondary;
  double passoffServoTimer;
  boolean passoffLatch;
  double RPMA;
  double LeftJoystickAngle;
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
  int outtakeReady4PickupHeight;
  boolean ZeroLiftLatch;
  double MissedPickTimer;
  boolean latchWristPosition2;
  double LeftJoystickPower;
  int SlightlyLoweredCommonDifference;
  boolean Snap2ZeroLatch2;
  double Snap2ZeroPower;
  boolean LatchReleasePixels2;
  boolean LatchReleasePixels3;

  /**
   * Intake
   */
  private void initialize() {
    int INTAKE_TICKS_PER_REVOLUTION;
    int TargetAngle;
    double GyroAdjustment;
    int liftRightPosition;
    int liftLeftPosition;
    int baseHeight;

    imuAsBNO055IMU.initialize(new BNO055IMU.Parameters());
    climberReleaseServo.setPosition(0.505);
    droneReleaseServo.setPosition(0.5);
    // Verify all initializations before "TargetAngle=0"
    ZeroLiftLatch = false;
    SlightlyLoweredCommonDifference = 20;
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
    outtakeReady4PickupHeight = 72;
    Snap2ZeroLatch = true;
    latchWristPosition2 = false;
    latchWristPosition1 = false;
    LatchReleasePixels2 = false;
    LatchReleasePixels3 = false;
    slowModeLatch = false;
    Snap2ZeroLatch2 = false;
    Snap2ZeroLatch2 = false;
    Snap2ZeroPower = 0.25;
    // Disabled because may not go to that height
    // The stuff for wrist function
    wristServoAsServo.setPosition(0.513);
    // The stuff for the toggle function
    scoringCenter = 0.497;
    scoringSecondary = 0.5125;
    scoringCentral = 0.475;
    toggleServoAsServo.setPosition(scoringCenter);
    desiredWristAngle0 = 0.5115;
    desiredWristAngle3 = 0.606;
    desiredWristAngle2 = 0.5775;
    desiredWristAngle1 = 0.5465;
    desiredWristAngle4 = 0.49045;
    desiredWristAngle5 = 0.4442;
    desiredWristAngle6 = 0.4071;
    FourbarPosition("Rest");
    // Sequence steps
    ReadyToGrabPixelsStep = 0;
    // The stuff for the pivot function
    outtakePivotServoAsServo.setPosition(0.4872);
    climberWinchMotor.setTargetPosition(0);
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
    climberWinchMotor.setDirection(DcMotor.Direction.FORWARD);
    climberWinchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
    climberWinchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    RobotMode = "RobotJustStarted";
  }

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    int DriveWithDetectStep;

    imuAsBNO055IMU = hardwareMap.get(BNO055IMU.class, "imuAsBNO055IMU");
    climberReleaseServo = hardwareMap.get(Servo.class, "climberReleaseServo");
    droneReleaseServo = hardwareMap.get(Servo.class, "droneReleaseServo");
    wristServoAsServo = hardwareMap.get(Servo.class, "wristServoAsServo");
    toggleServoAsServo = hardwareMap.get(Servo.class, "toggleServoAsServo");
    outtakePivotServoAsServo = hardwareMap.get(Servo.class, "outtakePivotServoAsServo");
    climberWinchMotor = hardwareMap.get(DcMotor.class, "climberWinchMotor");
    backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
    backRightMotorAsDcMotor = hardwareMap.get(DcMotor.class, "backRightMotorAsDcMotor");
    frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
    frontRightMotorAsDcMotor = hardwareMap.get(DcMotor.class, "frontRightMotorAsDcMotor");
    leftLiftMotor = hardwareMap.get(DcMotor.class, "leftLiftMotor");
    rightLiftMotorAsDcMotor = hardwareMap.get(DcMotor.class, "rightLiftMotorAsDcMotor");
    intakeMotorAsDcMotor = hardwareMap.get(DcMotor.class, "intakeMotorAsDcMotor");
    intakePivotServoAsServo = hardwareMap.get(Servo.class, "intakePivotServoAsServo");
    passoffServoAsCRServo = hardwareMap.get(CRServo.class, "passoffServoAsCRServo");
    passOffServo2 = hardwareMap.get(CRServo.class, "passOffServo2");

    initialize();
    waitForStart();
    if (opModeIsActive()) {
      while (opModeIsActive()) {
        CalculateJoystickAngle();
        CalculateRPMA();
        CalculateMovementComponents();
        Drive();
        SetMode();
        telemetry.addData("Lift right motor", leftLiftMotor.getCurrentPosition());
        telemetry.addData("Lift left motor", rightLiftMotorAsDcMotor.getCurrentPosition());
        telemetry.addData("MeasuredAngle", ReturnMeasuredAngle());
        telemetry.addData("JoystickAngle", LeftJoystickAngle);
        telemetry.addData("JoystickPower", LeftJoystickPower);
        telemetry.update();
        if (RobotMode.equals("drivingW/outpixles")) {
          drivingW_outPixles();
        } else if (RobotMode.equals("ReadyToGrabPixels")) {
          ReadyToGrabPixels();
        } else if (RobotMode.equals("Scoring")) {
          Scoring();
          DriveWithDetectStep = 0;
        } else if (RobotMode.equals("TakeOffPositive")) {
        } else if (RobotMode.equals("EnteringTheManipulator")) {
          EnteringTheManipulator();
        } else if (RobotMode.equals("ExitingTheManipulator")) {
          ExitingTheManipulator();
        } else if (RobotMode.equals("MissedPick")) {
          MissedPick();
        } else if (RobotMode.equals("TBD")) {
        }
        telemetry.addData("Mode", RobotMode);
        telemetry.addData("ReadyToGrabPixelsStep", ReadyToGrabPixelsStep);
        telemetry.addData("back left motor", backLeftMotor.getCurrentPosition());
        telemetry.addData("Right lift encoder", rightLiftMotorAsDcMotor.getCurrentPosition());
        telemetry.addData("Row Hieght", rowNumber);
        telemetry.addData("Wrist angle", wristServo);
        telemetry.addData("Missed pick step", MissedPickStep);
        telemetry.update();
      }
    }
  }

  /**
   * pick
   */
  private void MissedPick() {
    if (MissedPickStep == 0) {
      releasePixel(false, false, true);
      setWristAngle(0);
      MissedPickStep = 1;
    } else if (MissedPickStep == 1) {
      if (getRuntime() > MissedPickTimer + 0.5) {
        pivotAngle(0);
        MissedPickStep = 2;
      }
    } else if (MissedPickStep == 2) {
      if (getRuntime() > MissedPickTimer + 1) {
        callRowHeight(0.5, 0.25, false);
        MissedPickStep = 3;
      }
    } else if (MissedPickStep == 3) {
      ReadyToGrabPixelsStep = 5;
      RobotMode = "ReadyToGrabPixels";
    }
  }

  /**
   * Describe this function...
   */
  private void Scoring() {
    double scoringTimer;
    boolean latchSlidePosition1;
    boolean latchSlidePosition2;
    boolean latchReleasePixels;

    if (scoringStep == 0) {
      scoringTimer = getRuntime();
      wristServo = 0;
      SlightlyLoweredTrue_False = false;
      scoringStep = 1;
    } else if (scoringStep == 1) {
      piviotStep = 0;
      callRowHeight(1, 0.75, false);
      setWristAngle(wristServo);
      scoringStep = 2;
    } else if (scoringStep == 2) {
      pivotAngle(1);
      if (getRuntime() > scoringTimer + 0.6) {
        scoringStep = 3;
      }
    } else if (scoringStep == 3) {
      latchSlidePosition1 = false;
      latchSlidePosition2 = false;
      latchWristPosition1 = false;
      latchWristPosition2 = false;
      if (getRuntime() > scoringTimer + 0.65) {
        rowNumber = 2;
        scoringStep = 4;
        wristServo = 1;
        releasePixelsStep = 0;
      }
    } else if (scoringStep == 4) {
      if (getRuntime() > scoringTimer + 0.7) {
        callRowHeight(rowNumber, 0.75, SlightlyLoweredTrue_False);
        setWristAngle(wristServo);
      }
      if (gamepad2.a == true) {
        rowNumber = 1;
      } else if (gamepad2.x == true) {
        rowNumber = 3;
      } else if (gamepad2.y == true) {
        rowNumber = 5;
      } else if (gamepad2.b == true) {
        rowNumber = 7;
      }
      if (gamepad2.dpad_up == true && latchSlidePosition1 == false && rowNumber < 7) {
        rowNumber = rowNumber + 1;
        latchSlidePosition1 = true;
      } else if (gamepad2.dpad_down == true && latchSlidePosition2 == false && rowNumber > 1) {
        rowNumber = rowNumber - 1;
        latchSlidePosition2 = true;
      } else if (gamepad2.dpad_up == false && latchSlidePosition1 == true) {
        latchSlidePosition1 = false;
      } else if (gamepad2.dpad_down == false && latchSlidePosition2 == true) {
        latchSlidePosition2 = false;
      }
      if (gamepad2.right_bumper == true && latchWristPosition1 == false) {
        wristServo = wristServo + 1;
        latchWristPosition1 = true;
      } else if (gamepad2.right_bumper == false && latchWristPosition1 == true) {
        latchWristPosition1 = false;
      }
      if (gamepad2.left_bumper == true && latchWristPosition2 == false) {
        wristServo = wristServo - 1;
        latchWristPosition2 = true;
      } else if (gamepad2.left_bumper == false && latchWristPosition2 == true) {
        latchWristPosition2 = false;
      }
      if (gamepad2.left_stick_button == true) {
        MissedPickStep = 0;
        MissedPickTimer = getRuntime();
        RobotMode = "MissedPick";
      }
      if (gamepad1.left_trigger > 0.4) {
        releasePixel(true, true, false);
        latchReleasePixels = true;
      } else if (latchReleasePixels == true && gamepad1.left_trigger < 0.4) {
        latchReleasePixels = false;
        releasePixel(false, false, true);
        scoringStep = 5;
      }
      if (gamepad1.left_bumper == true) {
        releasePixel(false, true, false);
        LatchReleasePixels2 = true;
      } else if (LatchReleasePixels2 == true && gamepad1.left_bumper == false) {
        LatchReleasePixels2 = false;
        releasePixel(false, false, true);
      }
      if (gamepad1.right_bumper == true) {
        releasePixel(true, false, false);
        LatchReleasePixels3 = true;
      } else if (LatchReleasePixels3 == true && gamepad1.right_bumper == false) {
        LatchReleasePixels3 = false;
        releasePixel(false, false, true);
      }
    } else if (scoringStep == 5) {
      wristServo = 0;
      setWristAngle(wristServo);
      scoringTimer = getRuntime();
      scoringStep = 6;
      piviotStep = 0;
    } else if (scoringStep == 6) {
      pivotAngle(0);
      if (getRuntime() > scoringTimer + 0.35) {
        scoringTimer = getRuntime();
        scoringStep = 7;
      }
    } else if (scoringStep == 7) {
      if (getRuntime() > scoringTimer + 0.3) {
        callRowHeight(0, 0.65, false);
        RobotMode = "drivingW/outpixles";
      }
    }
  }

  /**
   * Describe this function...
   */
  private void ExitingTheManipulator() {
    scoringStep = 0;
    RobotMode = "Scoring";
  }

  /**
   * Describe this function...
   */
  private void releasePixel(boolean central, boolean secondary, boolean center) {
    double releasePixelsTimer;

    if (releasePixelsStep == 0) {
      releasePixelsTimer = getRuntime();
      releasePixelsStep = 1;
    } else if (releasePixelsStep == 1) {
      if (center == true) {
        toggleServoAsServo.setPosition(scoringCenter);
      } else if (central == true && secondary == true) {
        if (wristServo == 1 || wristServo == -1) {
          toggleServoAsServo.setPosition(scoringSecondary);
          if (getRuntime() > releasePixelsTimer + 0.5) {
            toggleServoAsServo.setPosition(scoringCentral);
          }
          if (getRuntime() > releasePixelsTimer + 1) {
            toggleServoAsServo.setPosition(scoringCenter);
            releasePixelsStep = 0;
          }
        } else if (wristServo == -2 || wristServo == 2 || wristServo == -3 || wristServo == 3) {
          toggleServoAsServo.setPosition(scoringCentral);
          if (getRuntime() > releasePixelsTimer + 0.5) {
            toggleServoAsServo.setPosition(scoringSecondary);
          }
          if (getRuntime() > releasePixelsTimer + 1) {
            toggleServoAsServo.setPosition(scoringCenter);
            releasePixelsStep = 0;
          }
        } else {
          toggleServoAsServo.setPosition(scoringSecondary);
          if (getRuntime() > releasePixelsTimer + 0.5) {
            toggleServoAsServo.setPosition(scoringCentral);
          }
          if (getRuntime() > releasePixelsTimer + 1) {
            toggleServoAsServo.setPosition(scoringCenter);
            releasePixelsStep = 0;
          }
        }
        central = false;
        secondary = false;
      } else if (central == true) {
        toggleServoAsServo.setPosition(scoringCentral);
        if (getRuntime() > releasePixelsTimer + 1) {
          toggleServoAsServo.setPosition(scoringCenter);
          releasePixelsStep = 0;
        }
        central = false;
        releasePixelsStep = 0;
      } else if (secondary == true) {
        toggleServoAsServo.setPosition(scoringSecondary);
        if (getRuntime() > releasePixelsTimer + 1) {
          toggleServoAsServo.setPosition(scoringCenter);
        }
        secondary = false;
        releasePixelsStep = 0;
      }
    }
  }

  /**
   * Describe this function...
   */
  private void Drive() {
    if (slowModeLatch == false && Snap2ZeroLatch == true) {
      frontLeftMotor.setPower(1 * LeftJoystickPower * (PercentY + PercentX) + 1 * gamepad1.right_stick_x);
      frontRightMotorAsDcMotor.setPower(1 * LeftJoystickPower * (PercentY - PercentX) - 1 * gamepad1.right_stick_x);
      backLeftMotor.setPower(1 * LeftJoystickPower * (PercentY - PercentX) + 1 * gamepad1.right_stick_x);
      backRightMotorAsDcMotor.setPower(1 * LeftJoystickPower * (PercentY + PercentX) - 1 * gamepad1.right_stick_x);
    } else if ((slowModeLatch == true) == (Snap2ZeroLatch == true)) {
      frontLeftMotor.setPower(0.35 * LeftJoystickPower * (PercentY + PercentX) + 0.35 * gamepad1.right_stick_x);
      frontRightMotorAsDcMotor.setPower(0.35 * LeftJoystickPower * (PercentY - PercentX) - 0.35 * gamepad1.right_stick_x);
      backLeftMotor.setPower(0.35 * LeftJoystickPower * (PercentY - PercentX) + 0.35 * gamepad1.right_stick_x);
      backRightMotorAsDcMotor.setPower(0.35 * LeftJoystickPower * (PercentY + PercentX) - 0.35 * gamepad1.right_stick_x);
    } else if (Snap2ZeroLatch == false) {
      telemetry.addData("Snap2ZeroLatch", Snap2ZeroLatch);
      Snap2Zero();
    }
  }

  /**
   * Describe this function...
   */
  private void ReadyToGrabPixels() {
    if (ReadyToGrabPixelsStep == 0) {
      ReadyToGrabPixelsTimer = getRuntime();
      pivotAngle(0);
      callRowHeight(0.5, 0.9, false);
      FourbarPosition("Intake");
      ReadyToGrabPixelsStep = 1;
    } else if (ReadyToGrabPixelsStep == 1) {
      if (getRuntime() > ReadyToGrabPixelsTimer + 0.35) {
        if (gamepad2.right_trigger > 0.4) {
          if (gamepad2.dpad_up) {
            FourbarPosition("2");
          } else if (gamepad2.dpad_down) {
            FourbarPosition("1");
          }
          runIntake("In");
          passoffConveyor("In");
        } else if (gamepad2.right_trigger < 0.4) {
          ReadyToGrabPixelsTimer = getRuntime();
          ReadyToGrabPixelsStep = 2;
        }
      }
    } else if (ReadyToGrabPixelsStep == 2) {
      runIntake("Out");
      if (getRuntime() > ReadyToGrabPixelsTimer + 0.5) {
        ReadyToGrabPixelsStep = 3;
      }
    } else if (ReadyToGrabPixelsStep == 3) {
      ReadyToGrabPixelsTimer = getRuntime();
      runIntake("Stop");
      passoffServoTimer = getRuntime();
      passoffLatch = false;
      if (gamepad2.left_trigger >= 0.4) {
        ReadyToGrabPixelsStep = 4;
      } else {
        ReadyToGrabPixelsStep = 3.5;
      }
    } else if (ReadyToGrabPixelsStep == 3.5) {
      if (getRuntime() > ReadyToGrabPixelsTimer + 0) {
        FourbarPosition("Rest");
        runIntake("Stop");
        passoffConveyor("Rest");
        EnteringTheManipulatorStep = 0;
        RobotMode = "EnteringTheManipulator";
      }
    } else if (ReadyToGrabPixelsStep == 4) {
      if (getRuntime() > ReadyToGrabPixelsTimer + 0.07) {
        passoffConveyor("Rest");
        FourbarPosition("Rest");
        runIntake("Stop");
        ReadyToGrabPixelsStep = 5;
      }
    } else if (ReadyToGrabPixelsStep == 5) {
      if (gamepad2.a == true) {
        ReadyToGrabPixelsTimer = getRuntime();
        passoffLatch = false;
        ReadyToGrabPixelsStep = 6;
      } else if (gamepad2.right_bumper == true) {
        EnteringTheManipulatorStep = 0;
        RobotMode = "EnteringTheManipulator";
      } else if (gamepad2.b == true) {
        FourbarPosition("Intake");
        runIntake("Out");
        passoffConveyor("Out");
      } else {
        ReadyToGrabPixelsStep = 4;
      }
    } else if (ReadyToGrabPixelsStep == 6) {
      passoffConveyor("In");
      if (getRuntime() > ReadyToGrabPixelsTimer + 1) {
        ReadyToGrabPixelsStep = 4;
      }
    }
  }

  /**
   * Describe this function...
   */
  private void RobotJustStarted() {
    double RobotJustStartedTimer;
    int RobotJustStartedStep;

    if (RobotJustStartedStep == 0) {
      RobotJustStartedTimer = getRuntime();
      callRowHeight(1, 0.5, false);
      RobotJustStartedStep = 1;
    }
    if (RobotJustStartedStep == 1) {
      setWristAngle((int) desiredWristAngle0);
      RobotJustStartedStep = 2;
    }
    if (RobotJustStartedStep == 2) {
      if (getRuntime() > RobotJustStartedTimer + 0.5) {
        pivotAngle(0);
        RobotJustStartedStep = 3;
      }
    }
    if (RobotJustStartedStep == 3) {
      if (getRuntime() > RobotJustStartedTimer + 1) {
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLiftMotorAsDcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLiftMotor.setPower(-0.25);
        rightLiftMotorAsDcMotor.setPower(-0.25);
        RobotJustStartedStep = 4;
      }
    }
    if (RobotJustStartedStep == 4) {
      if (getRuntime() > RobotJustStartedTimer + 1.5) {
        leftLiftMotor.setPower(0);
        rightLiftMotorAsDcMotor.setPower(0);
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLiftMotorAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLiftMotorAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RobotJustStartedStep = 5;
      }
    }
    if (RobotJustStartedStep == 5) {
      RobotMode = "drivingW/outPixles";
    }
  }

  /**
   * Describe this function...
   */
  private void pivotAngle(int pivotOrientation) {
    // TODO: Enter the type for variable named my_0123456
    UNKNOWN_TYPE my_0123456;
    double pivotTimer;

    if (piviotStep == 0) {
      pivotTimer = getRuntime();
      piviotStep = 1;
    } else if (piviotStep == 1) {
      if (pivotOrientation == 0) {
        if (my_0123456 == 0) {
          outtakePivotServoAsServo.setPosition(0.4872);
        } else if (my_0123456 != 0) {
          wristServoAsServo.setPosition(desiredWristAngle0);
          if (getRuntime() > pivotTimer + 0.25) {
            outtakePivotServoAsServo.setPosition(0.4872);
          }
        }
      } else if (pivotOrientation == 1) {
        if (my_0123456 == 0) {
          outtakePivotServoAsServo.setPosition(0.3948);
        } else if (my_0123456 != 0) {
          wristServoAsServo.setPosition(desiredWristAngle0);
          if (getRuntime() > pivotTimer + 0.25) {
            outtakePivotServoAsServo.setPosition(0.395);
          }
        }
      }
    }
  }

  /**
   * Passoff between chomper and manipulator
   */
  private void EnteringTheManipulator() {
    double EnteringTheManipulatorTimer;

    if (EnteringTheManipulatorStep == 0) {
      EnteringTheManipulatorTimer = getRuntime();
      EnteringTheManipulatorStep = 1;
    } else if (EnteringTheManipulatorStep == 1) {
      callRowHeight(0, 0.15, false);
      if (getRuntime() > EnteringTheManipulatorTimer + 0.75) {
        EnteringTheManipulatorStep = 2;
      }
    } else if (EnteringTheManipulatorStep == 2) {
      if (gamepad2.a == true) {
        ReadyToGrabPixelsStep = 5;
        callRowHeight(0.5, 0.25, false);
        RobotMode = "ReadyToGrabPixels";
      } else if (gamepad2.right_bumper == true) {
        RobotMode = "ExitingTheManipulator";
        EnteringTheManipulatorStep = 0;
      } else if (gamepad2.right_trigger > 0.4) {
        RobotMode = "ReadyToGrabPixels";
        ReadyToGrabPixelsStep = 0;
      }
    } else if (EnteringTheManipulatorStep == 3) {
      EnteringTheManipulatorStep = 3;
    } else if (EnteringTheManipulatorStep == 4) {
      if (getRuntime() > EnteringTheManipulatorTimer + 0.5) {
      }
    }
  }

  /**
   * We need to change the heights-these are the ones from last year
   */
  private void FourbarPosition(String IntakeRest12) {
    // Remember to change the servo after configuring robot!
    if (IntakeRest12.equals("Intake")) {
      intakePivotServoAsServo.setPosition(0.521);
    } else if (IntakeRest12.equals("Rest")) {
      intakePivotServoAsServo.setPosition(0.445);
    } else if (IntakeRest12.equals("1")) {
      intakePivotServoAsServo.setPosition(0.505);
    } else if (IntakeRest12.equals("2")) {
      intakePivotServoAsServo.setPosition(0.4991);
    }
  }

  /**
   * Describe this function...
   */
  private void CalculateMovementComponents() {
    PercentX = Math.sin(RPMA / 180 * Math.PI) / (Math.abs(Math.sin(RPMA / 180 * Math.PI)) + Math.abs(Math.cos(RPMA / 180 * Math.PI)));
    PercentY = Math.cos(RPMA / 180 * Math.PI) / (Math.abs(Math.sin(RPMA / 180 * Math.PI)) + Math.abs(Math.cos(RPMA / 180 * Math.PI)));
  }

  /**
   * Describe this function...
   */
  private void CalculateRPMA() {
    double RobotAngle;

    // Copy RPMA = Robot's perspective movement angle
    RobotAngle = ReturnMeasuredAngle();
    if (LeftJoystickAngle - RobotAngle > 180) {
      RPMA = (LeftJoystickAngle - RobotAngle) - 360;
    } else if (LeftJoystickAngle - RobotAngle < -180) {
      RPMA = (LeftJoystickAngle - RobotAngle) + 360;
    } else {
      RPMA = LeftJoystickAngle - RobotAngle;
    }
  }

  /**
   * Describe this function...
   */
  private double ReturnMeasuredAngle() {
    float MeasuredAngle;

    MeasuredAngle = -imuAsBNO055IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    return MeasuredAngle;
  }

  /**
   * Describe this function...
   */
  private void Slow_mode(int Speedtoggle) {
    if (Speedtoggle == 1) {
      slowModeLatch = true;
    }
    if (Speedtoggle == 0) {
      slowModeLatch = false;
    }
  }

  /**
   * Describe this function...
   */
  private void CalculateJoystickAngle() {
    float JoystickX;
    float JoystickY;

    JoystickX = gamepad1.left_stick_x;
    JoystickY = -gamepad1.left_stick_y;
    if (Math.abs(JoystickY) < 0.02) {
      if (Math.abs(JoystickX) < 0.02) {
        LeftJoystickPower = 0;
        LeftJoystickAngle = 0;
      } else if (JoystickX > 0) {
        LeftJoystickAngle = 90;
        LeftJoystickPower = JoystickX;
      } else {
        LeftJoystickAngle = -90;
        LeftJoystickPower = -JoystickX;
      }
    } else if (Math.abs(JoystickX) < 0.02) {
      if (JoystickY > 0) {
        LeftJoystickPower = JoystickY;
        LeftJoystickAngle = 0;
      } else {
        LeftJoystickAngle = 180;
        LeftJoystickPower = -JoystickY;
      }
    } else {
      LeftJoystickAngle = Math.atan2(JoystickX, JoystickY) / Math.PI * 180;
      LeftJoystickPower = Math.sqrt(JoystickX * JoystickX + JoystickY * JoystickY);
    }
  }

  /**
   * Describe this function...
   */
  private void callRowHeight(double rowNumber, double VSPower, boolean SlightlyLoweredTrue_False) {
    int row11;
    int row12;

    if (SlightlyLoweredTrue_False == true) {
      if (rowNumber == 1) {
        rightLiftMotorAsDcMotor.setTargetPosition(row1);
        leftLiftMotor.setTargetPosition(row1);
        rightLiftMotorAsDcMotor.setPower(VSPower);
        leftLiftMotor.setPower(VSPower);
      } else if (rowNumber == 2) {
        rightLiftMotorAsDcMotor.setTargetPosition(row2 - SlightlyLoweredCommonDifference);
        leftLiftMotor.setTargetPosition(row2 - SlightlyLoweredCommonDifference);
        rightLiftMotorAsDcMotor.setPower(VSPower);
        leftLiftMotor.setPower(VSPower);
      } else if (rowNumber == 3) {
        rightLiftMotorAsDcMotor.setTargetPosition(row3 - SlightlyLoweredCommonDifference);
        leftLiftMotor.setTargetPosition(row3 - SlightlyLoweredCommonDifference);
        rightLiftMotorAsDcMotor.setPower(VSPower);
        leftLiftMotor.setPower(VSPower);
      } else if (rowNumber == 4) {
        rightLiftMotorAsDcMotor.setTargetPosition(row4 - SlightlyLoweredCommonDifference);
        leftLiftMotor.setTargetPosition(row4 - SlightlyLoweredCommonDifference);
        rightLiftMotorAsDcMotor.setPower(VSPower);
        leftLiftMotor.setPower(VSPower);
      } else if (rowNumber == 5) {
        rightLiftMotorAsDcMotor.setTargetPosition(row5 - SlightlyLoweredCommonDifference);
        leftLiftMotor.setTargetPosition(row5 - SlightlyLoweredCommonDifference);
        rightLiftMotorAsDcMotor.setPower(VSPower);
        leftLiftMotor.setPower(VSPower);
      } else if (rowNumber == 6) {
        rightLiftMotorAsDcMotor.setTargetPosition(row6 - SlightlyLoweredCommonDifference);
        leftLiftMotor.setTargetPosition(row6 - SlightlyLoweredCommonDifference);
        rightLiftMotorAsDcMotor.setPower(VSPower);
        leftLiftMotor.setPower(VSPower);
      } else if (rowNumber == 7) {
        rightLiftMotorAsDcMotor.setTargetPosition(row7 - SlightlyLoweredCommonDifference);
        leftLiftMotor.setTargetPosition(row7 - SlightlyLoweredCommonDifference);
        rightLiftMotorAsDcMotor.setPower(VSPower);
        leftLiftMotor.setPower(VSPower);
      } else if (rowNumber == 8) {
        rightLiftMotorAsDcMotor.setTargetPosition(row8 - SlightlyLoweredCommonDifference);
        leftLiftMotor.setTargetPosition(row8 - SlightlyLoweredCommonDifference);
        rightLiftMotorAsDcMotor.setPower(VSPower);
        leftLiftMotor.setPower(VSPower);
      }
    } else if (SlightlyLoweredTrue_False == false) {
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
        // This is for the ReadyToGrabPixels Mode
      } else if (rowNumber == 0.5) {
        rightLiftMotorAsDcMotor.setTargetPosition(outtakeReady4PickupHeight);
        leftLiftMotor.setTargetPosition(outtakeReady4PickupHeight);
        rightLiftMotorAsDcMotor.setPower(VSPower);
        leftLiftMotor.setPower(VSPower);
      }
    }
  }

  /**
   * Describe this function...
   */
  private void drivingW_outPixles() {
    ReadyToGrabPixelsStep = 0;
    runIntake("Stop");
  }

  /**
   * Describe this function...
   */
  private void setWristAngle(int my_0123456) {
    if (my_0123456 == 0) {
      wristServoAsServo.setPosition(desiredWristAngle0);
    } else if (my_0123456 == 1) {
      wristServoAsServo.setPosition(desiredWristAngle1);
    } else if (my_0123456 == 2) {
      wristServoAsServo.setPosition(desiredWristAngle2);
    } else if (my_0123456 == 3) {
      wristServoAsServo.setPosition(desiredWristAngle3);
    } else if (my_0123456 == -1) {
      wristServoAsServo.setPosition(desiredWristAngle4);
    } else if (my_0123456 == -2) {
      wristServoAsServo.setPosition(desiredWristAngle5);
    } else if (my_0123456 == -3) {
      wristServoAsServo.setPosition(desiredWristAngle6);
    }
  }

  /**
   * Describe this function...
   */
  private void SetMode() {
    // Remember to check buttons and gamepads
    if (gamepad1.dpad_down == false || gamepad1.right_bumper == false) {
      climberWinchMotor.setPower(0);
    }
    if (RobotMode.equals("RobotJustStarted") && (gamepad1.left_stick_x > 0.4 || gamepad1.right_stick_x > 0.4)) {
      RobotMode = "drivingW/outpixles";
    } else if (RobotMode.equals("drivingW/outpixles") && gamepad2.right_trigger > 0.4) {
      RobotMode = "ReadyToGrabPixels";
    } else if (gamepad1.dpad_up == true && gamepad1.y == true) {
      climberReleaseServo.setPosition(0.6);
    } else if (gamepad1.dpad_down == true && gamepad1.y == true) {
      climberWinchMotor.setPower(1);
    } else if (gamepad1.dpad_left == true && gamepad1.y == true) {
      climberWinchMotor.setPower(-1);
    } else if (gamepad1.b == true) {
      droneReleaseServo.setPosition(0.4);
    } else if (RobotMode.equals("ReadyToGrabPixels") && gamepad2.right_trigger > 0.4 && ReadyToGrabPixelsStep == 5) {
      ReadyToGrabPixelsTimer = getRuntime();
      callRowHeight(0.5, 0.25, false);
      FourbarPosition("Intake");
      ReadyToGrabPixelsStep = 1;
      RobotMode = "ReadyToGrabPixels";
    } else if (gamepad1.right_trigger > 0.4 && slowModeLatch == false) {
      Slow_mode(1);
    } else if (gamepad1.right_trigger < 0.4 && slowModeLatch == true) {
      Slow_mode(0);
    } else if (gamepad1.back == true) {
      imuAsBNO055IMU.initialize(new BNO055IMU.Parameters());
    } else if (gamepad1.x == true && Snap2ZeroLatch == true) {
      Snap2ZeroLatch = false;
      Snap2Zero();
    } else if (gamepad1.x == false && Snap2ZeroLatch == false) {
      Snap2ZeroLatch = true;
    }
    if (gamepad2.back == true && ZeroLiftLatch == false) {
      leftLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      rightLiftMotorAsDcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      leftLiftMotor.setPower(-0.25);
      rightLiftMotorAsDcMotor.setPower(-0.25);
      ZeroLiftLatch = true;
    } else if (gamepad2.back == false && ZeroLiftLatch == true) {
      leftLiftMotor.setPower(0);
      rightLiftMotorAsDcMotor.setPower(0);
      leftLiftMotor.setTargetPosition(0);
      rightLiftMotorAsDcMotor.setTargetPosition(0);
      leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      rightLiftMotorAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      rightLiftMotorAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      rowNumber = 0;
      RobotMode = "drivingW/outpixles";
      ZeroLiftLatch = false;
    }
  }

  /**
   * Describe this function...
   */
  private void runIntake(String runIntakeInOutStop) {
    if (runIntakeInOutStop.equals("In")) {
      intakeMotorAsDcMotor.setPower(1);
      intakeMotorAsDcMotor.setTargetPosition(2000);
    } else if (runIntakeInOutStop.equals("Out")) {
      intakeMotorAsDcMotor.setPower(-1);
      intakeMotorAsDcMotor.setTargetPosition(-2000);
    } else if (runIntakeInOutStop.equals("Stop")) {
      intakeMotorAsDcMotor.setPower(0);
    }
  }

  /**
   * Describe this function...
   */
  private void Snap2Zero() {
    double SnapAngle;
    boolean Snap2ZeroLatch3;

    SnapAngle = 90 - ReturnMeasuredAngle();
    if (Snap2ZeroLatch == false && ReturnMeasuredAngle() > 89 && ReturnMeasuredAngle() < 91) {
      Snap2ZeroLatch = true;
    }
    if (SnapAngle < 180 && Snap2ZeroLatch == false) {
      Snap2ZeroLatch2 = true;
    } else if (SnapAngle > 180 && Snap2ZeroLatch == false) {
      SnapAngle = (SnapAngle - 180) * -1;
      Snap2ZeroLatch3 = true;
    }
    if (Snap2ZeroLatch2 == true && SnapAngle < 0) {
      frontRightMotorAsDcMotor.setPower(Snap2ZeroPower);
      backRightMotorAsDcMotor.setPower(Snap2ZeroPower);
      frontLeftMotor.setPower(-Snap2ZeroPower);
      backLeftMotor.setPower(-Snap2ZeroPower);
    } else if (Snap2ZeroLatch2 == true && ReturnMeasuredAngle() > 89 && ReturnMeasuredAngle() < 91) {
      Snap2ZeroLatch2 = false;
      Snap2ZeroLatch = true;
    }
    if (Snap2ZeroLatch3 == true && SnapAngle > 0) {
      frontRightMotorAsDcMotor.setPower(-Snap2ZeroPower);
      backRightMotorAsDcMotor.setPower(-Snap2ZeroPower);
      frontLeftMotor.setPower(Snap2ZeroPower);
      backLeftMotor.setPower(Snap2ZeroPower);
    } else if (Snap2ZeroLatch3 == true && ReturnMeasuredAngle() > 89 && ReturnMeasuredAngle() < 91) {
      Snap2ZeroLatch3 = false;
      Snap2ZeroLatch = true;
    }
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
      passOffServo2.setPower(1);
    } else if (passoffConveyor2.equals("Rest")) {
      passoffServoAsCRServo.setDirection(CRServo.Direction.REVERSE);
      passOffServo2.setDirection(CRServo.Direction.FORWARD);
      passoffServoAsCRServo.setPower(0);
      passOffServo2.setPower(0);
    } else if (passoffConveyor2.equals("getReady")) {
      if (passoffLatch == false) {
        passoffServoAsCRServo.setDirection(CRServo.Direction.FORWARD);
        passOffServo2.setDirection(CRServo.Direction.FORWARD);
        passoffServoAsCRServo.setPower(0.5);
        passOffServo2.setPower(0.5);
      } else if (passoffLatch == true) {
        passoffServoAsCRServo.setDirection(CRServo.Direction.REVERSE);
        passOffServo2.setDirection(CRServo.Direction.FORWARD);
        passoffServoAsCRServo.setPower(1);
        passOffServo2.setPower(0.7);
      }
      if (getRuntime() == passoffServoTimer + 0.15) {
        passoffServoAsCRServo.setDirection(CRServo.Direction.REVERSE);
        passOffServo2.setDirection(CRServo.Direction.FORWARD);
        passoffServoAsCRServo.setPower(1);
        passOffServo2.setPower(0.7);
        passoffLatch = true;
      }
    }
  }
}
