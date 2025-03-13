package frc.robot;


public final class Constants {


  public static final class Climb {
    /** This is the difference between the angle of the fully deployed cams and the angle of the fully retracted cams. */
    public static final double totalCamTravelAngle = 120;
    /** This is how many revolutions the cams make for one revolution of the motors. */
    public static final double gearRatio = 1/80;
    /** In amps, this is the current the motors should cut off at to avoid damaging the frame. */
    public static final double cutoffCurrent = 1.0;

  }

  public static final class CoralScoring {
    /** In inches, this is the retracted length of the arm */
    public static final double retractedArmLength = 30;
    /** In inches, this is the maximum extension of the arm. 0 is when it is all they way in. */
    public static final double maxExtension = 100;
    /** In radians, the maximum the shoulder can pivot. 0 is when it rests on the climb riser. */
    public static final double maxShoulderRotation = Math.PI / 2; 
    /** This is the position PID controller for the extension. Proportinal, Integral, Derivative */
    public static final double[] extendorPositionPID = {0.1, 0, 0};
    /** This is the velocity PID controller for the cams. Proportinal, Integral, Derivative, velocityFF */
    public static final double[] extendorVelocityPID = {0.001, 0, 0, 1.0 / 5767};
    /** This is the position PID controller for the shoulder. Proportinal, Integral, Derivative */
    public static final double[] shoulderPositionPID = {0.1, 0, 0};
    /** This is the velocity PID controller for the cams. Proportinal, Integral, Derivative, velocityFF */
    public static final double[] shoulderVelocityPID = {0.001, 0, 0, 1.0 / 5767};

    /** In rad/s, the maximum shoulder pivot speed */
    public static final double maxShoulderPivotSpeed = 0.5;
    /** In in/s, the maximum etension/retraction speed of the arm */
    public static final double maxExtensionSpeed = 1.5;
  }
}
