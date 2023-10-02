package pidNonsense;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.sun.tools.javac.comp.Todo;

@Config
@TeleOp (group = "tests")
public class PIDF_Lift extends OpMode {
    public PIDController controller;

    //TODO: edit variables on dashboard lol

    // p = increase if not reaching target position
    public static double p = 0, i = 0, d = 0; // d = dampener (dampens arm movement and is scary). ignore i
    public static double f = 0;  // prevents arm from falling from gravity

    public static int Ltarget = 0; // target position
    public static int Rtarget = 0;
    private final double ticks_in_degree = 700/180.0; //look this up later please :"" 537.6 / 360.0   700/ 180.0

    private DcMotorEx larm;
    private DcMotorEx rarm;

    @Override
    public void init(){
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        larm = hardwareMap.get(DcMotorEx.class,"la");
        rarm = hardwareMap.get(DcMotorEx.class,"ra");

        larm.setDirection(DcMotorEx.Direction.FORWARD);
        rarm.setDirection(DcMotorEx.Direction.REVERSE);

    }

    @Override
    public void loop(){

        controller.setPID(p, i, d);

        int larmPos = larm.getCurrentPosition();
        int rarmPos = rarm.getCurrentPosition();

        double Lpid = controller.calculate(larmPos, Ltarget);
        double Rpid = controller.calculate(rarmPos, Rtarget);

        //double Lff = Math.cos(Math.toRadians(Ltarget / ticks_in_degree)) * f;
        //double Rff = Math.cos(Math.toRadians(Rtarget / ticks_in_degree)) * f;

        double Lpower = Lpid + f;
        double Rpower = Rpid + f;

        larm.setPower(Lpower);
        rarm.setPower(Rpower);

        telemetry.addData("Lpos", larmPos);
        telemetry.addData("Rpos", rarmPos);
        telemetry.addData("Ltarget", Ltarget);
        telemetry.addData("Rtarget", Rtarget);
        telemetry.update();
    }
}