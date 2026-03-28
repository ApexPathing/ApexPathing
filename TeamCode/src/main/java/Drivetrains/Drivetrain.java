package Drivetrains;

import com.qualcomm.robotcore.hardware.DcMotorEx;
<<<<<<< HEAD
<<<<<<< HEAD
import com.qualcomm.robotcore.hardware.HardwareMap;

=======
>>>>>>> 5e9d75400a1896ae8936189bff63bffa3688d89b
=======
import com.qualcomm.robotcore.hardware.HardwareMap;


>>>>>>> be745d4b69d7d4ab5db5f97c7cc37107e354a4af

import java.util.List;

/**
 * Abstract class implemented by all drivetrain classes
 * @author Sohum Arora
 */
public abstract class Drivetrain {

<<<<<<< HEAD
<<<<<<< HEAD
    //set power methods
    public abstract void setPower(DcMotorEx motor,double power);
    public abstract void setPower(List<DcMotorEx> motors, double power);
=======
    //set power methods
    public abstract void setPower(DcMotorEx motor,double power);
    public abstract void setPower(List<DcMotorEx> motors, double power);

>>>>>>> be745d4b69d7d4ab5db5f97c7cc37107e354a4af
    public abstract void setPower(double power);
    //drive train init method
    public abstract void initDrive(HardwareMap hardwareMap, String lfName, String rfName, String lrName, String rrName);

<<<<<<< HEAD
=======
    public abstract void setPower(DcMotorEx motor,double power);
    public abstract void setPower(List<DcMotorEx> motors, double power);
>>>>>>> 5e9d75400a1896ae8936189bff63bffa3688d89b
=======

>>>>>>> be745d4b69d7d4ab5db5f97c7cc37107e354a4af
}
