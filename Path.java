//A combination of splines will be a path.
import java.util.ArrayList;
public class Path{
    ArrayList<CubicSpline> splines;
    double maxSpeed;
    double maxAccel;
    public final double timePeriod = .250;

    public Path(ArrayList<CubicSpline> splines, double maxSpeed){
        this.splines = splines;
        this.maxSpeed = maxSpeed;
    }

    public double findVelocity(double t, double xi, double vi, double a){
        
    }

    //creates a list of points that contain information on velocity, position, accel, and theta
    public ArrayList<TimePoint> createTimepoints(){
        ArrayList<TimePoint> timePoints = new ArrayList<TimePoint>();
        
        double xi = 0;
        double vi = 0;
        double ai = maxAccel;
        while(xi < totalDist){
            if(maxAccel*timePeriod+vi < maxSpeed){
                
            }

        }
        
        return timePoints;
    }


}