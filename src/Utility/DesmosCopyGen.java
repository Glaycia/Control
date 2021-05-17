package Utility;
import java.util.ArrayList;

import Splines.Vector2;

public class DesmosCopyGen {
    ArrayList<Vector2> PointList = new ArrayList<Vector2>();
    public DesmosCopyGen(ArrayList<Vector2> ListOfPoints){
        this.PointList = ListOfPoints;
    }
    double MagCheck(double in) {
    	if(in < 1e-3) {
    		return 0;
    	}else {
    		return in;
    	}
    }
    public void printWithPoints() {
        for(int i = 0; i < PointList.size(); i++) {
            String input = ("(" + MagCheck(PointList.get(i).x) + ", " + MagCheck(PointList.get(i).y) + ")");
            System.out.println(input);
        }
    }
    public void printWithLine() {
        for(int i = 1; i < PointList.size(); i++) {
            String input = ("((1-t)(" + MagCheck(PointList.get(i - 1).x) + ") + t(" + MagCheck(PointList.get(i).x) + "), (1-t)(" + MagCheck(PointList.get(i - 1).y) + ") + t(" + MagCheck(PointList.get(i).y) + "))");
            System.out.println(input);
        }
    }
}