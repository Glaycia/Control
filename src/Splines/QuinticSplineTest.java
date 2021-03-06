package Splines;

import java.util.ArrayList;

import Utility.DesmosCopyGen;

public class QuinticSplineTest {
	public static void main(String[] args) {
		ArrayList<Waypoint> Waypoints = new ArrayList<>();
		
		Waypoints.add(new Waypoint(new Vector2(0, 0), new Vector2(0, 100)));
		Waypoints.add(new Waypoint(new Vector2(1, 10), new Vector2(0, 100)));
		
		QuinticSpline Path = new QuinticSpline(Waypoints);
		Path.generateSegments();
		Path.sampleSegments();
		DesmosCopyGen Printer = new DesmosCopyGen(Path.Pathpoints);
		Printer.printWithPoints();
		//Path.integrateLength(0);
		//System.out.println(Path.length);
	}
}
