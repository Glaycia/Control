package Trajectories;

import java.util.ArrayList;

import Splines.QuinticSpline;
import Splines.Vector2;
import Splines.Waypoint;
import Utility.DesmosCopyGen;

public class TrajectoryTest {
	public static void main(String[] args) {
		ArrayList<Waypoint> Waypoints = new ArrayList<>();
		
		Waypoints.add(new Waypoint(new Vector2(0, 0), new Vector2(10, 0), 0));
		Waypoints.add(new Waypoint(new Vector2(10, 10), new Vector2(10, 0), 1));
		Waypoints.add(new Waypoint(new Vector2(20, 15), new Vector2(10, 0), 1));
		
		TrajectoryGenerator path_1 = new TrajectoryGenerator(0.6, 2, Waypoints);
		
		path_1.ForwardPass();
		path_1.BackwardsPass();
		path_1.CalculateTime();
		
		ArrayList<Vector2> PlotData = new ArrayList<>();
		
		for(int i = 0; i < path_1.pathpoints.size(); i++) {
			PlotData.add(new Vector2(path_1.pathpoints.get(i).timestamp, path_1.pathpoints.get(i).linVelocity));
		}
		DesmosCopyGen printer = new DesmosCopyGen(PlotData);
		DesmosCopyGen printer_2 = new DesmosCopyGen(path_1.path.Pathpoints);
		printer.printWithPoints();
		printer_2.printWithLine();
	}
}
