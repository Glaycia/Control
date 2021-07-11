package Utility;

import java.util.ArrayList;

import Splines.Vector2;

public class Approximator {
	ArrayList<Vector2> rawposition = new ArrayList<Vector2>();
	ArrayList<Vector2> rawvelocity = new ArrayList<Vector2>();
	ArrayList<Vector2> rawacceleration = new ArrayList<Vector2>();
	
	LRMAFilter posfilter = new LRMAFilter(rawposition, 10);
	LRMAFilter velfilter = new LRMAFilter(rawvelocity, 8);
	LRMAFilter accfilter = new LRMAFilter(rawacceleration, 6);
	
	int maxSampleSize = 10;
	
	public Approximator() {}
	//assuming 50 hz, we can sample to 10 points for 1/5 of a second
	
	void rawUpdate(Vector2 nextPoint) {
		// input as time, pos
		rawposition.add(nextPoint);
		if(rawposition.size() > maxSampleSize) rawposition = cutToLast(rawposition, maxSampleSize);
		if(rawposition.size() > 2) {
			ArrayList<Vector2> newvelocity = new ArrayList<Vector2>();
			ArrayList<Vector2> newacceleration = new ArrayList<Vector2>();
			
			for(int i = 0; i < maxSampleSize - 1; i ++) {
				newvelocity.add(new Vector2(rawposition.get(i + 1).x, (rawposition.get(i + 1).y - rawposition.get(i).y)/(rawposition.get(i + 1).x - rawposition.get(i).x)));
			}
			for(int i = 0; i < maxSampleSize - 2; i ++) {
				newacceleration.add(new Vector2(newvelocity.get(i + 1).x, (newvelocity.get(i + 1).y - newvelocity.get(i).y)/(newvelocity.get(i + 1).x - newvelocity.get(i).x)));
			}
		}
	}
	void filter() {
		posfilter = new LRMAFilter(rawposition, 10);
		velfilter = new LRMAFilter(rawvelocity, 8);
		accfilter = new LRMAFilter(rawacceleration, 6);
		
		posfilter.updateRegression();
		velfilter.updateRegression();
		accfilter.updateRegression();
	}
	double predictpos(double time) {
		return posfilter.predict(time);
	}
	double predictvel(double time) {
		return velfilter.predict(time);
	}
	double predictacc(double time) {
		return accfilter.predict(time);
	}
	ArrayList<Vector2> cutToLast(ArrayList<Vector2> data, int size){
		return (ArrayList<Vector2>) data.subList(data.size() - 1 - size, data.size() - 1);
	}
}
