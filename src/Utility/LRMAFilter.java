package Utility;
import java.util.ArrayList;
import Splines.Vector2;

public class LRMAFilter {
	ArrayList<Vector2> data;
	double slope;
	double intercept;
	int defMaxSample;
	
	public LRMAFilter() {}
	public LRMAFilter(int maxSample) {
		this.defMaxSample = maxSample;
	}
	public LRMAFilter(ArrayList<Vector2> data) {
		this.data = data;
	}
	public LRMAFilter(ArrayList<Vector2> data, int maxSample) {
		this.data = data;
		this.defMaxSample = maxSample;
	}
	double predict(double time) {
		return slope * time + intercept;
	}
	void updateRegression() {
		updateRegression(defMaxSample);
	}
	void updateRegression(int maxSample) {
		if(data.size() >= maxSample) {
			data = (ArrayList<Vector2>) data.subList(data.size() - 1 - maxSample, data.size() - 1);
		}
		slope = (data.size() * productSum() - xSum() * ySum())/(data.size() * xSquaredSum() - xSum() * xSum());
		intercept = (data.size() * ySum() - slope * xSum())/data.size();
	}
	
	double productSum() {
		double sum = 0;
		for(int i = 0; i < data.size(); i ++) {
			sum += data.get(i).x * data.get(i).y;
		}
		return sum;
	}
	double xSum() {
		double sum = 0;
		for(int i = 0; i < data.size(); i ++) {
			sum += data.get(i).x;
		}
		return sum;
	}
	double ySum() {
		double sum = 0;
		for(int i = 0; i < data.size(); i ++) {
			sum += data.get(i).y;
		}
		return sum;
	}
	double xSquaredSum() {
		double sum = 0;
		for(int i = 0; i < data.size(); i ++) {
			sum += data.get(i).x * data.get(i).x;
		}
		return sum;
	}
}
