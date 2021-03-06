package agents;

import java.awt.Graphics2D;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import yaes.framework.simulation.SimulationInput;
import yaes.sensornetwork.agents.ForwarderSensorAgent;
import yaes.sensornetwork.model.SensorNode;
import yaes.virtualcoordinate.VCContext;

import Jama.Matrix;
import Jama.SingularValueDecomposition;

public class BaseStation {
	
	private double sampleTime;
	private Graphics2D detectionEllipse;
	private List<VCAgent> detectionSensors;
	private double timeWindow;
	private VCAgent mobileTarget;
	private ArrayList<VCAgent> neighbourAgents;
	private Matrix mobileVC;
	private double[] currentMobileTC = new double[2];
	private double[] prevMobileTC = new double[2];
	private double[] predictedMobileTC = new double[2];
	private HashMap<VCAgent, double[]> TCofNetwork;
	private Matrix P;
	private Matrix A;
	private Matrix V;
	
	public BaseStation(SimulationInput sim, VCContext context) {
		setSampleTime(sim.getTimeResolution());
		List<VCAgent> myAgents = VCMessageHelper.getAllVCAgents(context.getWorld(), false);
		List<VCAgent> anchorAgents = VCMessageHelper.getAnchorAgents(
				context.getWorld(), ForwarderSensorAgent.class);
		getVCs(myAgents, anchorAgents);
		findTC(myAgents);
	}
	
	public HashMap<VCAgent, double[]> findTC(List<VCAgent> myAgents) {
		
		Matrix Psvd;
		SingularValueDecomposition B = new SingularValueDecomposition(A);
		V = B.getV().transpose();
		Psvd = P.times(V);

		for (int i  = 0; i < Psvd.getRowDimension(); i++) {
			double thetaT = Math.atan( (Psvd.get(i, 2)) / (Psvd.get(i, 1)));
			double rT = Math.sqrt(Math.pow(Psvd.get(i, 0), 2) + 
								  Math.pow(Psvd.get(i, 1), 2) + 
								  Math.pow(Psvd.get(i, 2), 2) );
			double[] axis = {rT * Math.cos(thetaT), rT * Math.sin(thetaT)};
			TCofNetwork.put(myAgents.get(i), axis);
			
		}
		
		return TCofNetwork;
		
		
	}
	
	public void sample() {
		Matrix Msvd;
		Msvd = mobileVC.times(V);
		double thetaT = Math.atan( (Msvd.get(1, 2)) / (Msvd.get(1, 1)));
		double rT = Math.sqrt(Math.pow(Msvd.get(1, 0), 2) + 
				  Math.pow(Msvd.get(1, 1), 2) + 
				  Math.pow(Msvd.get(1, 2), 2) );
		currentMobileTC[0] = rT * Math.cos(thetaT);
		currentMobileTC[1] = rT * Math.sin(thetaT);
	}
	
	public void predictTC() {
		
		double temp = Math.sqrt(Math.pow((currentMobileTC[0] - prevMobileTC[0]), 2) +
								Math.pow((currentMobileTC[1] - prevMobileTC[1]), 2));
		double velocity = temp/sampleTime;
		
		double angle = Math.acos((currentMobileTC[0] - prevMobileTC[0])/temp);
		
		predictedMobileTC[0] = currentMobileTC[0] + velocity * sampleTime * Math.cos(angle);
		predictedMobileTC[1] = currentMobileTC[1] + velocity * sampleTime * Math.sin(angle);
		
		prevMobileTC[0] = currentMobileTC[0];
		prevMobileTC[1] = currentMobileTC[1];
	}
	
	public void createEllipse(){
		
	}
	
	public void alertESernsors(){
		
	}
	
	private void getVCs(List<VCAgent> myAgents, List<VCAgent> anchorAgents){
		
		double[][] VCs = new double[myAgents.size()][anchorAgents.size()];
		double[][] AVCs = new double[anchorAgents.size()][anchorAgents.size()];
		
		int i = 0;
		
		for (VCAgent anchor : anchorAgents) {
			int j = 0;
			int k = 0;
			VCLocalTable table = anchor.getVcLocalTable();
			for (VCAgent agent : myAgents) {
				VCs[j][i] = table.getNumberOfHops(agent, anchor);
				j++;
				if (agent.isAnchor()) {
					AVCs[k][i] = table.getNumberOfHops(agent, anchor);
					k++;
				}
			}
			i++;
		}

		P = new Matrix(VCs);
		A = new Matrix(AVCs);
	}
	//at each time step multiple of sample time set the currentMobileTC value in update function
	
	
	public double[] getPrevTC() {
		return prevMobileTC;
	}
	
	public double[] getFutureTC() {
		return predictedMobileTC;
	}
	
	public double[] getCurrentTC() {
		return currentMobileTC;
	}
	
	public void setMobileVC(double[] VC, int anchorsize) {
		mobileVC = new Matrix(VC, anchorsize);
	}
	
	private void setSampleTime(double time) {
		sampleTime = time;
	}

}
