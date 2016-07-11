/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.gen.linear;

import ProOF.CplexExtended.iCplexExtract;
import ProOF.apl.UAV.Swing.Graphics2DReal;
import ProOF.apl.UAV.abst.UAVModel;
import ProOF.apl.UAV.abst.UAVModel.Callback;
import ProOF.apl.UAV.abst.UAVSystem;
import ProOF.apl.UAV.gen.linear.vehicle.oLinearVehicle;
import ilog.concert.IloException;

/**
 *
 * @author marcio
 * @param <Model>
 */
public interface LinearSystem <Model extends UAVModel> extends UAVSystem{
    public int N() throws Exception;
    public int Waypoints();
    public double maxControl();
    public double maxVelocity();
    public double Delta() throws Exception;
    
    public int Steps();
    public int Waypoint(int step);
    public double dt();
    public int Step(int t);
    
    public void solutionCallback(iCplexExtract ext, Model model, Callback type) throws Exception;
    public void repaint(Model model) throws Exception;
    public void paint(Graphics2DReal gr, double size) throws Exception;
    
    public LinearState[] build_states(Model model) throws Exception;
    public LinearControl[] build_controls(Model model) throws Exception;
    public void addConstraints(Model model) throws Exception;
    public void addObjective(Model model) throws Exception;
    
    public oLinearVehicle vehicle();
}
