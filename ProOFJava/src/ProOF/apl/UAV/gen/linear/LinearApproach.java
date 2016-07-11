/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.gen.linear;

import ProOF.apl.UAV.Swing.Graphics2DReal;
import ProOF.apl.UAV.abst.UAVApproach;
import ProOF.apl.UAV.gen.linear.vehicle.fLinearVehicle;
import ProOF.apl.UAV.gen.linear.vehicle.oLinearVehicle;
import ProOF.apl.UAV.gen.linear.vehicle.parts.pLinearWaypoints;
import ProOF.com.Linker.LinkerApproaches;

/**
 *
 * @author marcio
 * @param <Model>
 */
public abstract class LinearApproach<Model extends LinearModel> extends UAVApproach<Model> implements LinearSystem<Model>{
    protected oLinearVehicle vehicle;
    protected pLinearWaypoints waypoint;

    @Override
    public void services(LinkerApproaches link) throws Exception {
        super.services(link); //To change body of generated methods, choose Tools | Templates.
        vehicle = link.get(fLinearVehicle.obj, vehicle);
        waypoint = link.need(pLinearWaypoints.class, waypoint);
    }
    @Override
    public void load() throws Exception {
        super.load(); //To change body of generated methods, choose Tools | Templates.
        vehicle.start(this);
    }
    @Override
    public void paint(Graphics2DReal gr, double size) throws Exception {
        
    }
    @Override
    public abstract int N() throws Exception;
    
    @Override
    public int Waypoints() {
        return waypoint.Waypoints();
    }
    @Override
    public double maxControl() {
        return waypoint.maxControl();
    }
    @Override
    public double maxVelocity() {
        return waypoint.maxVelocity();
    }
    @Override
    public LinearState[] build_states(Model model) throws Exception{
        return vehicle.build_states(this, model);
    }

    @Override
    public LinearControl[] build_controls(Model model) throws Exception{
        return vehicle.build_controls(this, model);
    }

    @Override
    public void addConstraints(Model model) throws Exception{
        vehicle.addConstraints(this, model);
    }

    @Override
    public oLinearVehicle vehicle() {
        return vehicle;
    }
    
}
