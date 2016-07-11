/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.gen.linear.vehicle;

import ProOF.apl.UAV.gen.linear.LinearControl;
import ProOF.apl.UAV.gen.linear.LinearState;
import ProOF.apl.UAV.gen.linear.LinearModel;
import ProOF.apl.UAV.abst.vehicle.oVehicle;
import ProOF.apl.UAV.gen.linear.LinearSystem;

/**
 *
 * @author marcio
 */
public abstract class oLinearVehicle<App extends LinearSystem, Model extends LinearModel> extends oVehicle<App, Model, LinearState, LinearControl>{

    public abstract void start(App approach) throws Exception;
    
    @Override
    public abstract LinearState[] build_states(App approach, Model model) throws Exception;

    @Override
    public abstract LinearControl[] build_controls(App approach, Model model) throws Exception;

    @Override
    public abstract void addConstraints(App approach, Model model) throws Exception;

    
}
