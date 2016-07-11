/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.gen.linear.vehicle.parts;

import ProOF.apl.UAV.abst.vehicle.parts.oSpecific;
import ProOF.apl.UAV.gen.linear.LinearModel;
import ProOF.apl.UAV.gen.linear.LinearSystem;

/**
 *
 * @author marcio
 * @param <App>
 * @param <Model>
 */
public abstract class oLinearSpecific <App extends LinearSystem, Model extends LinearModel> extends oSpecific<App, Model>{

    @Override
    public abstract void addConstraints(App approach, Model model) throws Exception;
    
}

