/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.mission.Blackmore;

import ProOF.apl.UAV.gen.linear.LinearSystem;
import ProOF.apl.UAV.gen.linear.mission.parts.oLinearObjective;
import ProOF.apl.UAV.gen.linear.uncertainty.pLinearStateUncertainty;
import ProOF.apl.UAV.gen.linear.vehicle.parts.oLinearDynamic;

/**
 *
 * @author marcio
 */
public interface BlackmoreSystem extends LinearSystem<BlackmoreModel>{
    public int ParticleControl();
    public BlackmoreInstance inst();
    public BlackmorePlot plot();
    public pLinearStateUncertainty unc();
    public oLinearObjective objective();
    public oLinearDynamic dynamic();
    public int Nsteps();
    public int Naprox();
}
