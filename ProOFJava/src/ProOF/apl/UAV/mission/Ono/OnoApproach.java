/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.mission.Ono;

import ProOF.CplexExtended.iCplexExtract;
import ProOF.apl.UAV.Swing.Graphics2DReal;
import ProOF.apl.UAV.abst.UAVModel.Callback;
import ProOF.apl.UAV.abst.uncertainty.Delta;
import ProOF.apl.UAV.gen.linear.LinearApproach;
import ProOF.apl.UAV.gen.linear.mission.parts.oLinearObjective;
import ProOF.apl.UAV.gen.linear.uncertainty.LinearPerfectControlUncertainty;
import ProOF.apl.UAV.gen.linear.uncertainty.pLinearStateUncertainty;
import ProOF.apl.UAV.gen.linear.vehicle.parts.oLinearDynamic;
import ProOF.com.Linker.LinkerApproaches;
import ProOF.com.Linker.LinkerParameters;
import ProOF.utilities.uUtil;
import java.awt.Color;

/**
 *
 * @author marcio
 */
public class OnoApproach extends LinearApproach<OnoModel>{
    //private final BlackmoreStateUncertainty to_add = new BlackmoreStateUncertainty(this);
    //private final RoverStateUncertainty to_add = new RoverStateUncertainty(this);
    private final fOnoUncertainty to_get = new fOnoUncertainty(this);
    public final OnoPlot plot = new OnoPlot(this);
    
    public OnoInstance inst;
    public pLinearStateUncertainty unc;
    protected oLinearObjective objective;
    
    public oLinearDynamic dynamic;
    
    private final Delta delta = Delta.obj;
    
    private int Naprox;
    private int Nsteps;
    protected boolean strengthened[];
    @Override
    public String name() {
        return "Ono";
    }
    @Override
    public void services(LinkerApproaches link) throws Exception {
        super.services(link); //To change body of generated methods, choose Tools | Templates.
        link.add(delta);
        link.add(plot);
        link.add(fOnoOperator.obj);
        link.add(new LinearPerfectControlUncertainty(this));
        
        unc = link.get(to_get, unc);
        inst = link.get(fOnoInstance.obj, inst);
        objective = link.get(fOnoObjective.obj, objective);
        
        dynamic = link.need(oLinearDynamic.class, dynamic);
        
        //link.add(to_add);
        //unc = link.need(pLinearStateUncertainty.class, unc);
        
    }
    @Override
    public void parameters(LinkerParameters link) throws Exception {
        super.parameters(link); //To change body of generated methods, choose Tools | Templates.
        Naprox  = link.Int("Avoid-N-risk", 12, 1, 64, 
                "Number of picewise restriction to aproximate the "+
                "inverse of erf(x) to calculate the risk allocation "+
                "for obstacle avoidance");
        //Nsteps  = link.Int("Steps", 10, 5, 1000, "Number of steps to perform changes in obstacle edges");
        Nsteps = 6+1;
        strengthened = link.ListBool("Strengthened", "case-1", "case-2", "case-3", "case-5", "case-delta", "case-swaps", "case-implies", "bin-St", "steps");
        //Naprox = 8;
    }
    public int Naprox() {
        return Naprox;
    }
    private int waypoints[];
    private int steps[];
    @Override
    public void load() throws Exception {
        super.load(); //To change body of generated methods, choose Tools | Templates.
        if(strengthened[8] && !strengthened[7]){
            System.out.println("--------------------------[ "+name()+" - load()"+" ]--------------------------");
            int waypoints_distribution[] = new int[Steps()];
//            int total = Waypoints();
//            while(total>0){
//                for(int s=0; s<waypoints_distribution.length && total>0; s++){
//                    waypoints_distribution[s]++;
//                    total--;
//                }
//            }
            double remainder = 0;
            for(int s=0; s<Steps(); s++){
                double slice = remainder+(Waypoints()+1)*1.0/Steps();
                waypoints_distribution[s] = (int)(slice+1e-6);
                remainder = (slice-waypoints_distribution[s]);
            }
            uUtil.Print(System.out, "distribution:\n","\n", "%d", waypoints_distribution);

            waypoints = new int[Steps()+1];
            for(int s=1; s<waypoints.length; s++){
                waypoints[s] = waypoints[s-1] + waypoints_distribution[s-1];
            }
            uUtil.Print(System.out, "waypoints[s]:\n","\n", "%d", waypoints);
            steps = new int[Waypoints()+1];
            for(int s=0; s<Steps(); s++){
                for(int t=waypoints[s]; t<waypoints[s+1]; t++){
                    steps[t] = s;
                }
//                if(s+1==Steps()){
//                    steps[Waypoints()] = s;
//                }
            }
            uUtil.Print(System.out, "steps[t]:\n","\n", "%d", steps);
        }
    }
    public double Delta() throws Exception {
        return delta.Delta();
    }
    public double dt(){
        return waypoint.dt();
    }
    @Override
    public int N() throws Exception {
        return inst.N();
    }
    
    @Override
    public void solutionCallback(iCplexExtract ext, OnoModel model, Callback type) throws Exception{ 
        model.extract(ext, type);
        plot.addModel(model);
    }

    @Override
    public void repaint(OnoModel model) throws Exception {
        plot.addModel(model);
    }
    
    @Override
    public void paint(Graphics2DReal gr, double size) throws Exception {
        super.paint(gr, size); //To change body of generated methods, choose Tools | Templates.
        
        gr.paintOvalR(inst.start_state[0], inst.start_state[1], 0.01*size, 0.01*size, Color.ORANGE, Color.BLACK);
    }

    @Override
    public int Steps() {
        return Nsteps;
    }

    @Override
    public int Waypoint(int step) {
        return waypoints[step];
    }

    @Override
    public int Step(int t) {
        return steps[t];
    }

    @Override
    public void addObjective(OnoModel model) throws Exception {
        objective.addObjective(this, model);
    }
    
    
}
