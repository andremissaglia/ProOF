/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.problem.Blackmore;

import ProOF.CplexExtended.CplexExtended;
import ProOF.CplexExtended.iCplexExtract;
import ProOF.apl.UAV.Swing.Graphics2DReal;
import ProOF.apl.UAV.abst.UAVModel.Callback;
import ProOF.apl.UAV.abst.uncertainty.Delta;
import ProOF.apl.UAV.gen.linear.LinearControl;
import ProOF.apl.UAV.gen.linear.LinearState;
import ProOF.apl.UAV.gen.linear.mission.parts.fLinearObjective;
import ProOF.apl.UAV.gen.linear.mission.parts.oLinearObjective;
import ProOF.apl.UAV.gen.linear.uncertainty.pLinearStateUncertainty;
import ProOF.apl.UAV.gen.linear.vehicle.fLinearVehicle;
import ProOF.apl.UAV.gen.linear.vehicle.oLinearVehicle;
import ProOF.apl.UAV.gen.linear.vehicle.parts.oLinearDynamic;
import ProOF.apl.UAV.gen.linear.vehicle.parts.pLinearWaypoints;
import ProOF.apl.UAV.mission.Blackmore.BlackmoreInstance;
import ProOF.apl.UAV.mission.Blackmore.BlackmoreModel;
import ProOF.apl.UAV.mission.Blackmore.BlackmoreModelRiskAlloc;
import ProOF.apl.UAV.mission.Blackmore.BlackmorePlot;
import ProOF.apl.UAV.mission.Blackmore.BlackmoreSystem;
import ProOF.apl.UAV.mission.Blackmore.fBlackmoreAvoidance;
import ProOF.apl.UAV.mission.Blackmore.fBlackmoreInstance;
import ProOF.apl.UAV.mission.Blackmore.fBlackmoreUncertainty;
import ProOF.apl.UAV.mission.Blackmore.oBlackmoreAvoidance;
import ProOF.com.Linker.LinkerApproaches;
import ProOF.com.Linker.LinkerParameters;
import ProOF.com.Linker.LinkerResults;
import ProOF.gen.best.BestSol;
import ProOF.opt.abst.problem.meta.Objective;
import ProOF.opt.abst.problem.meta.Problem;
import ProOF.opt.abst.problem.meta.codification.Codification;
import ProOF.utilities.uUtil;
import java.awt.Color;

/**
 *
 * @author marcio
 */
public class BlackmoreProblem extends Problem<BestSol> implements BlackmoreSystem{
    private final fBlackmoreUncertainty to_get = new fBlackmoreUncertainty(this);
    public final BlackmorePlot plot = new BlackmorePlot(this);
    
    public BlackmoreInstance inst;
    public pLinearStateUncertainty unc;
    
    
    protected oLinearVehicle vehicle;
    protected pLinearWaypoints waypoint;
    protected oLinearObjective objective;
    public oLinearDynamic dynamic;
    
    private final Delta delta = Delta.obj;
    
    private int Nsteps;
    private int Naprox;
    public int ParticleControl;
    
    protected BlackmoreModel model;
    protected CplexExtended cpx;
    protected oBlackmoreAvoidance avoid;
    
    public GraphMap map = new GraphMap();
    
    @Override
    public String name() {
        return "BlackmoreProblem";
    }
    @Override
    public Codification build_codif() throws Exception {
        return new BlackmoreCodification();
    }
    @Override
    public Objective build_obj() throws Exception {
        return new BlackmoreObjective();
    }
    
    
    @Override
    public void services(LinkerApproaches link) throws Exception {
        super.services(link); //To change body of generated methods, choose Tools | Templates.
        vehicle = link.get(fLinearVehicle.obj, vehicle);
        waypoint = link.need(pLinearWaypoints.class, waypoint);
        link.add(delta);
        link.add(plot);
        
        
        
        unc = link.get(to_get, unc);
        inst = link.get(fBlackmoreInstance.obj, inst);
        objective = link.get(fLinearObjective.obj, objective);
        avoid = link.get(fBlackmoreAvoidance.obj, avoid);
        
        dynamic = link.need(oLinearDynamic.class, dynamic);
        
        link.add(BlackmoreOperatorProblem.obj);
        
        
        //cuts = link.need(oCuts.class, cuts);
        
    }
    @Override
    public void parameters(LinkerParameters link) throws Exception {
        super.parameters(link); //To change body of generated methods, choose Tools | Templates.
        Naprox  = link.Int("Avoid-N-risk", 12, 1, 16, 
                "Number of picewise restriction to aproximate the "+
                "inverse of erf(x) to calculate the risk allocation "+
                "for obstacle avoidance");
        Nsteps  = link.Int("Steps", 10, 5, 1000, 
                "Number of steps to perform changes in obstacle edges");
        ParticleControl  = link.Int("Particle-Control", 10000, 100, 1000000000);
        //ParticleControl = 100;
        //Naprox = 8;
    }

    
    private int waypoints[];
    private int steps[];
    @Override
    public void load() throws Exception {
        super.load(); //To change body of generated methods, choose Tools | Templates.
        plot.setInst(inst);
        
        int waypoints_distribution[] = new int[Steps()];
        int total = Waypoints();
        while(total>0){
            for(int s=0; s<waypoints_distribution.length && total>0; s++){
                waypoints_distribution[s]++;
                total--;
            }
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
            if(s+1==Steps()){
                steps[Waypoints()] = s+1;
            }
        }
        uUtil.Print(System.out, "steps[t]:\n","\n", "%d", steps);
    }

    @Override
    public void start() throws Exception {
        super.start(); //To change body of generated methods, choose Tools | Templates.
        vehicle.start(this);
        cpx = new CplexExtended();
        model = new BlackmoreModelRiskAlloc(this, "RAA", cpx, delta.Delta(), false);
        for(int j=0; j<inst.J(); j++){
            for(int s=0; s<Steps()+1; s++){
                avoid.OandN(this, model, j, s);
            }
        }
        map.createGraph(this, inst, unc, delta.Delta());
        map.graph.dijkstraToTarget();
    }
    
    
    
    public int Naprox() {
        return Naprox;
    }
    @Override
    public double Delta() throws Exception {
        return delta.Delta();
    }

    @Override
    public int N() throws Exception {
        return inst.N();
    }
    
    @Override
    public int Steps() {
        return Nsteps;
    }
//    private final int SPLIT = 3;
//    public int Steps() {
//        return (waypoint.Waypoints()+SPLIT-1)/SPLIT;
//    }
    @Override
    public int Waypoint(int step){
        if(step>Steps()){
            return waypoint.Waypoints();
        }
        return waypoints[step];
        //return Math.min(step*SPLIT, waypoint.Waypoints());
    }
    @Override
    public double dt(){
        return waypoint.dt();
    }
    @Override
    public int Step(int t){
        return steps[t];
        //return t/SPLIT;
    }
    
    @Override
    public void solutionCallback(iCplexExtract ext, BlackmoreModel model, Callback type) throws Exception{ 
        //System.err.println("solution callback| model = "+model.name+" | type = "+type);
        model.extract(ext, type);
        plot.addModel(model);
    }

    @Override
    public void repaint(BlackmoreModel model) throws Exception {
        plot.addModel(model);
    }
    
    @Override
    public void paint(Graphics2DReal gr, double size) throws Exception {
        map.paint(gr);
        
        //landing start
        if(inst.Si!=null){
            gr.paintOvalR(inst.Si.x, inst.Si.y, 0.01*size, 0.01*size, Color.GREEN, Color.BLACK);
        }
        
        //landing end
        if(inst.Sf!=null){
            gr.paintOvalR(inst.Sf.x, inst.Sf.y, 0.01*size, 0.01*size, Color.YELLOW, Color.BLACK);
        }
        
        //landing start
        if(inst.Ei!=null){
            gr.paintRectR(inst.Ei.x, inst.Ei.y, 0.01*size, 0.01*size, Color.YELLOW, Color.BLACK);
        }
        
        //landing end
        if(inst.Ef!=null){
            gr.paintRectR(inst.Ef.x, inst.Ef.y, 0.01*size, 0.01*size, Color.GREEN, Color.BLACK);
        }
        
        //start state
        gr.paintOvalR(inst.start_state[0], inst.start_state[1], 0.01*size, 0.01*size, Color.ORANGE, Color.BLACK);
        
        
        //end point
        gr.paintRectR(inst.end_point[0], inst.end_point[1], 0.01*size, 0.01*size, Color.ORANGE, Color.BLACK);
        
        
    }

    @Override
    public BestSol best() {
        return BestSol.object();
    }

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
    public LinearState[] build_states(BlackmoreModel model) throws Exception {
        return vehicle.build_states(this, model);
    }

    @Override
    public LinearControl[] build_controls(BlackmoreModel model) throws Exception {
        return vehicle.build_controls(this, model);
    }

    @Override
    public void addConstraints(BlackmoreModel model) throws Exception {
        vehicle.addConstraints(this, model);
    }

    @Override
    public int ParticleControl() {
        return ParticleControl;
    }

    @Override
    public BlackmoreInstance inst() {
        return inst;
    }

    @Override
    public BlackmorePlot plot() {
        return plot;
    }

    @Override
    public pLinearStateUncertainty unc() {
        return unc;
    }

    @Override
    public oLinearObjective objective() {
        return objective;
    }

    @Override
    public oLinearDynamic dynamic() {
        return dynamic;
    }

    @Override
    public int Nsteps() {
        return Nsteps;
    }

    @Override
    public void addObjective(BlackmoreModel model) throws Exception {
        objective.addObjective(this, model);
    }

    @Override
    public oLinearVehicle vehicle() {
        return vehicle;
    }


}
