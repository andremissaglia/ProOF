/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV;
import ProOF.CplexExtended.CplexExtended;
import ProOF.apl.UAV.abst.UAVApproach;
import ProOF.apl.UAV.abst.UAVModel;
import ProOF.com.Linker.LinkerApproaches;
import ProOF.com.Linker.LinkerParameters;
import ProOF.com.Linker.LinkerResults;
import ProOF.opt.abst.run.Heuristic;
import ProOF.utilities.uTime;
import ProOF.utilities.uTimeMilli;

/**
 *
 * @author marcio
 */
public class UAVfullRebuild extends Heuristic{
    private UAVApproach approach;
    private oRebuild re_op;
    
    protected double execTime;
    protected double epGap;
    protected int threads;
    protected int nIterations;
    private boolean print_war;
    private boolean print_out;
    
    private UAVModel model;
    private final uTime elapsed_time = new uTimeMilli();
    
    private int iter;
    private double total_time;

    @Override
    public String name() {
        return "UAV re-build";
    }
    @Override
    public void services(LinkerApproaches link) throws Exception {
        approach = link.get(fUAVApproach.obj, approach);
        re_op = link.need(oRebuild.class, re_op);
    }
    @Override
    public void parameters(LinkerParameters link) throws Exception {
        execTime = link.Dbl("Time", 3600.0, 1.0, 180000.0);
        epGap = link.Dbl("Gap Rel", 0.0001, 0.0, 100.0);
        threads = link.Int("Threads", 1, 0, 16);
        nIterations = link.Int("nIterations", 40, 1, 100);
        //nIterations = 20;
        
        print_war = link.Bool("warning", false);
        print_out = link.Bool("output", true);
    }
    private double remaining_time(){
        return execTime-elapsed_time.time();
    }
    @Override
    public void execute() throws Exception {
        elapsed_time.start();
        model = re_op.rebuild_model(approach, new CplexExtended(), null);
        iter = 0;
        while(!model.solve(remaining_time()-10, epGap, threads, print_war, print_out) && iter<nIterations){
            //System.in.read();
            model.cplex.end();
            model = re_op.rebuild_model(approach, new CplexExtended(), model);
            iter++;
        }
        total_time = elapsed_time.time();
    }

    @Override
    public void results(LinkerResults link) throws Exception {
        link.writeDbl("method-time", total_time);
        if(model.isFeasible()){
            link.writeDbl("method-objective", model.upper());
            link.writeDbl("method-lower", model.lower());
        }
        link.writeDbl("method-iter", iter);
        model.results(link);
        if(model.isFeasible()){
            model.save();
        }
    }
}
