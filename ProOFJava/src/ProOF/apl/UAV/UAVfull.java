/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV;
import ProOF.apl.UAV.abst.UAVApproach;
import ProOF.apl.UAV.abst.UAVModel;
import ProOF.com.Linker.LinkerApproaches;
import ProOF.CplexOpt.CplexFull;
import ProOF.com.Linker.LinkerResults;
import ProOF.utilities.uTime;
import ProOF.utilities.uTimeMilli;
import ilog.concert.IloException;

/**
 *
 * @author marcio
 */
public class UAVfull extends CplexFull{
    private UAVApproach approach;
    private oFull full_op;
    private UAVModel model;
    
    private final uTime elapsed_time = new uTimeMilli();
    private double total_time;
    public UAVfull() throws IloException {
        
    }
    
    @Override
    public void services(LinkerApproaches link) throws Exception {
        super.services(link);
        approach = link.get(fUAVApproach.obj, approach);
        full_op = link.need(oFull.class, full_op);
    }
    @Override
    public String name() {
        return "UAVfull";
    }
    @Override
    public void model() throws Exception {
        model = full_op.build_model(approach, cpx);
        //cpx.exportModel("./"+name()+".lp");
    }
    @Override
    public void solve() throws Exception{
        elapsed_time.start();
        model();
        if(model.solve()){
            print();
        }
        total_time = elapsed_time.time();
    }
    @Override
    public void results(LinkerResults link) throws Exception {
        //super.results(link); //This results are obsolet if its use callback im model
        link.writeDbl("method-time", total_time);
        if(model.isFeasible()){
            link.writeDbl("method-objective", model.upper());
            link.writeDbl("method-lower", model.lower());
        }
        model.results(link);
        model.save();
    }
}
