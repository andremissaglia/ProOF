/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.problem.Blackmore;

import ProOF.com.Linker.LinkerResults;
import ProOF.opt.abst.problem.meta.objective.SingleObjective;
import java.util.ArrayList;

/**
 *
 * @author marcio
 */
public class BlackmoreObjective extends SingleObjective<BlackmoreProblem, BlackmoreCodification, BlackmoreObjective> {
    private static ArrayList<Integer> hash = new ArrayList<Integer>();
    
    public BlackmoreObjective() throws Exception {
        super();
    }
    @Override
    public void evaluate(BlackmoreProblem prob, BlackmoreCodification codif) throws Exception {
        codif.repair(prob);
        
        
        
        double fitness = 0;
        int i = 0;    //source
        for(int j : codif.path){
            double cost = prob.map.graph.vertexes[i].costTo(prob.map.graph.vertexes[j]);
            if(cost==Double.POSITIVE_INFINITY){
                fitness += 1e3; //penality
            }else{
                fitness += cost;
            }
            i = j;
        }
        double cost = prob.map.graph.vertexes[i].costTo(prob.map.graph.vertexes[1]);    //target
        if(cost==Double.POSITIVE_INFINITY){
            fitness += 1e3; //penality
        }else{
            fitness += cost;
        }
        
        if(fitness<50){
            int this_hash = codif.path.hashCode();
            if(!hash.contains(this_hash)){
                prob.map.fix(prob, prob.inst, prob.unc, prob.model.delta_to_cut, codif.graph_path(prob), prob.model, false);
                if(prob.model.solve(30, 1e-4, 1, false, false)){
                    fitness = prob.model.upper();
                }else{
                    fitness += 100;
                }
                hash.add(this_hash);
                if(hash.size()>1000000){
                    hash.remove(0);
                }
                //System.out.println(hash);
            }else{
                fitness += 500;
            }
        }
        
        set(fitness);       //set de fitness to the ProOF
    }
    @Override
    public BlackmoreObjective build(BlackmoreProblem prob) throws Exception {
        return new BlackmoreObjective();
    }

    @Override
    public void results(BlackmoreProblem prob, LinkerResults link, BlackmoreCodification codif) throws Exception {
        hash.clear();
        evaluate(prob, codif);
        super.results(prob, link, codif); //To change body of generated methods, choose Tools | Templates.
        prob.model.results(link);
    }
    
    
}