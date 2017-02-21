package ProOF.apl.UAV.problem.Blackmore;

import ProOF.com.Linker.LinkerResults;
import ProOF.opt.abst.problem.meta.objective.SingleObjective;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 *
 * @author marcio
 */
public class BlackmoreObjective extends SingleObjective<BlackmoreProblem, BlackmoreCodification, BlackmoreObjective> {
    private final LinkedHashMap<Integer, Double> hash = new LinkedHashMap(100000, 0.75f, true){
        public static final int MAX_SIZE=75000;
        @Override
        protected boolean removeEldestEntry(Map.Entry entry) {
            return size() > MAX_SIZE;
        }
        
    };
    
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
            fitness += Math.min(cost, 1e3);
            i = j;
        }
        double cost = prob.map.graph.vertexes[i].costTo(prob.map.graph.vertexes[1]);    //target
        fitness += Math.min(cost, 1e3);
        
        if(fitness<50){
            int this_hash = codif.path.hashCode();
            if(!hash.containsKey(this_hash)){
                prob.map.fix(prob, prob.inst, prob.unc, prob.model.delta_to_cut, codif.weights, codif.graph_path(prob), prob.model, false);
                if(prob.model.solve(30, 1e-4, 1, false, false)){
                    fitness = prob.model.upper();
                }else{
                    fitness += 100;
                }
                hash.put(this_hash,fitness);
            }else{
                fitness = hash.get(this_hash);
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
        super.results(prob, link, codif);
        prob.model.results(link);
    }
    
    
}
