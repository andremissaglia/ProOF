package ProOF.apl.UAV.problem.Blackmore.operators;

import ProOF.apl.UAV.problem.Blackmore.BlackmoreCodification;
import ProOF.apl.UAV.problem.Blackmore.BlackmoreProblem;
import ProOF.apl.UAV.problem.Blackmore.GraphVertex;
import ProOF.com.Linker.LinkerResults;
import ProOF.gen.operator.oMutation;
import java.util.ArrayList;

public class MutDelete2 extends oMutation<BlackmoreProblem, BlackmoreCodification> {
    private static int iguais = 0;
    private static int muts = 0;
    @Override
    public String name() {
        return "Mut-Delete2";
    }

    @Override
    public void mutation(BlackmoreProblem prob, BlackmoreCodification ind) throws Exception {
        muts++;
        ind.repair(prob);
        int N = prob.map.graph.vertexes.length;
        int source = prob.map.graph.source().id;
        int target = prob.map.graph.target().id;
        int pathLength = ind.path.size();
        int count = pathLength;
        int fromIdx = prob.rnd.nextInt(count);
        while(count-- > 0){
            int from = fromIdx > 0 ? ind.path.get(fromIdx-1) : source;
            int to = fromIdx + 1 < pathLength ? ind.path.get(fromIdx+1) : target;
            if(in(to, prob.map.graph.vertexes[from].adj)){
                ind.path.remove(fromIdx);
                return;
            }
        }
        iguais++;
    }
    public boolean in(int x, ArrayList<GraphVertex> adj){
        return (adj.stream().anyMatch((v) -> (v.id == x)));
    }

    @Override
    public void results(LinkerResults link) throws Exception {
        super.results(link); 
        link.writeInt("Delete2-equals", iguais);
        link.writeInt("Delete2-muts", muts);
    }
    
}