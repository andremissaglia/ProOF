package ProOF.apl.UAV.problem.Blackmore.operators;

import ProOF.apl.UAV.problem.Blackmore.BlackmoreCodification;
import ProOF.apl.UAV.problem.Blackmore.BlackmoreProblem;
import ProOF.apl.UAV.problem.Blackmore.GraphVertex;
import ProOF.gen.operator.oMutation;
import java.util.ArrayList;

public class MutInsert extends oMutation<BlackmoreProblem, BlackmoreCodification> {

    @Override
    public String name() {
        return "Mut-Insert";
    }

    @Override
    public void mutation(BlackmoreProblem prob, BlackmoreCodification ind) throws Exception {
        mutInsert(prob, ind);
    }
    public static ArrayList<Integer> getCommon(ArrayList<GraphVertex> a, ArrayList<GraphVertex> b, int N){
        ArrayList<Integer> common = new ArrayList<>();
        boolean used[] = new boolean[N];
        java.util.Arrays.fill(used, false);
        
        a.forEach((v) -> {
            used[v.id] = true;
        });
        b.stream()
            .filter((v) -> (used[v.id]))
            .forEach((v) -> {
                common.add(v.id);
            });
        return common;
    }
    public static void mutInsert(BlackmoreProblem prob, BlackmoreCodification ind){
        ind.repair(prob);
        int N = prob.map.graph.vertexes.length;
        int source = prob.map.graph.source().id;
        int target = prob.map.graph.target().id;
        int pathLength = ind.path.size();
        int count = pathLength + 1;
        int fromIdx = prob.rnd.nextInt(count) - 1;
        while(count-- > 0){
            int from = fromIdx >= 0 ? ind.path.get(fromIdx) : source;
            int to = fromIdx + 1 < pathLength ? ind.path.get(fromIdx+1) : target;
            ArrayList<Integer> common = getCommon(prob.map.graph.vertexes[from].adj, prob.map.graph.vertexes[to].adj, N);
            if(!common.isEmpty()){
                int newVertex = common.get(prob.rnd.nextInt(common.size()));
                ind.path.add(fromIdx + 1, newVertex);
                break;
            }
        }
    }
}