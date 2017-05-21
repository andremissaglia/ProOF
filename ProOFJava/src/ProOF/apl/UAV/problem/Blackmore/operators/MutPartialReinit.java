package ProOF.apl.UAV.problem.Blackmore.operators;

import ProOF.apl.UAV.problem.Blackmore.BlackmoreCodification;
import ProOF.apl.UAV.problem.Blackmore.BlackmoreProblem;
import ProOF.apl.UAV.problem.Blackmore.GraphVertex;
import ProOF.gen.operator.oMutation;
import ProOF.utilities.uRouletteList;

public class MutPartialReinit extends oMutation<BlackmoreProblem, BlackmoreCodification> {

    @Override
    public String name() {
        return "Mut-Partial-Reinit";
    }

    @Override
    public void mutation(BlackmoreProblem prob, BlackmoreCodification ind) throws Exception {
        mutPartialReinit(prob, ind);
    }
    public static void mutPartialReinit(BlackmoreProblem prob, BlackmoreCodification ind){
        ind.repair(prob);
        int N = prob.map.graph.vertexes.length;
        int W = prob.Waypoints();
        int source = prob.map.graph.source().id;
        int target = prob.map.graph.target().id;
        int cut = prob.rnd.nextInt(ind.path.size());
        ind.path.subList(cut, ind.path.size()).clear();
        int current = source;
        if(ind.path.size() > 0) {
            ind.path.get(ind.path.size()-1);
        }
        uRouletteList roulette = new uRouletteList(prob.rnd);
        boolean visited[] = new boolean[prob.map.graph.vertexes.length];
        java.util.Arrays.fill(visited, false);
        ind.path.forEach((i)-> {visited[i] = true;});
        
        while(current != target){
            roulette.clear();
            boolean dead_end = true;
            boolean toTarget = false;
            for (GraphVertex v : prob.map.graph.vertexes[current].adj) {
                if (!visited[v.id]) {
                    if (v.id == target) {
                        toTarget = true;
                        break;
                    }
                    roulette.add(1.0 / v.costToTarget, v.id);
                    dead_end = false;
                }
            }

            if (dead_end || toTarget || ind.path.size() == W) {
                break;
            }
            // Select a candidate by roulette
            current = roulette.roulette_wheel();
            ind.path.add(current);

            visited[current] = true;
        }
    }
}