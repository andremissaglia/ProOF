package ProOF.apl.UAV.problem.Blackmore.operators;

import ProOF.apl.UAV.problem.Blackmore.BlackmoreCodification;
import ProOF.apl.UAV.problem.Blackmore.BlackmoreProblem;
import ProOF.gen.operator.oCrossover;
import java.util.ArrayList;

public class CutCrossover extends oCrossover<BlackmoreProblem, BlackmoreCodification> {

    @Override
    public String name() {
        return "Cut";
    }

    @Override
    @SuppressWarnings("empty-statement")
    public BlackmoreCodification crossover(BlackmoreProblem prob, BlackmoreCodification ind1, BlackmoreCodification ind2) throws Exception {
        BlackmoreCodification child = ind1.build(prob);
        boolean used[] = new boolean[prob.map.graph.vertexes.length];
        java.util.Arrays.fill(used, false);

        // find common vertexes
        ind1.path.forEach((i) -> {
            used[i] = true;
        });
        ArrayList<Integer> common = new ArrayList<>();
        ind2.path.forEach((i) -> {
            if (used[i]) {
                common.add(i);
            }
        });

        if (common.isEmpty()) {
            if (prob.rnd.nextBoolean()) {
                child.copy(prob, ind1);
            } else {
                child.copy(prob, ind2);
            }
            return child;
        }
        // choose a cut point
        int cut = common.get(prob.rnd.nextInt(common.size()));

        // build child
        int i, v;
        for (i = 0; i < ind1.path.size(); i++) {
            v = ind1.path.get(i);
            if (v == cut) {
                break;
            }
            child.path.add(v);

        }
        for (i = 0; ind2.path.get(i) != cut; i++);
        for (; i < ind2.path.size(); i++) {
            v = ind2.path.get(i);
            child.path.add(v);
        }

        return child;
    }
}
