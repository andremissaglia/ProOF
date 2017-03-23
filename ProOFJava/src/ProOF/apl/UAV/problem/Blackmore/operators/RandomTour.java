package ProOF.apl.UAV.problem.Blackmore.operators;

import ProOF.apl.UAV.problem.Blackmore.BlackmoreCodification;
import ProOF.apl.UAV.problem.Blackmore.BlackmoreProblem;
import ProOF.gen.operator.oInitialization;

public class RandomTour extends oInitialization<BlackmoreProblem, BlackmoreCodification> {

    @Override
    public String name() {
        return "Random Tour";
    }

    @Override
    public void initialize(BlackmoreProblem prob, BlackmoreCodification ind) throws Exception {
        ind.path.clear();
        int size = prob.rnd.nextInt(prob.map.graph.vertexes.length + 1);
        for (int i = 0; i < size; i++) {
            ind.path.add(prob.rnd.nextInt(prob.map.graph.vertexes.length));
        }
    }
}
