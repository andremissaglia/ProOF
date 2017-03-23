package ProOF.apl.UAV.problem.Blackmore.operators;

import ProOF.apl.UAV.problem.Blackmore.BlackmoreCodification;
import ProOF.apl.UAV.problem.Blackmore.BlackmoreProblem;
import ProOF.gen.operator.oCrossover;

public class Uniform extends oCrossover<BlackmoreProblem, BlackmoreCodification> {

    @Override
    public String name() {
        return "Uniform";
    }

    @Override
    public BlackmoreCodification crossover(BlackmoreProblem prob, BlackmoreCodification ind1, BlackmoreCodification ind2) throws Exception {
        BlackmoreCodification child = ind1.build(prob);

        int min_size = Math.min(ind1.path.size(), ind2.path.size());
        int max_size = Math.max(ind1.path.size(), ind2.path.size());

        for (int i = 0; i < min_size; i++) {
            if (prob.rnd.nextBoolean()) {
                child.path.add(ind1.path.get(i));
            } else {
                child.path.add(ind2.path.get(i));
            }
        }
        for (int i = min_size; i < max_size; i++) {
            if (prob.rnd.nextBoolean()) {
                if (i < ind1.path.size()) {
                    child.path.add(ind1.path.get(i));
                } else {
                    child.path.add(ind2.path.get(i));
                }
            }
        }
        return child;
    }
}
