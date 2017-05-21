package ProOF.apl.UAV.problem.Blackmore.operators;

import ProOF.apl.UAV.problem.Blackmore.BlackmoreCodification;
import ProOF.apl.UAV.problem.Blackmore.BlackmoreProblem;
import ProOF.gen.operator.oMutation;

public class MutDelete extends oMutation<BlackmoreProblem, BlackmoreCodification> {

    @Override
    public String name() {
        return "Mut-Delete";
    }

    @Override
    public void mutation(BlackmoreProblem prob, BlackmoreCodification ind) throws Exception {
        int idx = prob.rnd.nextInt(ind.path.size());
        ind.path.remove(idx);
    }
}