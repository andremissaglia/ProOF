package ProOF.apl.UAV.problem.Blackmore.operators;

import ProOF.apl.UAV.problem.Blackmore.BlackmoreCodification;
import ProOF.apl.UAV.problem.Blackmore.BlackmoreProblem;
import ProOF.gen.operator.oLocalMove;

public class MovPartialReinit extends oLocalMove<BlackmoreProblem, BlackmoreCodification> {

    @Override
    public String name() {
        return "Mov-Partial-Reinit";
    }

    @Override
    public void local_search(BlackmoreProblem prob, BlackmoreCodification ind) throws Exception {
        MutPartialReinit.mutPartialReinit(prob, ind);
    }
}
