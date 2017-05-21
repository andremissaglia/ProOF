package ProOF.apl.UAV.problem.Blackmore.operators;

import ProOF.apl.UAV.problem.Blackmore.BlackmoreCodification;
import ProOF.apl.UAV.problem.Blackmore.BlackmoreProblem;
import ProOF.gen.operator.oLocalMove;

public class MovInsert extends oLocalMove<BlackmoreProblem, BlackmoreCodification> {

    @Override
    public String name() {
        return "Mov-Insert";
    }

    @Override
    public void local_search(BlackmoreProblem prob, BlackmoreCodification ind) throws Exception {
        MutInsert.mutInsert(prob, ind);
    }
}
