package ProOF.apl.UAV.problem.Blackmore.operators;

import ProOF.apl.UAV.problem.Blackmore.BlackmoreCodification;
import static ProOF.apl.UAV.problem.Blackmore.BlackmoreOperatorProblem.random_swap;
import ProOF.apl.UAV.problem.Blackmore.BlackmoreProblem;
import ProOF.gen.operator.oLocalMove;

public class MovExchange extends oLocalMove<BlackmoreProblem, BlackmoreCodification> {

    @Override
    public String name() {
        return "Mov-Exchange";
    }

    @Override
    public void local_search(BlackmoreProblem prob, BlackmoreCodification ind) throws Exception {
        random_swap(prob.rnd, ind);
    }
}
