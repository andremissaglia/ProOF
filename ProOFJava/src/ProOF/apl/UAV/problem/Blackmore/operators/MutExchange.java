package ProOF.apl.UAV.problem.Blackmore.operators;

import ProOF.apl.UAV.problem.Blackmore.BlackmoreCodification;
import ProOF.apl.UAV.problem.Blackmore.BlackmoreOperatorProblem;
import ProOF.apl.UAV.problem.Blackmore.BlackmoreProblem;
import ProOF.gen.operator.oMutation;

public class MutExchange extends oMutation<BlackmoreProblem, BlackmoreCodification> {

    @Override
    public String name() {
        return "Mut-Exchange";
    }

    @Override
    public void mutation(BlackmoreProblem prob, BlackmoreCodification ind) throws Exception {
        BlackmoreOperatorProblem.random_swap(prob.rnd, ind);
    }
}