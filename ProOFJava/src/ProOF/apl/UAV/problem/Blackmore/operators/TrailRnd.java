package ProOF.apl.UAV.problem.Blackmore.operators;

import ProOF.apl.UAV.problem.Blackmore.BlackmoreProblem;

public class TrailRnd extends TrailPheromone {

    @Override
    protected void fill(BlackmoreProblem prob) {
        pheromones.forEach((e) -> {
            e.setPheromone(prob.rnd.nextDouble());
            for (int k = 0; k < W; k++) {
                e.setWeight(k, prob.rnd.nextDouble());
            }
        });
    }

    @Override
    public String name() {
        return "Trail <-- random";
    }

}
