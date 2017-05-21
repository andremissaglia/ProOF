package ProOF.apl.UAV.problem.Blackmore.operators;

import ProOF.apl.UAV.problem.Blackmore.BlackmoreProblem;

public class TrailCij extends TrailPheromone {

    @Override
    public String name() {
        return "Trail <-- 1/Cij";
    }

    @Override
    protected void fill(BlackmoreProblem prob) {
        pheromones.forEach((e) -> {
            e.setPheromone(1.0 / Cij[e.getFrom()][e.getTo()]);
            for (int k = 0; k < W; k++) {
                e.setWeight(k, 1.0);
            }
        });
    }
}
