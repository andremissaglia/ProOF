package ProOF.apl.UAV.problem.Blackmore;

import ProOF.apl.UAV.problem.Blackmore.operators.*;
import ProOF.com.language.Factory;
import ProOF.opt.abst.problem.meta.codification.Operator;
import java.util.Random;

/**
 *
 * @author marcio
 */
public class BlackmoreOperatorProblem extends Factory<Operator> {

    public static final BlackmoreOperatorProblem obj = new BlackmoreOperatorProblem();

    @Override
    public String name() {
        return "BlackmoreOperatorProblem";
    }

    @Override
    public Operator build(int index) {  // Build the operators
        switch (index) {
            case 0: return new RandomTour();    // Initialization
            case 1: return new MutExchange();   // Mutation
            case 2: return new UniformCrossover();       // Crossover
            case 3: return new MovExchange();   // Local movement
            case 4: return new TrailConst();    // Pheromone Trail
            case 5: return new TrailRnd();      // Pheromone Trail
            case 6: return new TrailCij();      // Pheromone Trail
            case 7: return new TrailHeuristic();// Pheromone Trail
            case 8: return new RandomBestFirst();// Pheromone Trail
            case 9: return new CutCrossover();       // Crossover
            case 10: return new MutDelete();       // Mutation
            case 11: return new MutDelete2();       // Mutation
            case 12: return new MutInsert();       // Mutation
            case 13: return new MutPartialReinit(); // Mutation
            case 14: return new MovInsert();        // Local movement
            case 15: return new MovPartialReinit(); // Local movement
            case 16: return new KSP(); // Local movement
        }
        return null;
    }

    public static void random_swap(Random rmd, BlackmoreCodification ind) {
        if (ind.path.size() > 1) {
            int a = rmd.nextInt(ind.path.size());
            int b = rmd.nextInt(ind.path.size());
            int v1 = ind.path.get(a);
            int v2 = ind.path.get(b);
            ind.path.set(a, v2);
            ind.path.set(b, v1);
        }
    }
}
