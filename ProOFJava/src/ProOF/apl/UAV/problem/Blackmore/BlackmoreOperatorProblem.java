/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.problem.Blackmore;

import ProOF.apl.UAV.problem.Blackmore.operators.MovExchange;
import ProOF.apl.UAV.problem.Blackmore.operators.MutExchange;
import ProOF.apl.UAV.problem.Blackmore.operators.RandomBestFirst;
import ProOF.apl.UAV.problem.Blackmore.operators.RandomTour;
import ProOF.apl.UAV.problem.Blackmore.operators.TrailCij;
import ProOF.apl.UAV.problem.Blackmore.operators.TrailConst;
import ProOF.apl.UAV.problem.Blackmore.operators.TrailHeuristic;
import ProOF.apl.UAV.problem.Blackmore.operators.TrailRnd;
import ProOF.apl.UAV.problem.Blackmore.operators.Uniform;
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
            case 2: return new Uniform();       // Crossover
            case 3: return new MovExchange();   // Local movement
            case 4: return new TrailConst();    // Pheromone Trail
            case 5: return new TrailRnd();      // Pheromone Trail
            case 6: return new TrailCij();      // Pheromone Trail
            case 7: return new TrailHeuristic();// Pheromone Trail
            case 8: return new RandomBestFirst();// Pheromone Trail
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
