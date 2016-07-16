/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.problem.Blackmore;

import ProOF.com.Linker.LinkerParameters;
import ProOF.com.language.Factory;
import ProOF.gen.operator.oCrossover;
import ProOF.gen.operator.oInitialization;
import ProOF.gen.operator.oLocalMove;
import ProOF.gen.operator.oMutation;
import ProOF.gen.operator.oTrailPheromone;
import ProOF.opt.abst.problem.meta.codification.Operator;
import ProOF.utilities.uRouletteList;
import ProOF.utilities.uUtil;
import java.util.Random;
import jsc.util.Arrays;

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
        }
        return null;
    }

    private class RandomTour extends oInitialization<BlackmoreProblem, BlackmoreCodification> {

        @Override
        public String name() {
            return "Random Tour";
        }

        @Override
        public void initialize(BlackmoreProblem prob, BlackmoreCodification ind) throws Exception {
            ind.path.clear();
            int size = prob.rnd.nextInt(prob.map.graph.vertexes.length + 1);
            for (int i = 0; i < size; i++) {
                ind.path.add(prob.rnd.nextInt(prob.map.graph.vertexes.length));
            }
        }
    }

    private class MutExchange extends oMutation<BlackmoreProblem, BlackmoreCodification> {

        @Override
        public String name() {
            return "Mut-Exchange";
        }

        @Override
        public void mutation(BlackmoreProblem prob, BlackmoreCodification ind) throws Exception {
            random_swap(prob.rnd, ind);
        }
    }

    private class MovExchange extends oLocalMove<BlackmoreProblem, BlackmoreCodification> {

        @Override
        public String name() {
            return "Mov-Exchange";
        }

        @Override
        public void local_search(BlackmoreProblem prob, BlackmoreCodification ind) throws Exception {
            random_swap(prob.rnd, ind);
        }
    }

    private class Uniform extends oCrossover<BlackmoreProblem, BlackmoreCodification> {

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

    private abstract class TrailPheromone extends oTrailPheromone<BlackmoreProblem, BlackmoreCodification, BlackmoreObjective> {

        private double decay_rate;  //decay rate of the pheromone
        private double Tij[][];     //pheromone trail
        protected double Cij[][];   //Distance matrix
        protected int N;

        @Override
        public void parameters(LinkerParameters link) throws Exception {
            decay_rate = link.Dbl("decay rate", 0.5, 0.0, 1.0);
        }

        protected abstract void fill(BlackmoreProblem prob, double Tij[][]);

        @Override
        public void initialize(BlackmoreProblem prob) throws Exception {
            this.N = prob.map.graph.vertexes.length;
            Tij = new double[N][N];
            Cij = new double[N][N];
            for (int i = 0; i < N; i++) {
                for (int j = 0; j < N; j++) {
                    Cij[i][j] = prob.map.graph.vertexes[i].costTo(prob.map.graph.vertexes[j]);
                }
            }
            fill(prob, Tij);
            // Normalizes to [0,1]
            double max = 0;
            for (double array[] : Tij) {
                max = uUtil.maxDbl(max, Arrays.max(array));
            }
            for (int i = 0; i < N; i++) {
                for (int j = 0; j < N; j++) {
                    if (i != j) {
                        Tij[i][j] /= max;
                    }
                }
            }

//            System.out.println("----------------- initialize ["+this.name()+"] --------------------");
//            uUtil.Print(System.out, "%10.6f", Tij);
        }

        @Override
        public double build(BlackmoreProblem prob, BlackmoreCodification ant) throws Exception {
            int source = prob.map.graph.source().id;
            int target = prob.map.graph.target().id;

            // List visited nodes
            boolean visited[] = new boolean[N];
            java.util.Arrays.fill(visited, false);

            // First vertex is chosen
            ant.path.ensureCapacity(N);
            ant.path.add(source);
            visited[source] = true;
            int current = source;

            uRouletteList roulette = new uRouletteList(prob.rnd);

            double probability = 1.0;

            // Build step by step a path following pheromone trail
            while (current != target) {
                roulette.clear();
                boolean dead_end = true;
                for (GraphVertex j : prob.map.graph.vertexes[current].adj) {
                    if (!visited[j.id]) {
                        roulette.add(Tij[current][j.id], j.id);
                        dead_end = false;
                    }
                }
                if (dead_end) {
                    // Builds an infeasible path, which will be penalized
                    ant.path.add(target);
                    break;
                }
                // Select a candidate by roulette
                current = roulette.roulette_wheel();
                probability *= roulette.probability(current);
                ant.path.add(current);
                // Update the visited list
                visited[current] = true;
            }
            return probability;
        }

        @Override
        public void evaporate(BlackmoreProblem prob) throws Exception {
            for (int i = 0; i < N; i++) {
                for (int j = 0; j < N; j++) {
                    Tij[i][j] = (1 - decay_rate) * Tij[i][j];
                }
            }
        }

        @Override
        public void deposit(BlackmoreProblem prob, BlackmoreCodification ant, BlackmoreObjective obj, double weight) throws Exception {
            for (int n = 1; n < ant.path.size(); n++) {
                int i = ant.path.get(n - 1);
                int j = ant.path.get(n);
                Tij[i][j] += weight / obj.abs_value();
            }
        }
    }

    private class TrailConst extends TrailPheromone {

        @Override
        protected void fill(BlackmoreProblem prob, double[][] Tij) {
            for (int i = 0; i < N; i++) {
                for (int j = 0; j < N; j++) {
                    if (i != j) {
                        Tij[i][j] = 1.0;
                    }
                }
            }
        }

        @Override
        public String name() {
            return "Trail <-- 1/N";
        }

    }

    private class TrailRnd extends TrailPheromone {

        @Override
        protected void fill(BlackmoreProblem prob, double[][] Tij) {
            for (int i = 0; i < N; i++) {
                for (int j = 0; j < N; j++) {
                    if (i != j) {
                        Tij[i][j] = prob.rnd.nextDouble();
                    }
                }
            }
        }

        @Override
        public String name() {
            return "Trail <-- random";
        }

    }

    private class TrailCij extends TrailPheromone {

        @Override
        public String name() {
            return "Trail <-- 1/Cij";
        }

        @Override
        protected void fill(BlackmoreProblem prob, double[][] Tij) {
            for (int i = 0; i < N; i++) {
                for (int j = 0; j < N; j++) {
                    if (i != j) {
                        Tij[i][j] = 1.0 / Cij[i][j];
                    }
                }
            }
        }
    }

    private static void random_swap(Random rmd, BlackmoreCodification ind) {
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
