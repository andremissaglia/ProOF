/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.problem.Blackmore;

import ProOF.apl.UAV.problem.Blackmore.Graph.Graph;
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
import java.util.ArrayList;
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
    
    private class RandomBestFirst extends oInitialization<BlackmoreProblem, BlackmoreCodification> {
        @Override
        public String name() {
            return "Random Best First";
        }
        
        @Override
        public void initialize(BlackmoreProblem prob, BlackmoreCodification ind) throws Exception {
            int source = prob.map.graph.source().id;
            int target = prob.map.graph.target().id;
            int W = prob.Waypoints();
            
            // List visited nodes
            boolean visited[] = new boolean[prob.map.graph.vertexes.length];
            java.util.Arrays.fill(visited, false);

            // First vertex is chosen
            ind.path.clear();
            ind.path.add(source);
            
            visited[source] = true;
            int current = source;

            uRouletteList roulette = new uRouletteList(prob.rnd);
            
            // Build step by step a path following pheromone trail
            while (current != target) {
                roulette.clear();
                boolean dead_end = true;
                boolean toTarget = false;
                for(GraphVertex v : prob.map.graph.vertexes[current].adj){
                    if (!visited[v.id]) {
                        if(v.id == target){
                            toTarget = true;
                            break;
                        }
                        roulette.add(1.0/ v.costToTarget, v.id);
                        dead_end = false;
                    }
                }
                
                if (dead_end || toTarget || ind.path.size() == W) {
                    ind.path.add(target);
                    break;
                }
                // Select a candidate by roulette
                current = roulette.roulette_wheel();
                ind.path.add(current);
                
                visited[current] = true;
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
        protected Graph pheromones;
        protected double Cij[][];   //Distance matrix
        protected double distanceToTarget[];
        protected int N;
        protected int W;
        private boolean evolveWeights;
        
        @Override
        public void parameters(LinkerParameters link) throws Exception {
            decay_rate = link.Dbl("decay rate", 0.5, 0.0, 1.0);
            evolveWeights = link.Bool("Evolve Weights", false);
        }

        protected abstract void fill(BlackmoreProblem prob);

        @Override
        public void initialize(BlackmoreProblem prob) throws Exception {
            this.N = prob.map.graph.vertexes.length;
            this.W = prob.Waypoints() / 2;
            this.pheromones = new Graph(N, W);
            Cij = new double[N][N];
            for (int i = 0; i < N; i++) {
                for (int j = 0; j < N; j++) {
                    Cij[i][j] = prob.map.graph.vertexes[i].costTo(prob.map.graph.vertexes[j]);
                }
            }
            distanceToTarget = new double[N];
            for(int i = 0; i < N; i++){
                //distanceToTarget[i] = prob.map.graph.vertexes[i].costTo(prob.map.graph.vertexes[prob.map.graph.target().id]);
                distanceToTarget[i] = prob.map.graph.vertexes[i].costToTarget;
            }
            for(GraphVertex v1: prob.map.graph.vertexes){
                v1.adj.forEach((v2) -> {
                    pheromones.addEdge(v1.id, v2.id);
                });
            }
            fill(prob);
            // Normalizes to [0,1]
            double max = 0.0;
            for(int v = 0; v < W; v++){
                for(Graph.Edge e : pheromones.getEdges(v)){
                    max = uUtil.maxDbl(max, e.getPheromone());
                }
            }
            for(int v = 0; v < W; v++){
                for(Graph.Edge e : pheromones.getEdges(v)){
                    e.setPheromone(e.getPheromone()/max);
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
            ant.path.clear();
            ant.path.ensureCapacity(N);
            ant.path.add(source);
            
            if(evolveWeights){
                if (ant.weights == null){
                    ant.weights = new ArrayList<>();
                    
                }
                ant.weights.clear();
                ant.weights.ensureCapacity(N-1);                
            } else {
                ant.weights = null;
            }
            
            visited[source] = true;
            int current = source;

            uRouletteList roulette = new uRouletteList(prob.rnd);

            double probability = 1.0;
            
            // Build step by step a path following pheromone trail
            while (current != target) {
                roulette.clear();
                boolean dead_end = true;
                for(Graph.Edge e: pheromones.getEdges(current)){
                    if (!visited[e.getTo()]) {
                        roulette.add(e.getPheromone(), e.getTo());
                        dead_end = false;
                    }
                }
                if (dead_end || ant.path.size() == this.W) {
                    // Builds an infeasible path, which will be penalized
                    ant.path.add(target);
                    
                    if(evolveWeights){
                        probability *= chooseWeight(ant, roulette, current, target);
                    }
                    break;
                }
                // Select a candidate by roulette
                int next = roulette.roulette_wheel();
                probability *= roulette.probability(next);
                ant.path.add(next);
                
                // Choose weight
                if(evolveWeights){
                    probability *= chooseWeight(ant, roulette, current, next);
                }
                
                // Update the visited 
                current = next;
                visited[current] = true;
            }
            return probability;
        }
        private double chooseWeight(BlackmoreCodification ant, uRouletteList roulette, int i, int j){
            roulette.clear();
            Graph.Edge e = pheromones.get(i, j);
            if(e == null){
                ant.weights.add(0.0);
                return 1;
            }
            for(int k = 0; k < W; k++){
                roulette.add(e.getWeight(k), k);
            }
            int weight = roulette.roulette_wheel();
            
            ant.weights.add((double)weight);
            return roulette.probability(weight);
        }
        @Override
        public void evaporate(BlackmoreProblem prob) throws Exception {
            pheromones.forEach((e) -> {
                e.setPheromone((1 - decay_rate) * e.getPheromone());
                if(evolveWeights){
                    for(int k = 0; k < W; k++){
                        e.setWeight(k, (1 - decay_rate) *e.getWeight(k));
                    }
                }
            });
        }

        @Override
        public void deposit(BlackmoreProblem prob, BlackmoreCodification ant, BlackmoreObjective obj, double weight) throws Exception {
            for (int n = 1; n < ant.path.size(); n++) {
                int i = ant.path.get(n - 1);
                int j = ant.path.get(n);
                Graph.Edge e = pheromones.get(i, j);
                double update_value = weight / obj.abs_value();
                e.setPheromone(e.getPheromone()+update_value);
                
                // update weights phromones
                if(evolveWeights){
                    int k = (int) ant.weights.get(n-1).doubleValue();
                    e.setWeight(k, e.getWeight(k)+update_value);
                }                
            }
        }
    }

    private class TrailConst extends TrailPheromone {

        @Override
        protected void fill(BlackmoreProblem prob) {
            pheromones.forEach((e) -> {
                e.setPheromone(1.0);
                for(int k = 0; k < W; k++){
                    e.setWeight(k, 1.0);
                }
            });
        }
        
        @Override
        public String name() {
            return "Trail <-- 1/N";
        }

    }

    private class TrailRnd extends TrailPheromone {

        @Override
        protected void fill(BlackmoreProblem prob) {
            pheromones.forEach((e) -> {
                e.setPheromone(prob.rnd.nextDouble());
                for(int k = 0; k < W; k++){
                    e.setWeight(k, prob.rnd.nextDouble());
                }
            });
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
        protected void fill(BlackmoreProblem prob) {
            pheromones.forEach((e) -> {
                e.setPheromone(1.0/ Cij[e.getFrom()][e.getTo()]);
                for(int k = 0; k < W; k++){
                    e.setWeight(k, 1.0);
                }
            });
        }
    }
    private class TrailHeuristic extends TrailPheromone {

        @Override
        public String name() {
            return "Trail <-- 1/(Cij + dtt)";
        }
        
        @Override
        protected void fill(BlackmoreProblem prob) {
            pheromones.forEach((e) -> {
                e.setPheromone(1.0/ (Cij[e.getFrom()][e.getTo()] + distanceToTarget[e.getTo()]));
                for(int k = 0; k < W; k++){
                    e.setWeight(k, 1.0);
                }
            });
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
