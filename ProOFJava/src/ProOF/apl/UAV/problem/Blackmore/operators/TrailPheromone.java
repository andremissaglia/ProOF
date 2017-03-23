package ProOF.apl.UAV.problem.Blackmore.operators;

import ProOF.apl.UAV.problem.Blackmore.BlackmoreCodification;
import ProOF.apl.UAV.problem.Blackmore.BlackmoreObjective;
import ProOF.apl.UAV.problem.Blackmore.BlackmoreProblem;
import ProOF.apl.UAV.problem.Blackmore.Graph.Graph;
import ProOF.apl.UAV.problem.Blackmore.GraphVertex;
import ProOF.com.Linker.LinkerParameters;
import ProOF.gen.operator.oTrailPheromone;
import ProOF.utilities.uRouletteList;
import ProOF.utilities.uUtil;
import java.util.ArrayList;

public abstract class TrailPheromone extends oTrailPheromone<BlackmoreProblem, BlackmoreCodification, BlackmoreObjective> {

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
        for (int i = 0; i < N; i++) {
            //distanceToTarget[i] = prob.map.graph.vertexes[i].costTo(prob.map.graph.vertexes[prob.map.graph.target().id]);
            distanceToTarget[i] = prob.map.graph.vertexes[i].costToTarget;
        }
        for (GraphVertex v1 : prob.map.graph.vertexes) {
            v1.adj.forEach((v2) -> {
                pheromones.addEdge(v1.id, v2.id);
            });
        }
        fill(prob);
        // Normalizes to [0,1]
        double max = 0.0;
        for (int v = 0; v < W; v++) {
            for (Graph.Edge e : pheromones.getEdges(v)) {
                max = uUtil.maxDbl(max, e.getPheromone());
            }
        }
        for (int v = 0; v < W; v++) {
            for (Graph.Edge e : pheromones.getEdges(v)) {
                e.setPheromone(e.getPheromone() / max);
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

        if (evolveWeights) {
            if (ant.weights == null) {
                ant.weights = new ArrayList<>();

            }
            ant.weights.clear();
            ant.weights.ensureCapacity(N - 1);
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
            for (Graph.Edge e : pheromones.getEdges(current)) {
                if (!visited[e.getTo()]) {
                    roulette.add(e.getPheromone(), e.getTo());
                    dead_end = false;
                }
            }
            if (dead_end || ant.path.size() == this.W) {
                // Builds an infeasible path, which will be penalized
                ant.path.add(target);

                if (evolveWeights) {
                    probability *= chooseWeight(ant, roulette, current, target);
                }
                break;
            }
            // Select a candidate by roulette
            int next = roulette.roulette_wheel();
            probability *= roulette.probability(next);
            ant.path.add(next);

            // Choose weight
            if (evolveWeights) {
                probability *= chooseWeight(ant, roulette, current, next);
            }

            // Update the visited 
            current = next;
            visited[current] = true;
        }
        return probability;
    }

    private double chooseWeight(BlackmoreCodification ant, uRouletteList roulette, int i, int j) {
        roulette.clear();
        Graph.Edge e = pheromones.get(i, j);
        if (e == null) {
            ant.weights.add(0.0);
            return 1;
        }
        for (int k = 0; k < W; k++) {
            roulette.add(e.getWeight(k), k);
        }
        int weight = roulette.roulette_wheel();

        ant.weights.add((double) weight);
        return roulette.probability(weight);
    }

    @Override
    public void evaporate(BlackmoreProblem prob) throws Exception {
        pheromones.forEach((e) -> {
            e.setPheromone((1 - decay_rate) * e.getPheromone());
            if (evolveWeights) {
                for (int k = 0; k < W; k++) {
                    e.setWeight(k, (1 - decay_rate) * e.getWeight(k));
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
            e.setPheromone(e.getPheromone() + update_value);

            // update weights phromones
            if (evolveWeights) {
                int k = (int) ant.weights.get(n - 1).doubleValue();
                e.setWeight(k, e.getWeight(k) + update_value);
            }
        }
    }
}
