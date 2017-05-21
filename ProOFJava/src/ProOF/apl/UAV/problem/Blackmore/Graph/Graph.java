package ProOF.apl.UAV.problem.Blackmore.Graph;

import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.stream.Stream;

/**
 *
 * @author andre
 */
public class Graph {
    private ArrayList<ArrayList<Edge>> vertexes;
    private final int N;
    private final int W;
    public Graph(int N, int W) {
        this.N = N;
        this.W = W;
        this.vertexes = new ArrayList<>();
        this.vertexes.ensureCapacity(N);
        for(int i = 0; i < N; i++){
            this.vertexes.add(new ArrayList<>());
        }
    }
    public Edge addEdge(int from, int to){
        Edge edge = new Edge(from, to);
        this.vertexes.get(from).add(edge);
        return edge;
    }
    public Edge get(int from, int to){
        for(Edge e : this.vertexes.get(from)){
            if(e.to == to) return e;
        }
        return null;
    }
    public void forEach(int from, Consumer<Edge> consumer){
        this.vertexes.get(from).forEach(e -> {
            consumer.accept(e);
        });
    }
    public void forEach(Consumer<Edge> consumer){
        this.vertexes.forEach(v-> {
            v.forEach(e -> {
                consumer.accept(e);
            });
        });
                
    }
    public ArrayList<Edge> getEdges(int from){
        return this.vertexes.get(from);
    }
    public class Edge{
        private final int from;
        private final int to;
        private double pheromone;
        private double weights[];

        public Edge(int from, int to) {
            this.from = from;
            this.to = to;
            this.weights = new double[W];
        }

        public int getFrom() {
            return from;
        }

        public int getTo() {
            return to;
        }

        public double getPheromone() {
            return pheromone;
        }

        public void setPheromone(double pheromone) {
            this.pheromone = pheromone;
        }

        public double getWeight(int k) {
            return weights[k];
        }

        public void setWeight(int k, double weight) {
            weights[k] = weight;
        }
        
        
    }
}
