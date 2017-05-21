package ProOF.apl.UAV.problem.Blackmore.Graph;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.PriorityQueue;
import java.util.Stack;
import java.util.function.Consumer;

/**
 * More generic Graph undirected graph implementation, with util methods.
 * @author andre
 */
public class Graph2 {
    private final ArrayList<ArrayList<Edge>> vertexes;
    private final int N;
    public Graph2(int N) {
        this.N = N;
        this.vertexes = new ArrayList<>();
        this.vertexes.ensureCapacity(N);
        for(int i = 0; i < N; i++){
            this.vertexes.add(new ArrayList<>());
        }
    }
    public Graph2(Graph2 original){
        this.N = original.N;
        this.vertexes = new ArrayList<>();
        this.vertexes.ensureCapacity(N);
        for(int i = 0; i < N; i++){
            this.vertexes.add(new ArrayList<>());
        }
        original.forEach((Edge e) -> {
            _addEdge(e.from, e.to, e.weight);
        });
    }
    public void addEdge(int from, int to, double weight){
        if(get(from,to) >= 0){
            remEdge(from, to);
        }
        _addEdge(from, to, weight);
        _addEdge(to, from, weight);
    }
    private void _addEdge(int from, int to, double weight){
        this.vertexes.get(from).add(new Edge(from, to, weight));
    }
    public void remEdge(int from, int to){
        _remEdge(from, to);
        _remEdge(to, from);
    }
    private void _remEdge(int from, int to){
        ArrayList v = vertexes.get(from);
        Iterator<Edge> it = v.iterator();
        while(it.hasNext()){
            Edge e = it.next();
            if(e.to == to){
                it.remove();
                break;
            }
        }
    }
    public void removeNode(int vertex){
        forEach(vertex, (Edge e)-> {
            _remEdge(e.to, vertex);
        });
        this.vertexes.get(vertex).clear();
    }
    public double get(int from, int to){
        for(Edge e : this.vertexes.get(from)){
            if(e.to == to) return e.weight;
        }
        return -1;
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
        public double weight;

        public Edge(int from, int to, double weight) {
            this.from = from;
            this.to = to;
            this.weight = weight;
        }

        public int getFrom() {
            return from;
        }

        public int getTo() {
            return to;
        }
    }
    public ArrayList<Integer> dijkstra(int source, int dest){
        int pi[] = new int[N];
        boolean visited[] = new boolean[N];
        double d[] = new double[N];
        
        for(int i = 0; i < N; i++){
            pi[i] = -1;
            d[i] = 1E6;
            visited[i] = false;
        }
        
        d[source] = 0;
        
        while(true){
            double mind = Double.POSITIVE_INFINITY;
            int v = -1;
            for(int i = 0; i < N; i++){
                if(!visited[i] && d[i] < mind){
                    v = i;
                    mind = d[i];
                }
            }
            if(v < 0){
                break;
            }
            visited[v] = true;
            for(Edge e : getEdges(v)){
                double newDist = d[v]+e.weight;
                if(newDist < d[e.to]){
                    pi[e.to] = v;
                    d[e.to] = newDist;
                }
            }
        }
        if(pi[dest] < 0){
            return null;
        }
        Stack<Integer> s = new Stack<>();
        
        int v = dest;
        s.push(v);
        while(pi[v] != -1){
            v = pi[v];
            s.push(v);
        }
        ArrayList<Integer> path = new ArrayList<>();
        while (!s.empty()){
            path.add(s.pop());
        }

        return path;
    }
}
