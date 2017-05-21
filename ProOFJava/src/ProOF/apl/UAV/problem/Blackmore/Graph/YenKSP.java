package ProOF.apl.UAV.problem.Blackmore.Graph;

import java.util.ArrayList;
import java.util.List;
import java.util.PriorityQueue;

public class YenKSP {
    private final ArrayList<Path> A;
    private final PriorityQueue<Path> B;
    private final Graph2 g;
    private final int source;
    private final int dest;
    
    public YenKSP(Graph2 g, int source, int dest){
        this.g = g;
        A = new ArrayList<>();
        B = new PriorityQueue<>();
        this.source = source;
        this.dest = dest;
    }
    
    public Path next(){
        if(A.isEmpty()){
            Path p = new Path(g.dijkstra(source, dest));
            p.evaluate(g);
            A.add(p);
            return p;
        }
        ArrayList<Integer> lastPath = A.get(A.size()-1).path;
        for(int i = 0; i < lastPath.size()-1;i++){
            Graph2 gCopy = new Graph2(g);
            int spurNode = lastPath.get(i);
            List<Integer> rootPath = lastPath.subList(0, i+1);
            for(Path p : A){
                if(compare(rootPath, p.path)){
                    gCopy.remEdge(p.path.get(i), p.path.get(i+1));
                }
            }
            rootPath.subList(0, rootPath.size()-1).forEach((node) -> {
                gCopy.removeNode(node);
            });
            
            ArrayList<Integer> spurPath = gCopy.dijkstra(spurNode, dest);
            if(spurPath != null){
                ArrayList<Integer> newPath = new ArrayList<>(rootPath.subList(0, i));
                newPath.addAll(spurPath);
                Path p = new Path(newPath);
                p.evaluate(g);
                B.add(p);
            }
        }
        if(B.isEmpty()){
            return null;
        }
        Path p = B.poll();
        A.add(p);
        return p;
    }
    
    public boolean compare(List<Integer> a, List<Integer> b){
        if(a.size() > b.size()){
            return false;
        }
        for(int i = 0; i < a.size(); i++){
            if(!a.get(i).equals(b.get(i))){
                return false;
            }
        }
        return true;
    }
    
    public static void main(String[] args) {
        Graph2 g = new Graph2(10);
        g.addEdge(0,1,24);
        g.addEdge(0,4,11);
        g.addEdge(0,5,13);
        g.addEdge(0,7,26);
        g.addEdge(1,5,35);
        g.addEdge(1,6,25);
        g.addEdge(1,8,22);
        g.addEdge(2,7,28);
        g.addEdge(2,8,21);
        g.addEdge(3,5,32);
        g.addEdge(3,7,20);  
        g.addEdge(4,5,16);
        g.addEdge(4,8,9);
        g.addEdge(5,7,14);
        g.addEdge(5,8,22);
        g.addEdge(5,9,15);
        g.addEdge(6,9,28);
        g.addEdge(8,9,19);
        YenKSP ksp = new YenKSP(g, 6, 3);
        System.out.println(ksp.next());
    }
    
}
