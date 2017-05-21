package ProOF.apl.UAV.problem.Blackmore.Graph;

import java.util.ArrayList;

public class Path implements Comparable<Path>{
    public ArrayList<Integer> path;
    public double lenght;
    
    @Override
    public int compareTo(Path t) {
        double diff = (this.lenght - t.lenght);
        if(diff < 1e-3){
            return -1;
        }
        if(diff > 1e-3){
            return 1;
        }
        return 0;
    }
    public void evaluate(Graph2 g){
        this.lenght = 0;
        for(int i = 0; i < path.size() - 1; i++){
            this.lenght += g.get(path.get(i), path.get(i+1));
        }
    }

    public Path(ArrayList<Integer> path) {
        this.path = path;
    }

    @Override
    public String toString() {
        return lenght + ": " + path.toString();
    }
    
    
}
