package ProOF.apl.UAV.problem.Blackmore;

import ProOF.CplexExtended.Hyperplane;
import ProOF.apl.UAV.Swing.Graphics2DReal;
import ProOF.apl.UAV.gen.linear.LinearSystem;
import ProOF.apl.UAV.gen.linear.uncertainty.pLinearStateUncertainty;
import ProOF.apl.UAV.map.Obstacle;
import ProOF.apl.UAV.map.Obstacle2D;
import ProOF.apl.UAV.map.Obstacle3DHalf;
import ProOF.apl.UAV.mission.Blackmore.BlackmoreInstance;
import ProOF.apl.UAV.mission.Blackmore.BlackmoreModel;
import java.io.IOException;
import java.util.ArrayList;
import java.util.LinkedList;

/**
 *
 * @author marcio
 */
public class GraphMap {
    protected GraphStruct graph;
    protected GraphPath best_path;
    protected ArrayList<GraphObstacle> obstacles;
    protected ArrayList<Integer> indexes;
    
    public String name() {
        return "Abusive";
    }

    public void createGraph(LinearSystem approach, BlackmoreInstance inst, pLinearStateUncertainty unc, double delta_to_cut)throws Exception {
        System.out.println("add "+name());

        System.out.println("clear cut data");
        graph = null;
        //ArrayList<GraphObstacle> obstacles = new ArrayList<GraphObstacle>();
        obstacles = new ArrayList<GraphObstacle>();
        indexes = new ArrayList<Integer>();
        int j=0;
        for(Obstacle obs : inst.obstacles){
            if(obs instanceof Obstacle2D){
                //LinearSystem approach, pLinearStateUncertainty unc, Obstacle2D obstacle, double delta_to_cut
                obstacles.add(new GraphObstacle(approach, unc, (Obstacle2D)obs, delta_to_cut));
                indexes.add(j++);
            }else if(obs instanceof Obstacle3DHalf){
                if(((Obstacle3DHalf)obs).is2D){
                    obstacles.add(new GraphObstacle(approach, unc, (Obstacle3DHalf)obs, delta_to_cut));
                    indexes.add(j++);
                }
            }
        }
        if(obstacles.isEmpty()){
            throw new Exception("it don't has 2D obstacles to create a Graph");
        }else{
            System.out.println("build new cut data");
            graph  =new GraphStruct(inst, obstacles);
        }
    }
    
    public void fix(LinearSystem approach, BlackmoreInstance inst, pLinearStateUncertainty unc, double delta_to_cut, ArrayList<Double> weights, GraphPath graph_path, BlackmoreModel model, boolean print) throws Exception{
        double cost = graph_path.cost;
        double d_cost= cost/approach.Waypoints();

        LinkedList<Integer>[][] Ssj = new LinkedList[approach.Steps()+1][obstacles.size()];
        for(int s=0; s<approach.Steps()+1; s++){
            for(int j=0; j<Ssj[s].length; j++){
                Ssj[s][j] = new LinkedList<Integer>();
            }
        }


        GraphVertex path[] = graph_path.path();
        if(print){
            System.out.printf("path = ");
            for(GraphVertex p : path){
                System.out.printf("%s ", p);
            }
            System.out.println();
        }
            
        
        Double costs[] = graph_path.costs();
        if(print){
            System.out.printf("costs = ");
            for(double c : costs){
                System.out.printf("%g ", c);
            }
            System.out.println();
        }
        ArrayList<Integer> waypoints = new ArrayList<Integer>();
        if(weights == null) {
            for(int i=0; i<path.length; i++){
                int t = (int)(costs[i]/d_cost+(path.length-i)*0.99999/(path.length));
                t = Math.max(t, 0);
                t = Math.min(t, approach.Waypoints());
                waypoints.add(t);
            }
        } else {
             if(print){
                System.out.printf("weights= ");
                for(double w : weights){
                    System.out.printf("%s ", (int)w);
                }
                System.out.println();
            }
            double total_weight = weights.stream().reduce(0.0, Double::sum);
            double sum = 0;
            waypoints.add(0);
            for(int i=0; i < weights.size(); i++){
                double w = weights.get(i);
                int t = (int) Math.round(approach.Waypoints()*(sum + w)/total_weight);
                t = Math.max(t, 0);
                t = Math.min(t, approach.Waypoints());
                sum += w;
                waypoints.add(t);
            }
            
        }
        correction(waypoints, costs, print);
        if(print){
            System.out.printf("waypoints= ");
            for(int w : waypoints){
                System.out.printf("%s ", w);
            }
            System.out.println();
        }
        
        for(int i=1; i<path.length; i++){
            int k = i-1;
            int lb = waypoints.get(k);
            int ub = waypoints.get(i);

            //int lb = Math.max((int)(costs[k]/d_cost+(path.length-k)*0.99999/(path.length)),0);
            //int ub = Math.min((int)(costs[i]/d_cost+(path.length-i)*0.99999/(path.length)),approach.Waypoints());
            if(print)System.out.printf("[%d][lb;ub] = [ %2d ; %2d ]\n",i, lb, ub);
            if(lb==ub){
                System.out.println("aaaaaaaaaa");
                throw new Exception("lb==ub");
            }
            int j=0;
            for(GraphObstacle obs : obstacles){
                for(int n=0; n<obs.obs.Gj(); n++){
                    Hyperplane H = obs.obs.hyperplans[n];
                    // - obs.obs.hyperplans[i].b
                    double c = unc.risk_allocation(0, delta_to_cut, H.a);

                    for(int t=lb; t<=ub; t++){
                        double alpha = (t-lb)*1.0/(ub-lb);
                        double x = path[k].point.getX()*(1-alpha) + path[i].point.getX()*alpha;
                        double y = path[k].point.getY()*(1-alpha) + path[i].point.getY()*alpha;
                        if(H.scalarProd(x, y) - H.b - c >= -1e-6){
                            int s = approach.Step(t);
                            if(!Ssj[s][j].contains(n)){
                                Ssj[s][j].addLast(n);
                            }
                        }
                    }


                }
                j++;
            }

        }

        if(print){
            System.out.println("---------------------------------------------------------------------");
            for(int s=0; s<approach.Steps()+1; s++){
                System.out.printf("|%2d| = ",s);
                for(int j=0; j<Ssj[s].length; j++){
                    System.out.printf(" %2d %10s",j, Ssj[s][j]);
                }
                System.out.println();
            }
        }

        for(int j=0; j<obstacles.size(); j++){
            for(int s=1; s<approach.Steps()+1; s++){
                boolean has = false;
                for(Integer n : Ssj[s-1][j]){
                    if(Ssj[s][j].contains(n)){
                        has = true;
                        break;
                    }
                }
                if(!has){
                    for(Integer n : Ssj[s-1][j]){
                        if(!Ssj[s][j].contains(n)){
                            Ssj[s][j].add(n);
                        }
                    }
                    for(Integer n : Ssj[s][j]){
                        if(!Ssj[s-1][j].contains(n)){
                            Ssj[s-1][j].add(n);
                        }
                    }
                }
            }
        }

        if(print){
            System.out.println("--------------------------[correction]-------------------------------");
            for(int s=0; s<approach.Steps()+1; s++){
                System.out.printf("|%2d| = ",s);
                for(int j=0; j<Ssj[s].length; j++){
                    System.out.printf(" %2d %10s",j, Ssj[s][j]);
                }
                System.out.println();
            }

            System.out.println("add new cuts");
        }
        int cont_tot = 0;
        int cont_cut = 0;
        for(int s=0; s<approach.Steps()+1; s++){
            int j1 = 0;
            for(GraphObstacle obs : obstacles){
                int j = indexes.get(j1++);
                for(int i=0; i<obs.obs.Gj(); i++){
                    if(Ssj[s][j].contains(i)){
                        model.Zs(j, s, i).setUB(1);
                    }else{
                        model.Zs(j, s, i).setUB(0);
                        cont_cut++;
                    }
                    cont_tot++;
                }
            }
        }
        if(print)System.out.printf("cuts reduce the binaries to %5.2f %%\n", cont_cut*100.0/cont_tot);
    }
    
    public void paint(Graphics2DReal gr) throws Exception {
        if(graph!=null){
            graph.paint(gr);
            for(GraphObstacle obs : obstacles){
                obs.paint(gr);
            }
        }
        if(best_path!=null){
            best_path.paint(gr);
        }
    }
    private static void correction(ArrayList<Integer> waypoints, Double costs[], boolean print) throws IOException{
        if(print){
            System.out.printf("waypoints = %s\n", waypoints);
            System.out.printf("d-costs   = [ ");
            for(int i=1; i<waypoints.size(); i++){
                if(waypoints.get(i)>waypoints.get(i-1)+1){
                    double val = (costs[i]-costs[i-1])/Math.pow(waypoints.get(i)-waypoints.get(i-1)+waypoints.get(i)/waypoints.get(waypoints.size()-1), 1);
                    System.out.printf("%g ", val);
                }else{
                    System.out.printf("%s ", "X");
                }
            }
            System.out.println("]");
        }
//        System.out.println("start:");
//        System.in.read();
        
        int index = -1;
        int n = 1;
        while(n<waypoints.size()){
            if(waypoints.get(n-1).equals(waypoints.get(n))){
                double min = Double.MAX_VALUE;
                for(int i=1; i<waypoints.size(); i++){
                    if(waypoints.get(i)>waypoints.get(i-1)+1){
                        double val = (costs[i]-costs[i-1])/Math.pow(waypoints.get(i)-waypoints.get(i-1)+waypoints.get(i)/waypoints.get(waypoints.size()-1), 1);
                        if(val<min){
                            min = val;
                            index = i;
                        }
                    }
                }
                if(print)System.out.printf("[%d | %g]\n", index, min);
                break;
            }
            n++;
        }
        if(index!=-1){
            if(index<n){
                for(int i=index; i<n; i++){
                    waypoints.set(i, waypoints.get(i)-1);
                }
            }else{
                for(int i=n; i<index; i++){
                    waypoints.set(i, waypoints.get(i)+1);
                }
            }
            correction(waypoints, costs, print);
            //System.in.read();
        }
//        else{
//            System.out.println("end:");
//            System.in.read();
//        }
    }
}
