/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.mission.Blackmore;

import Jama.Matrix;
import ProOF.apl.UAV.Swing.Graphics2DReal;
import ProOF.CplexExtended.Hyperplane;
import ProOF.apl.UAV.map.Obstacle;
import ProOF.apl.UAV.map.Obstacle2D;
import ProOF.apl.UAV.map.Obstacle3DHalf;
import ProOF.apl.UAV.oCuts;
import ProOF.com.language.Factory;
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Polygon;
import java.awt.geom.Path2D;
import java.awt.geom.Point2D;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedList;


/**
 *
 * @author marcio
 */
public class fBlackmoreCuts extends Factory<oCuts>{
    public static final fBlackmoreCuts obj = new fBlackmoreCuts();
    @Override
    public String name() {
        return "Blackmore Cuts";
    }
    @Override
    public oCuts build(int index) {  //build the operators
        switch(index){
            case 0: return new Empty();
            case 1: return new CutAbusive();
            case 2: return new CutConservative();
        }
        return null;
    }
    private class Empty extends oCuts<BlackmoreApproach, BlackmoreModel>{
        @Override
        public String name() {
            return "Empty Cust";
        }
        @Override
        public boolean[][] add_cuts(BlackmoreApproach approach, BlackmoreModel model) throws Exception {
            return null;
        }
        @Override
        public void paint(Graphics2DReal gr) throws Exception {
            
        }
    }
    
    private static class Vertex{
        private final int id;
        private final GraphObstacle obs;
        private final Point2D point;
        private final ArrayList<Vertex> adj = new ArrayList<Vertex>();
        private double cost;
        private Vertex from;
        
        public Vertex(int id, GraphObstacle obs, Point2D point) {
            this.id = id;
            this.obs = obs;
            this.point = point;
        }
        public Vertex(int id, double x, double y) {
            this.id = id;
            this.obs = null;
            this.point = new Point2D.Double(x, y);
        }
        private void addEdgeTo(Vertex v) {
            adj.add(v);
        }

        @Override
        public String toString() {
            return ""+id;
        }
    }
    
    private static class Dijkstra{
        private final double cost;
        private final LinkedList<Vertex> path;
        private final LinkedList<Double> costs;
        public Dijkstra(double cost, LinkedList<Vertex> path) {
            this.cost = cost;
            this.path = path;
            this.costs = new LinkedList<Double>();
            Vertex u = path.getLast();
            while(u!=null){
                costs.addFirst(u.cost);
                u = u.from;
            }
        }
        
        public Vertex[] path(){
            return path.toArray(new Vertex[path.size()]);
        }
        public Double[] costs(){
            return costs.toArray(new Double[costs.size()]);
        }
        
        @Override
        public boolean equals(Object obj) {
            Dijkstra a = this;
            Dijkstra b = (Dijkstra)obj;
            if(a.path.size()==b.path.size()){
                Iterator<Vertex> itA = a.path.iterator();
                Iterator<Vertex> itB = b.path.iterator();
                while(itA.hasNext() && itA.next()==itB.next()){
                    
                }
                return !itA.hasNext();
            }
            return super.equals(obj); //To change body of generated methods, choose Tools | Templates.
        }

        @Override
        public String toString() {
            return String.format("cost %5.2f | path = %s", cost, path);
        }
    }
    
    private static class GraphStruct{
        private final Vertex[] vertexes;
        private final Dijkstra dijkstra;
        public GraphStruct(BlackmoreApproach approach, ArrayList<GraphObstacle> obstacles) {
            ArrayList<Vertex> list = new ArrayList<Vertex>();
            list.add(new Vertex(0,approach.inst.start_state[0], approach.inst.start_state[1]));
            list.add(new Vertex(1,approach.inst.end_point[0], approach.inst.end_point[1]));
            for(GraphObstacle obs : obstacles){
                for(Point2D p : obs.vertexes){
                    list.add(new Vertex(list.size(), obs, p));
                }
            }
            this.vertexes = list.toArray(new Vertex[list.size()]);
            for(int i=0; i<vertexes.length; i++){
                for(int j=i+1; j<vertexes.length; j++){
                    if(!colision(vertexes[i], vertexes[j], obstacles)){
                        vertexes[i].addEdgeTo(vertexes[j]);
                        vertexes[j].addEdgeTo(vertexes[i]);
                    }
                }
            }
            
//            LinkedList<Dijkstra> All = new LinkedList<Dijkstra>();
//            Dijkstra sol = dijkstra();
//            
//            AllDijkstra(All, sol, 10, 0);
//            //AllDijkstra(All, sol, 10, 1);
//            //AllDijkstra(All, sol, 10, 2);
//            //AllDijkstra(All, sol, 20, 3);
//            //AllDijkstra(All, sol, 20, 4);
//            dijkstra = All.get(0);            
            dijkstra = dijkstra();
        }
        public Vertex min(LinkedList<Vertex> Q){
            Iterator<Vertex> it = Q.iterator();
            Vertex min = it.next();
            while(it.hasNext()){
                Vertex v = it.next();
                if(v.cost<min.cost){
                    min = v;
                }
            }
            return min;
        }

        public final void AllDijkstra(LinkedList<Dijkstra> All, Dijkstra current, int limit, int level){
            if(limit>All.size()){
                if(level==0){
                    if(All.contains(current)){
                        //System.out.println("repeat = "+current);
                    }else{
                        System.out.println("All["+All.size()+"] = "+current);
                        All.addLast(current);
                    }
                }else if(level>0){
                    Iterator<Vertex> it = current.path.iterator();
                    Vertex a = it.next();
                    while(it.hasNext() && limit>All.size()){
                        Vertex b = it.next();
                        a.adj.remove(b);
                        b.adj.remove(a);

                        AllDijkstra(All, dijkstra(), limit, level-1);
                        
                        a.adj.add(b);
                        b.adj.add(a);

                        a = b;
                    }
                }
            }
                
        }

        public final Dijkstra dijkstra(){
            LinkedList<Vertex> Q = new LinkedList<Vertex>();
            for (Vertex v : vertexes) {
                v.cost = Double.MAX_VALUE;
                v.from = null;
                Q.addLast(v);
            }
            vertexes[0].cost = 0.0; //source
            while(!Q.isEmpty()){
                Vertex u = min(Q);
                Q.remove(u);
                for(Vertex v : u.adj){
                    //if(Q.contains(v)){
                        double dist = u.cost + u.point.distance(v.point);
                        if(dist < v.cost){
                            v.cost = dist;
                            v.from = u;
                        }
                    //}
                }
            }
            //path
            Q.clear();
            Vertex u =vertexes[1];  //target 
            while(u!=null){
                Q.addFirst(u);
                u = u.from;
            }
            return new Dijkstra(vertexes[1].cost, Q);
        }
        private static boolean atBorder(Vertex from, Vertex to, ArrayList<GraphObstacle> obstacles) {
            if(from==to){
                return true;
            }else if(from.obs==null || to.obs==null){
                return false;
            }else if(from.obs==to.obs){
                GraphObstacle obs = from.obs;
                Point2D cur = obs.vertexes.get(obs.vertexes.size()-1);
                Iterator<Point2D> it = obs.vertexes.iterator();
                while(it.hasNext()){
                    Point2D next = it.next();
                    if( (cur==from.point && next==to.point) || (cur==to.point && next==from.point)){
                        return !colisionWith(from, to, obstacles, obs);
                    }
                    cur = next;
                }
            }
            return false;
        }
        private static boolean contains(Vertex v, ArrayList<GraphObstacle> obstacles){
            for(GraphObstacle obs: obstacles){
                if(v.obs!=obs && obs.path.contains(v.point)){
                    return true;
                }
            }
            return false;
        }
        
        private static boolean contains(Point2D point, ArrayList<GraphObstacle> obstacles, GraphObstacle exception){
            for(GraphObstacle obs: obstacles){
                if(obs!=exception && obs.path.contains(point)){
                    return true;
                }
            }
            return false;
        }
        private static final int MAX = 100;
        private static boolean colisionWith(Vertex from, Vertex to, ArrayList<GraphObstacle> obstacles, GraphObstacle exception){
            for(int i=1; i<MAX; i++){
                double alfa = (i*1.0)/MAX;
                double x = from.point.getX()*alfa+to.point.getX()*(1-alfa);
                double y = from.point.getY()*alfa+to.point.getY()*(1-alfa);
                Point2D point = new Point2D.Double(x, y);
                if(contains(point, obstacles, exception)){
                    return true;
                }
            }
            return false;
        }
        private static boolean colision(Vertex from, Vertex to, ArrayList<GraphObstacle> obstacles){
            if(contains(from, obstacles)){
                return true;
            }else if(contains(to, obstacles)){
                return true;
            }else if(atBorder(from,to, obstacles)){
                return false;
            }else {
                return colisionWith(from, to, obstacles, null);
            }
//            if(contains(from, obstacles)){
//                return true;
//            }else if(contains(to, obstacles)){
//                return true;
//            }else if(atBorder(from,to)){
//                return false;
//            }else {
//                return colisionWith(from, to, obstacles);
//            }
        }
        public void paint(Graphics2DReal gr) throws Exception {
            int i=0;
            for(Vertex v : vertexes){
                gr.setColor(Color.BLUE);
                gr.fillOvalR(v.point.getX(), v.point.getY(), 0.05, 0.05);
                
//                gr.setColor(Color.BLACK);
//                gr.drawStringR("%d", v.point.getX(), v.point.getY()-0.15, i++);
            }
            gr.setColor(Color.BLUE);
            for (Vertex v : vertexes) {
                for (Vertex u : v.adj) {
                    gr.drawLineR(v.point, u.point);
                }
            }
            gr.setColor(Color.GREEN);
            gr.g2.setStroke(new BasicStroke(4));
            Vertex v = null;
            for(Vertex u : dijkstra.path){
                gr.fillOvalR(u.point.getX(), u.point.getY(), 0.08, 0.08);
                
                if(v!=null){
                    gr.drawLineR(v.point, u.point);
                }
                v = u;
            }
            gr.g2.setStroke(new BasicStroke(1));
        }

        
    }
    
    private static class GraphObstacle{
        private final ArrayList<Point2D> vertexes = new ArrayList<Point2D>();
        private final Path2D path;
        private final Obstacle obs;
        public GraphObstacle(BlackmoreApproach approach, BlackmoreModel model, Obstacle2D obstacle) throws Exception {
            this.obs = obstacle;
            this.path = new Path2D.Double();
            for(int i = 0; i<obstacle.hyperplans.length; i++){
                int k = (i-1+obstacle.hyperplans.length)%obstacle.hyperplans.length;
                
                double fixed_risk_i = Double.MAX_VALUE;
                double fixed_risk_k = Double.MAX_VALUE;
                for(int t=0; t<approach.Waypoints()+1; t++){
                    fixed_risk_i = Math.min(fixed_risk_i, approach.unc.risk_allocation(t, model.delta_to_cut, obstacle.hyperplans[i].a));
                    fixed_risk_k = Math.min(fixed_risk_k, approach.unc.risk_allocation(t, model.delta_to_cut, obstacle.hyperplans[k].a));
                }
                //c_(i,t) (δ_jt ) + b_i ≤ a_i^T*μ_t
                //c_(k,t) (δ_jt ) + b_k ≤ a_k^T*μ_t
                //C+B = A*x
                //x = A^(-1)*(C+B)
                Matrix A = new Matrix(new double[][]{
                    {obstacle.hyperplans[i].a[0],obstacle.hyperplans[i].a[1]},
                    {obstacle.hyperplans[k].a[0],obstacle.hyperplans[k].a[1]},
                });
                Matrix B = new Matrix(new double[][]{
                    {obstacle.hyperplans[i].b},
                    {obstacle.hyperplans[k].b}
                });
                Matrix C = new Matrix(new double[][]{
                    {fixed_risk_i},
                    {fixed_risk_k}
                });
                Matrix x = A.inverse().times(C.plus(B));
                Point2D p = new Point2D.Double(x.get(0, 0), x.get(1, 0));
                vertexes.add(p);
                if(i==0){
                    path.moveTo(p.getX(), p.getY());
                }else{
                    path.lineTo(p.getX(), p.getY());
                }
            }
            path.closePath();
        }
        public GraphObstacle(BlackmoreApproach approach, BlackmoreModel model, Obstacle3DHalf obstacle) throws Exception {
            this.obs = obstacle;
            this.path = new Path2D.Double();
            final int H = obstacle.hyperplans.length;
            for(int i = 0; i<H; i++){
                int k = (i-1+H)%H;
                
                double fixed_risk_i = Double.MAX_VALUE;
                double fixed_risk_k = Double.MAX_VALUE;
                for(int t=0; t<approach.Waypoints()+1; t++){
                    fixed_risk_i = Math.min(fixed_risk_i, approach.unc.risk_allocation(t, model.delta_to_cut, obstacle.hyperplans[i].a));
                    fixed_risk_k = Math.min(fixed_risk_k, approach.unc.risk_allocation(t, model.delta_to_cut, obstacle.hyperplans[k].a));
                }
                //c_(i,t) (δ_jt ) + b_i ≤ a_i^T*μ_t
                //c_(k,t) (δ_jt ) + b_k ≤ a_k^T*μ_t
                //C+B = A*x
                //x = A^(-1)*(C+B)
                Matrix A = new Matrix(new double[][]{
                    {obstacle.hyperplans[i].a[0],obstacle.hyperplans[i].a[1]},
                    {obstacle.hyperplans[k].a[0],obstacle.hyperplans[k].a[1]},
                });
                Matrix B = new Matrix(new double[][]{
                    {obstacle.hyperplans[i].b},
                    {obstacle.hyperplans[k].b}
                });
                Matrix C = new Matrix(new double[][]{
                    {fixed_risk_i},
                    {fixed_risk_k}
                });
                Matrix x = A.inverse().times(C.plus(B));
                Point2D p = new Point2D.Double(x.get(0, 0), x.get(1, 0));
                vertexes.add(p);
                if(i==0){
                    path.moveTo(p.getX(), p.getY());
                }else{
                    path.lineTo(p.getX(), p.getY());
                }
            }
            path.closePath();
        }
        public void paint(Graphics2DReal gr) throws Exception {
//            gr.setColor(Color.BLACK);
//            for(int i=0; i<obs.Gj(); i++){
//                int k = (i+1)%vertexes.size();
//                Point2D p1 = vertexes.get(i);
//                Point2D p2 = vertexes.get(k);
//                gr.drawStringR("%d", (p1.getX()+p2.getX())/2, (p1.getY()+p2.getY())/2, i);
//            }
            //gr.draw(path);
        }
    }

    private static int getIndex(double val, Double costs[]){
        double sum = costs[1];
        for(int i=1; i<costs.length-1; i++){
            if(sum>val){
                return i-1;
            }else{
                sum = costs[i+1];
            }
        }
        return costs.length-1;
    }
    
    private static void correction(ArrayList<Integer> waypoints, Double costs[]) throws IOException{
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
                System.out.printf("[%d | %g]\n", index, min);
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
            correction(waypoints, costs);
            //System.in.read();
        }
//        else{
//            System.out.println("end:");
//            System.in.read();
//        }
    }
    
    private static class CutAbusive extends oCuts<BlackmoreApproach, BlackmoreModel>{
        private GraphStruct graph;
        private ArrayList<GraphObstacle> obstacles;
        private ArrayList<Integer> indexes;
        @Override
        public String name() {
            return "Abusive";
        }
        
        @Override
        public boolean[][] add_cuts(BlackmoreApproach approach, BlackmoreModel model) throws Exception {
            System.out.println("add "+name());
            
            System.out.println("clear cut data");
            graph = null;
            //ArrayList<GraphObstacle> obstacles = new ArrayList<GraphObstacle>();
            obstacles = new ArrayList<GraphObstacle>();
            indexes = new ArrayList<Integer>();
            int j=0;
            for(Obstacle obs : approach.inst.obstacles){
                if(obs instanceof Obstacle2D){
                    obstacles.add(new GraphObstacle(approach, model, (Obstacle2D)obs));
                    indexes.add(j++);
                }else if(obs instanceof Obstacle3DHalf){
                    if(((Obstacle3DHalf)obs).is2D){
                        obstacles.add(new GraphObstacle(approach, model, (Obstacle3DHalf)obs));
                        indexes.add(j++);
                    }
                }
            }
            if(obstacles.isEmpty()){
                System.out.println("it don't has 2D obstacles to add new cuts");
                return null;
            }else{
                System.out.println("build new cut data");
                
                
                graph  =new GraphStruct(approach, obstacles);
                
                double cost = graph.dijkstra.cost;
                double d_cost= cost/approach.Waypoints();
                
                LinkedList<Integer>[][] Ssj = new LinkedList[approach.Steps()+1][obstacles.size()];
                for(int s=0; s<approach.Steps()+1; s++){
                    for(j=0; j<Ssj[s].length; j++){
                        Ssj[s][j] = new LinkedList<Integer>();
                    }
                }
                
                
                Vertex path[] = graph.dijkstra.path();
                Double costs[] = graph.dijkstra.costs();
                System.out.printf("costs = ");
                for(double c : costs){
                    System.out.printf("%g ", c);
                }
                System.out.println();

                ArrayList<Integer> waypoints = new ArrayList<Integer>();
                for(int i=0; i<path.length; i++){
                    int t = (int)(costs[i]/d_cost+(path.length-i)*0.99999/(path.length));
                    t = Math.max(t, 0);
                    t = Math.min(t, approach.Waypoints());
                    waypoints.add(t);
                }
                
                correction(waypoints, costs);
                
                for(int i=1; i<path.length; i++){
                    int k = i-1;
                    int lb = waypoints.get(k);
                    int ub = waypoints.get(i);
                    
                    //int lb = Math.max((int)(costs[k]/d_cost+(path.length-k)*0.99999/(path.length)),0);
                    //int ub = Math.min((int)(costs[i]/d_cost+(path.length-i)*0.99999/(path.length)),approach.Waypoints());
                    System.out.printf("[%d][lb;ub] = [ %2d ; %2d ]\n",i, lb, ub);
                    if(lb==ub){
                        throw new Exception("lb==ub");
                    }
                    j=0;
                    for(GraphObstacle obs : obstacles){
                        for(int n=0; n<obs.obs.Gj(); n++){
                            Hyperplane H = obs.obs.hyperplans[n];
                            // - obs.obs.hyperplans[i].b
                            double c = approach.unc.risk_allocation(0, model.delta_to_cut, H.a);

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

                
                System.out.println("---------------------------------------------------------------------");
                for(int s=0; s<approach.Steps()+1; s++){
                    System.out.printf("|%2d| = ",s);
                    for(j=0; j<Ssj[s].length; j++){
                        System.out.printf(" %2d %10s",j, Ssj[s][j]);
                    }
                    System.out.println();
                }
                
                for(j=0; j<obstacles.size(); j++){
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
                
                System.out.println("--------------------------[correction]-------------------------------");
                for(int s=0; s<approach.Steps()+1; s++){
                    System.out.printf("|%2d| = ",s);
                    for(j=0; j<Ssj[s].length; j++){
                        System.out.printf(" %2d %10s",j, Ssj[s][j]);
                    }
                    System.out.println();
                }
                
                System.out.println("add new cuts");
                int cont_tot = 0;
                int cont_cut = 0;
                for(int s=0; s<approach.Steps()+1; s++){
                    int j1 = 0;
                    for(GraphObstacle obs : obstacles){
                        j = indexes.get(j1++);
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
                System.out.printf("cuts reduce the binaries to %5.2f %%\n", cont_cut*100.0/cont_tot);
                
                return null;
            }
        }
        
        @Override
        public void paint(Graphics2DReal gr) throws Exception {
            if(graph!=null){
                graph.paint(gr);
                for(GraphObstacle obs : obstacles){
                    obs.paint(gr);
                }
            }
        } 
    }
    
    private static class CutConservative extends oCuts<BlackmoreApproach, BlackmoreModel>{
        private GraphStruct graph; 
        private ArrayList<GraphObstacle> obstacles;
        private ArrayList<Integer> indexes;
        @Override
        public String name() {
            return "Conservative";
        }
        
        @Override
        public boolean[][] add_cuts(BlackmoreApproach approach, BlackmoreModel model) throws Exception {
            System.out.println("add "+name());
            
            System.out.println("clear cut data");
            graph = null;
            //ArrayList<GraphObstacle> obstacles = new ArrayList<GraphObstacle>();
            obstacles = new ArrayList<GraphObstacle>();
            indexes = new ArrayList<Integer>();
            int j=0;
            for(Obstacle obs : approach.inst.obstacles){
                if(obs instanceof Obstacle2D){
                    obstacles.add(new GraphObstacle(approach, model, (Obstacle2D)obs));
                    indexes.add(j++);
                }
            }
            if(obstacles.isEmpty()){
                System.out.println("it don't has 2D obstacles to add new cuts");
                return null;
            }else{
                System.out.println("build new cut data");
                
                
                graph  =new GraphStruct(approach, obstacles);
                
                double cost = graph.dijkstra.cost;
                double d_cost= cost/approach.Waypoints();
                
                LinkedList<Integer>[][] Ssj = new LinkedList[approach.Steps()+1][obstacles.size()];
                for(int s=0; s<approach.Steps()+1; s++){
                    for(j=0; j<Ssj[s].length; j++){
                        Ssj[s][j] = new LinkedList<Integer>();
                    }
                }
                
                final int E = 0;
                Vertex path[] = graph.dijkstra.path();
                Double costs[] = graph.dijkstra.costs();
                for(int i=1; i<path.length; i++){
                    int k = i-1;
                    
                    int lb = Math.max((int)(costs[k]/d_cost)-E,0);
                    int ub = Math.min((int)(costs[i]/d_cost+0.99999)+E,approach.Waypoints());
                    
                    LinkedList<Integer>[] Oj = new LinkedList[obstacles.size()];
                    j=0;
                    for(GraphObstacle obs : obstacles){
                        Oj[j] = new LinkedList<Integer>();
                        for(int n=0; n<obs.obs.Gj(); n++){
                            Hyperplane H = obs.obs.hyperplans[n];
                            // - obs.obs.hyperplans[i].b
                            double c = approach.unc.risk_allocation(0, model.delta_to_cut, H.a);
                            if(H.scalarProd(path[k].point.getX(), path[k].point.getY()) - H.b - c >= -1e-6 ||
                                H.scalarProd(path[i].point.getX(), path[i].point.getY()) - H.b - c >= -1e-6){
                                Oj[j].addLast(n);
                                for(int t=lb; t<=ub; t++){
                                    int s = approach.Step(t);
                                    if(!Ssj[s][j].contains(n)){
                                        Ssj[s][j].addLast(n);
                                    }
                                }
                            }
                        }
                        j++;
                    }
                    
                    System.out.printf("[%d][lb;ub] = [ %2d ; %2d ] |",i, lb, ub);
                    for(j=0; j<Oj.length; j++){
                        System.out.printf(" %2d %s",j, Oj[j]);
                    }
                    System.out.println();
                    
                }

                System.out.println("---------------------------------------------------------------------");
                for(int s=0; s<approach.Steps()+1; s++){
                    System.out.printf("|%d| = ",s);
                    for(j=0; j<Ssj[s].length; j++){
                        //if(Ojt[j][t]){
                            System.out.printf(" %2d %s",j, Ssj[s][j]);
                        //}
                    }
                    System.out.println();
                }
                
                System.out.println("add new cuts");
                int cont_tot = 0;
                int cont_cut = 0;
                for(int s=0; s<approach.Steps()+1; s++){
                    int j1 = 0;
                    for(GraphObstacle obs : obstacles){
                        j = indexes.get(j1++);
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
                System.out.printf("cuts reduce the binaries to %5.2f %%\n", cont_cut*100.0/cont_tot);
                
                return null;
            }
        }
        
        @Override
        public void paint(Graphics2DReal gr) throws Exception {
            if(graph!=null){
                graph.paint(gr);
                for(GraphObstacle obs : obstacles){
                    obs.paint(gr);
                }
            }
        }
    }
}

