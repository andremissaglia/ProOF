/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.problem.Blackmore;

import ProOF.apl.UAV.Swing.Graphics2DReal;
import ProOF.apl.UAV.mission.Blackmore.BlackmoreInstance;
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedList;

/**
 *
 * @author marcio
 */
public class GraphStruct {
    public final GraphVertex[] vertexes;
    
    public GraphStruct(BlackmoreInstance inst, ArrayList<GraphObstacle> obstacles) {
        ArrayList<GraphVertex> list = new ArrayList<GraphVertex>();
        list.add(new GraphVertex(0,inst.start_state[0], inst.start_state[1]));
        list.add(new GraphVertex(1,inst.end_point[0], inst.end_point[1]));
        for(GraphObstacle obs : obstacles){
            for(Point2D p : obs.vertexes){
                list.add(new GraphVertex(list.size(), obs, p));
            }
        }
        this.vertexes = list.toArray(new GraphVertex[list.size()]);
        for(int i=0; i<vertexes.length; i++){
            for(int j=i+1; j<vertexes.length; j++){
                if(!colision(vertexes[i], vertexes[j], obstacles)){
                    vertexes[i].addEdgeTo(vertexes[j]);
                    vertexes[j].addEdgeTo(vertexes[i]);
                }
            }
        }
    }
    public final GraphVertex source(){
        return vertexes[0];
    }
    public final GraphVertex target(){
        return vertexes[1];
    }
    public GraphVertex min(LinkedList<GraphVertex> Q){
        Iterator<GraphVertex> it = Q.iterator();
        GraphVertex min = it.next();
        while(it.hasNext()){
            GraphVertex v = it.next();
            if(v.costFromSource<min.costFromSource){
                min = v;
            }
        }
        return min;
    }
    public GraphVertex minTarget(LinkedList<GraphVertex> Q){
        Iterator<GraphVertex> it = Q.iterator();
        GraphVertex min = it.next();
        while(it.hasNext()){
            GraphVertex v = it.next();
            if(v.costToTarget<min.costToTarget){
                min = v;
            }
        }
        return min;
    }

    public final void AllDijkstra(LinkedList<GraphPath> All, GraphPath current, int limit, int level){
        if(limit>All.size()){
            if(level==0){
                if(All.contains(current)){
                    //System.out.println("repeat = "+current);
                }else{
                    System.out.println("All["+All.size()+"] = "+current);
                    All.addLast(current);
                }
            }else if(level>0){
                Iterator<GraphVertex> it = current.path.iterator();
                GraphVertex a = it.next();
                while(it.hasNext() && limit>All.size()){
                    GraphVertex b = it.next();
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

    public final GraphPath dijkstra(){
        LinkedList<GraphVertex> Q = new LinkedList<GraphVertex>();
        for (GraphVertex v : vertexes) {
            v.costFromSource = Double.MAX_VALUE;
            v.from = null;
            Q.addLast(v);
        }
        vertexes[0].costFromSource = 0.0; //source
        while(!Q.isEmpty()){
            GraphVertex u = min(Q);
            Q.remove(u);
            for(GraphVertex v : u.adj){
                //if(Q.contains(v)){
                    double dist = u.costFromSource + u.point.distance(v.point);
                    if(dist < v.costFromSource){
                        v.costFromSource = dist;
                        v.from = u;
                    }
                //}
            }
        }
        //path
        Q.clear();
        GraphVertex u =vertexes[1];  //target 
        while(u!=null){
            Q.addFirst(u);
            u = u.from;
        }
        return new GraphPath(vertexes[1].costFromSource, Q);
    }
    public final void dijkstraToTarget(){
        LinkedList<GraphVertex> Q = new LinkedList<GraphVertex>();
        for (GraphVertex v : vertexes) {
            v.costToTarget = Double.MAX_VALUE;
            v.to = null;
            Q.addLast(v);
        }
        vertexes[1].costToTarget = 0.0; // target
        while(!Q.isEmpty()){
            GraphVertex u = minTarget(Q);
            Q.remove(u);
            for(GraphVertex v : u.adj){
                //if(Q.contains(v)){
                    double dist = u.costToTarget + u.point.distance(v.point);
                    if(dist < v.costToTarget){
                        v.costToTarget = dist;
                        v.to = u;
                    }
                //}
            }
        }
    }
    private static boolean atBorder(GraphVertex from, GraphVertex to, ArrayList<GraphObstacle> obstacles) {
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
    private static boolean contains(GraphVertex v, ArrayList<GraphObstacle> obstacles){
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
    private static boolean colisionWith(GraphVertex from, GraphVertex to, ArrayList<GraphObstacle> obstacles, GraphObstacle exception){
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
    private static boolean colision(GraphVertex from, GraphVertex to, ArrayList<GraphObstacle> obstacles){
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
        for(GraphVertex v : vertexes){
            gr.setColor(Color.BLUE);
            gr.fillOvalR(v.point.getX(), v.point.getY(), 0.05, 0.05);

//                gr.setColor(Color.BLACK);
//                gr.drawStringR("%d", v.point.getX(), v.point.getY()-0.15, i++);
        }
        gr.setColor(Color.BLUE);
        for (GraphVertex v : vertexes) {
            for (GraphVertex u : v.adj) {
                gr.drawLineR(v.point, u.point);
            }
        }
    }
}
