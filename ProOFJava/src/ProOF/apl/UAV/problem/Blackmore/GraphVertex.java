/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.problem.Blackmore;

import java.awt.geom.Point2D;
import java.util.ArrayList;

/**
 *
 * @author marcio
 */
public class GraphVertex {
    protected final int id;
    protected final GraphObstacle obs;
    protected final Point2D point;
    protected final ArrayList<GraphVertex> adj = new ArrayList<GraphVertex>();
    protected double cost;
    protected GraphVertex from;

    public GraphVertex(int id, GraphObstacle obs, Point2D point) {
        this.id = id;
        this.obs = obs;
        this.point = point;
    }
    public GraphVertex(int id, double x, double y) {
        this.id = id;
        this.obs = null;
        this.point = new Point2D.Double(x, y);
    }
    protected void addEdgeTo(GraphVertex v) {
        adj.add(v);
    }
    protected double costTo(GraphVertex v){
        for(GraphVertex a : adj){
            if(a==v){
                return point.distance(v.point);
            }
        }
        return Double.POSITIVE_INFINITY;
    }
    
    @Override
    public String toString() {
        return ""+id;
    }
}
