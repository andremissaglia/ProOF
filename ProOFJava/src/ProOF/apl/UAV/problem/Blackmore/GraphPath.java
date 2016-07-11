/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.problem.Blackmore;

import ProOF.apl.UAV.Swing.Graphics2DReal;
import java.awt.BasicStroke;
import java.awt.Color;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedList;

/**
 *
 * @author marcio
 */
public class GraphPath {
    protected final double cost;
    protected final LinkedList<GraphVertex> path;
    protected final LinkedList<Double> costs;
    public GraphPath(double cost, LinkedList<GraphVertex> path) {
        this.cost = cost;
        this.path = path;
        this.costs = new LinkedList<Double>();
        GraphVertex u = path.getLast();
        while(u!=null){
            costs.addFirst(u.cost);
            u = u.from;
        }
    }
    public GraphPath(ArrayList<Integer> codif, GraphStruct struct){
        this.path = new LinkedList<GraphVertex>();
        double total = 0;
        
        GraphVertex from = struct.source();
        from.from = null;
        from.cost = total;
        this.path.addLast(from);
        for(int v: codif){
            struct.vertexes[v].from = from;
            from = struct.vertexes[v];
            this.path.addLast(from);
            
            total += from.costTo(from.from);
            from.cost = total;
        }
        struct.target().from = from;
        from = struct.target();
        this.path.addLast(from);
        total += from.costTo(from.from);
        from.cost = total;
        
        this.cost = total;
        this.costs = new LinkedList<Double>();
        GraphVertex u = this.path.getLast();
        while(u!=null){
            costs.addFirst(u.cost);
            u = u.from;
        }
        
         //System.exit(-1);
        //
    }

    public GraphVertex[] path(){
        return path.toArray(new GraphVertex[path.size()]);
    }
    public Double[] costs(){
        return costs.toArray(new Double[costs.size()]);
    }

    @Override
    public boolean equals(Object obj) {
        GraphPath a = this;
        GraphPath b = (GraphPath)obj;
        if(a.path.size()==b.path.size()){
            Iterator<GraphVertex> itA = a.path.iterator();
            Iterator<GraphVertex> itB = b.path.iterator();
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
    
    public void paint(Graphics2DReal gr) throws Exception {
        gr.setColor(Color.GREEN);
        gr.g2.setStroke(new BasicStroke(4));
        GraphVertex v = null;
        for(GraphVertex u : path){
            gr.fillOvalR(u.point.getX(), u.point.getY(), 0.08, 0.08);

            if(v!=null){
                gr.drawLineR(v.point, u.point);
            }
            v = u;
        }
        gr.g2.setStroke(new BasicStroke(1));
    }
}
