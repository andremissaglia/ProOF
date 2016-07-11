/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.map;

import ProOF.CplexExtended.Hyperplane;
import ProOF.apl.UAV.Swing.Graphics2DReal;
import java.awt.Color;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;


/**
 *
 * @author marcio
 */
public abstract class Obstacle {
    public final static boolean PRINT = false;
    //public abstract void paint();
    public Hyperplane hyperplans[];
    public Rectangle2D rect;
    
    public abstract Point2D center();
    public abstract void paint(Graphics2DReal gr, double size);
    public abstract void paint(Graphics2DReal gr, Color color, double size);
    public abstract void paint_sense(Graphics2DReal gr, double r, double f);
    
    
    public final int Gj(){
        return hyperplans.length;
    }    
    public final Hyperplane plane(int i){
        return hyperplans[i]; 
    }
    
    /**
     * Calculate the distance for a point p to this obstacle
     * @param x
     * @return
     * @throws Exception 
     */
    public final double distance(double x[]) throws Exception{
        double max = Double.NEGATIVE_INFINITY;
        for(int i=0; i<Gj(); i++){
            //c_(i,t) (δ_jt ) ≤ a_ji^T*μ_t - b_ji
            double c = hyperplans[i].scalarProd(x) - hyperplans[i].b;
            max = Math.max(max, c);
        }
        return max>0 ? max : 0.0;
    }
    public final double distance(Point2D p) throws Exception{
        double max = Double.NEGATIVE_INFINITY;
        for(int i=0; i<Gj(); i++){
            //c_(i,t) (δ_jt ) ≤ a_ji^T*μ_t - b_ji
            double c = hyperplans[i].scalarProd(p.getX(), p.getY()) - hyperplans[i].b;
            max = Math.max(max, c);
        }
        return max>0 ? max : 0.0;
    }
    /**
     * Calculate the distance form a point p to the ith hyperplane of obstacle
     * @param i
     * @param x
     * @return max( 0 , a^T * p - b)
     */
    public final double distance(int i, double x[]){
        double c = hyperplans[i].scalarProd(x) - hyperplans[i].b;
        return Math.max(c, 0.0);
    }
    

    
    public final int indexNear(double[] p) {
        int k = 0;
        double max = Double.NEGATIVE_INFINITY;
        for(int i=0; i<Gj(); i++){
            //c_(i,t) (δ_jt ) ≤ a_ji^T*μ_t - b_ji
            double c = hyperplans[i].scalarProd(p) - hyperplans[i].b;
            if(c>max){
                max = c;
                k=i;
            }
        }
        return k;
    }
    
    public static double trueAllocation(double x[], Obstacle ...obs) throws Exception{
        double alloc = Double.POSITIVE_INFINITY;
        for(Obstacle o : obs){
            alloc = Math.min(alloc, o.distance(x));
        }
        return alloc;
    }
    public static int indexNear(double[] p, Obstacle ...obs) throws Exception{
        int k = -1;
        double dist = Double.POSITIVE_INFINITY;
        for(int j=0; j<obs.length; j++){
            double val = obs[j].distance(p);
            if(val<dist){
                dist = val;
                k = j;
            }
        }
        return k;
    }
    
    
    public static Rectangle2D rectangleFrom(Point2D points[]){
        Rectangle2D rect = null;
        for(Point2D p:points){
            if(rect==null){
                rect = new Rectangle2D.Double(p.getX(), p.getY(), 0, 0);
            }else{
                rect.add(p);
            }
        }
        return rect;
    }

    
    
    
    protected static double averageX(Point2D[] points){
        double sum = 0;
        for(Point2D p : points){
            sum += p.getX();
        }
        return sum/points.length;
    }
    protected static double averageY(Point2D[] points){
        double sum = 0;
        for(Point2D p : points){
            sum += p.getY();
        }
        return sum/points.length;
    }
}
