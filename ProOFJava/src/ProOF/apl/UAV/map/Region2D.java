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
import java.util.Scanner;

/**
 *
 * @author marcio
 */
public class Region2D extends Obstacle{
    private final String name;
    private final Point2D points[];
    public Region2D(Scanner sc, boolean anticlockwise) throws Exception {
        this.name = null;
        sc.nextLine();
        points = new Point2D[Integer.parseInt(sc.nextLine())];
        if(anticlockwise){
            for(int i=points.length-1; i>=0; i--){
                points[i] = new Point2D.Double(sc.nextDouble(), sc.nextDouble());
                sc.nextLine();
            }
        }else{
            for(int i=0; i<points.length; i++){
                points[i] = new Point2D.Double(sc.nextDouble(), sc.nextDouble());
                sc.nextLine();
            }
        }
            
        this.hyperplans = hyperplansFrom(false, points);
        this.rect = rectangleFrom(points);
    }
    public Region2D(String name, boolean dim3D, Point2D ...points) {
        this.name = name;
        this.points = points;
        this.hyperplans = hyperplansFrom(dim3D, points);
        this.rect = rectangleFrom(points);
    }
    
    private static Hyperplane[] hyperplansFrom(boolean dim3D, Point2D points[]){
        Hyperplane[] hyperplans = new Hyperplane[points.length];
        for(int i=0; i<points.length; i++){
            int k = (i+1) % points.length;
            double x1 = points[i].getX();
            double y1 = points[i].getY();
            double x2 = points[k].getX();
            double y2 = points[k].getY();

            //x'
            double ax = -( y2 - y1 );
            //y'
            double ay = +( x2 - x1 );
            
            
            //normalize
            double norm = Math.sqrt(ax*ax+ay*ay);
            ax /= norm;
            ay /= norm;

            double b = ax * x1  + ay * y1;

            if(dim3D){
                hyperplans[i] = new Hyperplane(b, ax, ay, 0.0);
            }else{
                hyperplans[i] = new Hyperplane(b, ax, ay);
            }
            //if(PRINT)System.out.printf(Locale.ENGLISH, "%8.4f * x + %8.4f * y  = %8.4f\n", ax, ay, b);
        }
        return hyperplans;
    }

    
    @Override
    public Point2D center() {
        double mx = averageX(points);
        double my = averageY(points);
        return new Point2D.Double(mx, my);
    }
    @Override
    public void paint(Graphics2DReal gr, double size) {
        if(name!=null){
            gr.paintPolygon(points, name, size, Color.LIGHT_GRAY, Color.BLACK);
        }else{
            gr.paintPolygon(points, Color.BLACK);
        }
    }
    @Override
    public void paint(Graphics2DReal gr, Color color, double size) {
        gr.paintPolygon(points, name, size, color, Color.BLACK);
    }
    @Override
    public void paint_sense(Graphics2DReal gr, double r, double f) {
        gr.paintNormal(points, hyperplans, r, f, Color.BLUE);
    }
}
