/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.map;

import ProOF.CplexExtended.Hyperplane;
import ProOF.apl.UAV.Swing.Graphics2DReal;
import ProOF.utilities.uIO;
import java.awt.Color;
import java.awt.geom.Point2D;
import java.util.Scanner;

/**
 *
 * @author marcio
 */
public class Obstacle2D extends Obstacle{
    private final Point2D points[];
    private final String name;
    //private final Font font;
    public Obstacle2D(String name, Scanner sc) {
        this(name, sc, true);
    }
    public Obstacle2D(String name, Scanner sc, boolean ignore_frist_line) {
        this.name = name;
        if(ignore_frist_line)sc.nextLine();
        double x[] = uIO.toVectorDouble(sc.nextLine());
        double y[] = uIO.toVectorDouble(sc.nextLine());
        this.points = new Point2D[x.length];
        for(int i=0; i<x.length; i++){
            points[i] = new Point2D.Double(x[i], y[i]);
        }
        this.hyperplans = hyperplansFrom(points);
        this.rect = rectangleFrom(points);
    }
    public Obstacle2D(String name, double x[], double y[]) {
        this.name = name;
        this.points = new Point2D[x.length];
        for(int i=0; i<x.length; i++){
            points[i] = new Point2D.Double(x[i], y[i]);
        }
        this.hyperplans = hyperplansFrom(points);
        this.rect = rectangleFrom(points);
    }
    public Obstacle2D(String name, Point2D ...points) {
        this.name = name;
        this.points = points;
        this.hyperplans = hyperplansFrom(points);
        this.rect = rectangleFrom(points);
    }
    public Obstacle2D(String name, Point2D center, double length, double width, double rotation) {
        double x, y;
        //double R = (length+width)/4;
        Point2D points[] = new Point2D[4];
        
        x = (-width/2) * Math.cos(rotation) - (-length/2) * Math.sin(rotation);
        y = (-width/2) * Math.sin(rotation) + (-length/2) * Math.cos(rotation);
        points[0] = new Point2D.Double((x + center.getX()), (y + center.getY()));

        x = (-width/2) * Math.cos(rotation) - (+length/2) * Math.sin(rotation);
        y = (-width/2) * Math.sin(rotation) + (+length/2) * Math.cos(rotation);
        points[1] = new Point2D.Double((x + center.getX()), (y + center.getY()));

        x = (+width/2) * Math.cos(rotation) - (+length/2) * Math.sin(rotation);
        y = (+width/2) * Math.sin(rotation) + (+length/2) * Math.cos(rotation);
        points[2] = new Point2D.Double((x + center.getX()), (y + center.getY()));

        x = (+width/2) * Math.cos(rotation) - (-length/2) * Math.sin(rotation);
        y = (+width/2) * Math.sin(rotation) + (-length/2) * Math.cos(rotation);
        points[3] = new Point2D.Double((x + center.getX()), (y + center.getY()));
        
        
//        x = (-width/2) * Math.cos(rotation) - (-width/2) * Math.sin(rotation);
//        y = (-length/2) * Math.sin(rotation) + (-length/2) * Math.cos(rotation);
//        points[0] = new Point2D.Double((x + center.getX()), (y + center.getY()));
//
//        x = (-width/2) * Math.cos(rotation) - (+width/2) * Math.sin(rotation);
//        y = (-length/2) * Math.sin(rotation) + (+length/2) * Math.cos(rotation);
//        points[1] = new Point2D.Double((x + center.getX()), (y + center.getY()));
//
//        x = (+width/2) * Math.cos(rotation) - (+width/2) * Math.sin(rotation);
//        y = (+length/2) * Math.sin(rotation) + (+length/2) * Math.cos(rotation);
//        points[2] = new Point2D.Double((x + center.getX()), (y + center.getY()));
//
//        x = (+width/2) * Math.cos(rotation) - (-width/2) * Math.sin(rotation);
//        y = (+length/2) * Math.sin(rotation) + (-length/2) * Math.cos(rotation);
//        points[3] = new Point2D.Double((x + center.getX()), (y + center.getY()));
//        
        this.name = name;
        this.points = points;
        this.hyperplans = hyperplansFrom(points);
        this.rect = rectangleFrom(points);
    }
    
    private static Hyperplane[] hyperplansFrom(Point2D points[]){
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

            hyperplans[i] = new Hyperplane(b, ax, ay);
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
