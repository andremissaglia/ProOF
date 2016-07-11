/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.map;


/**
 *
 * @author marcio
 */
public class Point3D {
    private final static double eixoY = 40000000.0;
    private final static double eixoX = 40075000.0;
    public final double x;
    public final double y;
    public final double h;
    public Point3D(double x, double y, double h) {
        this.x = x;
        this.y = y;
        this.h = h;
    }
    public Point3D(PointGeo base, PointGeo point) {
        double dif_lat = point.latitude - base.latitude;
        this.y = dif_lat*eixoY/360.0;

        double dif_long = point.longitude - base.longitude;
        double eixo = eixoX*Math.cos(base.latitude*Math.PI/180.0);
        this.x = dif_long*eixo/360.0;

        this.h = point.altitude - base.altitude;
    }
    public PointGeo parseTo(PointGeo base){
        //y = (p.lat-b.lat)*eixoY/360;
        //y*360/eixoY + b.lat = p.lat
        
        //x = (p.long-b.long)*eixoX*cos(b.lat*PI/180.0)/360.0;
        //x*360/(eixoX*cos(b.lat*PI/180)) + b.long = p.long
        return new PointGeo(
            base.longitude+ x*360/(eixoX*Math.cos(base.latitude*Math.PI/180)), 
            base.latitude + y*360/eixoY, 
            base.altitude + h
        );
    }
    public Point3D minus(Point3D o){
        return new Point3D(x-o.x, y-o.y, h-o.h);
    }
    public double angle(){
        return Math.atan2(y, x);
    }
}
