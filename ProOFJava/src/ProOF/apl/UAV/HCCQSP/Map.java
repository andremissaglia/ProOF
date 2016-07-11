/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.HCCQSP;

import ProOF.apl.UAV.map.Obstacle2D;
import ilog.concert.IloNumVar;
import java.util.LinkedList;

/**
 *
 * @author marcio
 */
public class Map {
    public final String w[] = new String[]{"NFZ", "base", "lake", "forest"};  //é um conjunto com todos os tipos no qual uma região r está classificada
    public final Region r[] = new Region[]{
        new Region("NFZ",   "NFZ-1",    new double[]{3.01,1.76,1.74,3.04},      new double[]{2.42,2.42,3.13,3.17}),
        new Region("NFZ",   "NFZ-2",    new double[]{10.39,7.95,7.94,10.46},    new double[]{1.25,1.96,3.61,3.66}),
        new Region("lake",  "lake-1",   new double[]{5.49,6.53,6.3,5.17},       new double[]{0.26,0.69,1.64,1.7}),
        new Region("lake",  "lake-2",   new double[]{7.84,9.42,9.77},           new double[]{5.51,4.44,5.69}),
        new Region("base",  "base",     new double[]{0.48,0.98,0.96,0.49},      new double[]{2.46,2.46,3.15,3.11}),
        new Region("forest","forest-1", new double[]{11.62,12.48,12.52,11.68},  new double[]{1.48,1.45,2.37,2.39}),
        new Region("forest","forest-2", new double[]{10.44,11.36,11.38,10.49},  new double[]{4.0,3.98,4.95,4.92}),
    };

    public class Region extends Obstacle2D{
        public final String type;
        public Region(String type, String name, double x[], double y[]) {
            super(name, x, y);
            this.type = type;
        }
    }
    public final Region [] set(String w){
        LinkedList<Region> list = new LinkedList<Map.Region>();
        for(Region i : r){
            if(i.type.equals(w)){
                list.addLast(i);
            }
        }
        return list.toArray(new Region[list.size()]);
    }
}
