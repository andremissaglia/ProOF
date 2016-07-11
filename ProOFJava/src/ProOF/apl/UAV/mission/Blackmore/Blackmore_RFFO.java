/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.mission.Blackmore;

import ProOF.apl.UAV.oFull;
import ProOF.apl.advanced2.FMS.RFFO.RFFOModel;
import ProOF.apl.advanced2.FMS.RFFO.RelaxVar;
import ProOF.com.Linker.LinkerApproaches;
import ProOF.com.Linker.LinkerParameters;
import ProOF.com.Linker.LinkerResults;
import ProOF.utilities.uDataUp;
import ProOF.utilities.uSort;
import ilog.concert.IloException;
import ilog.concert.IloNumVarType;
import java.util.ArrayList;

/**
 *
 * @author marcio
 */
public class Blackmore_RFFO extends RFFOModel{
    private static final IloNumVarType type = IloNumVarType.Bool;
    
    private BlackmoreApproach approach = new BlackmoreApproach();
    private oFull full_op;
    private BlackmoreModel model;
    
    public Blackmore_RFFO() throws IloException {
        
    }

    @Override
    public String name() {
        return "BlackmoreRFFO";
    }
    @Override
    public void services(LinkerApproaches link) throws Exception {
        super.services(link); //To change body of generated methods, choose Tools | Templates.
        approach = link.add(approach);
        full_op = link.need(oFull.class, full_op);
    }
    
    @Override
    public void parameters(LinkerParameters link) throws Exception {
        super.parameters(link);
    }
    
//    
//    private ArrayList<RelaxVar> raster = null;
//    private ArrayList<RelaxVar> raster(){
//        if(raster==null){
//            raster = new ArrayList<RelaxVar>();
//            for(int s=0; s<approach.Steps()+1; s++){
//                for(int j=0; j<approach.inst.J(); j++){
//                    for(int i=0; i<approach.inst.Gj(j); i++){
//                        raster.add(new RelaxVar(IloNumVarType.Bool, model.Zs(j, s, i)));
//                    }
//                }
//            }
//        }
//        return raster;
//    }
    
    @Override
    protected ArrayList<RelaxVar> relax_variables() throws Exception {
        
        ArrayList<uDataUp<RelaxVar>> ord = new ArrayList<uDataUp<RelaxVar>>();
        //Guied by solution path, added infeasible obstacles frist, in sequence the most near
        double Xt[][] = model.getXt();
        for (int s = 0; s < approach.Steps()+ 1; s++) {
            for (int j = 0; j < approach.inst.J(); j++) {
                //double fixed_delta = approach.Delta();        //FRR
                
                // calculate the minimum distance in the path from s to s+1
                double distance = Double.POSITIVE_INFINITY;
                for(int t=approach.Waypoint(s); t<approach.Waypoint(s+1); t++){
                    distance = Math.min(distance, approach.inst.obstacles[j].distance(Xt[t]));
                }
                if(s+1==approach.Steps()){
                    int t = approach.Waypoints();
                    distance = Math.min(distance, approach.inst.obstacles[j].distance(Xt[t]));
                }
                
                for(int i=0; i<approach.inst.Gj(j); i++){
                    ord.add(new uDataUp<RelaxVar>(new RelaxVar(model.Zs(j, s, i)), distance));
                }
            }
        }
        uDataUp<RelaxVar> R[] = ord.toArray(new uDataUp[ord.size()]);
        uSort.sort(R);
       
        
        ArrayList<RelaxVar> list = new ArrayList<RelaxVar>();
        for(uDataUp<RelaxVar> d : R){
            list.add(d.data);
        }
        return list;
//        ArrayList<RelaxVar> list = new ArrayList<RelaxVar>();
//        for(int s=0; s<approach.Steps()+1; s++){
//            for(int j=0; j<approach.inst.J(); j++){
//                for(int i=0; i<approach.inst.Gj(j); i++){
//                    list.add(new RelaxVar(IloNumVarType.Bool, model.Zs(j, s, i)));
//                }
//            }
//        }
//        return list;
    }

    @Override
    public ArrayList<RelaxVar> fix_variables() throws Exception {
//        ArrayList<RelaxVar> list = new ArrayList<RelaxVar>();
//        for(int s=0; s<approach.Steps()+1; s++){
//            for(int j=0; j<approach.inst.J(); j++){
//                for(int i=0; i<approach.inst.Gj(j); i++){
//                    list.add(new RelaxVar(IloNumVarType.Bool, model.Zs(j, s, i)));
//                }
//            }
//        }
//        return list;
        ArrayList<uDataUp<RelaxVar>> ord = new ArrayList<uDataUp<RelaxVar>>();
        //Guied by solution path, added infeasible obstacles frist, in sequence the most near
        double Xt[][] = model.getXt();
        for (int s = 0; s < approach.Steps()+ 1; s++) {
            for (int j = 0; j < approach.inst.J(); j++) {
                //double fixed_delta = approach.Delta();        //FRR
                
                // calculate the minimum distance in the path from s to s+1
                double distance = Double.POSITIVE_INFINITY;
                for(int t=approach.Waypoint(s); t<approach.Waypoint(s+1); t++){
                    distance = Math.min(distance, approach.inst.obstacles[j].distance(Xt[t]));
                }
                if(s+1==approach.Steps()){
                    int t = approach.Waypoints();
                    distance = Math.min(distance, approach.inst.obstacles[j].distance(Xt[t]));
                }
                
                for(int i=0; i<approach.inst.Gj(j); i++){
                    ord.add(new uDataUp<RelaxVar>(new RelaxVar(model.Zs(j, s, i)), distance));
                }
            }
        }
        uDataUp<RelaxVar> R[] = ord.toArray(new uDataUp[ord.size()]);
        uSort.sort(R);
       
        
        ArrayList<RelaxVar> list = new ArrayList<RelaxVar>();
        for(uDataUp<RelaxVar> d : R){
            list.add(d.data);
        }
        return list;
    }

    @Override
    public void print3() throws IloException{
        System.out.printf("------------------------------[xx]------------------------------\n");
        System.out.printf("cost = %20g\n", cost());
        System.out.printf("sol  = [ \n");
        
        uDataUp R[] = new uDataUp[relax_variables.size()];
        for(int j=0; j<relax_variables.size(); j++){
            R[j] = new uDataUp(relax_variables.get(j), relax_variables.get(j).var.getName());
        }
        //uSort.sort(R);
//        model.obst.clear();
        int i=0;
        uDataUp<RelaxVar> before = null;
        for(uDataUp<RelaxVar> v : R){
            if(before==null || v.data != before.data){
                if(free.contains(v.data.var)){
                    System.out.printf("(%5s %8.5f %s) ", v.data.var, cpx.getValue(v.data.var), "*");
                    

//                    for (int j = 0; j < approach.inst.J(); j++) {
//                        boolean flag = false;
//                        for (int s = 0; s < approach.Steps()+ 1 && !flag; s++) {
//                            for(int k=0; k<approach.inst.Gj(j); k++){
//                                if(model.Zs(j, s, k).equals(v.data.var)){
//                                    flag = true;
//                                    break;
//                                }
//                            }
//                        }
//                        if(flag){
//                            model.obst.addLast(j);
//                        }
//                    }
                }else{
                    System.out.printf("(%5s %8.5f %s) ", v.data.var, cpx.getValue(v.data.var), " ");
                }

                if(++i % 80 == 0){
                    System.out.println();
                }
                
            }
            before = v;
        }
        System.out.println("]");
//        try {
//            approach.repaint(model);
//            Thread.sleep(2000); 
//        } catch (Exception ex) {
//            Logger.getLogger(Blackmore_RFFO.class.getName()).log(Level.SEVERE, null, ex);
//        }
    }
    
    @Override
    public void extra_conversion() throws IloException {
        
    }
    @Override
    public void model() throws Exception {
        model = (BlackmoreModel) full_op.build_model(approach, cpx, true);
    }
    
    @Override
    public void results(LinkerResults link) throws Exception {
        super.results(link); //To change body of generated methods, choose Tools | Templates.
        model.results(link);
        model.save();
    }
    @Override
    public void print() throws Exception{
        System.out.println("Solution status           = " + cpx.getStatus());
        System.out.println("Solution value            = " + cpx.getObjValue());
    }
}
