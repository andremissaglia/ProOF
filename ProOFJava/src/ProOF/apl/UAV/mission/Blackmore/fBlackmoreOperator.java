/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.mission.Blackmore;

import ProOF.CplexExtended.CplexExtended;
import ProOF.apl.UAV.oCSA;
import ProOF.apl.UAV.oCuts;
import ProOF.apl.UAV.oFull;
import ProOF.apl.UAV.oRebuild;
import ProOF.com.Linker.LinkerApproaches;
import ProOF.com.language.Factory;
import ProOF.opt.abst.problem.meta.codification.Operator;

/**
 *
 * @author marcio
 */
public class fBlackmoreOperator extends Factory<Operator>{
    public static final fBlackmoreOperator obj = new fBlackmoreOperator();
    @Override
    public String name() {
        return "Blackmore Operators";
    }
    @Override
    public Operator build(int index) {  //build the operators
        switch(index){
            case 0: return new RAA();
            case 1: return new FRR();
            case 2: return new FRT();
            case 3: return new CSA();
            case 4: return new CSA1();
            case 5: return new CSA2();
            case 6: return new CSA3();
            case 7: return new RebuildRAA();
        }
        return null;
    }
    private abstract class Full extends oFull<BlackmoreApproach, BlackmoreModel>{
        protected oBlackmoreAvoidance avoid;
        protected oCuts cuts;
        
        @Override
        public void services(LinkerApproaches link) throws Exception {
            super.services(link); //To change body of generated methods, choose Tools | Templates.
            avoid = link.get(fBlackmoreAvoidance.obj, avoid);
            cuts = link.get(fBlackmoreCuts.obj, cuts);
        }
        public abstract BlackmoreModel build(BlackmoreApproach approach, CplexExtended cplex, boolean isRelax) throws Exception;
        @Override
        public final BlackmoreModel build_model(BlackmoreApproach approach, CplexExtended cplex, boolean isRelax) throws Exception {
            BlackmoreModel model = build(approach, cplex, isRelax);
            boolean isO[][] = cuts.add_cuts(approach, model);
            for(int j=0; j<approach.inst.J(); j++){
                for(int s=0; s<approach.Steps()+1; s++){
                    if(isO==null || isO[j][s]){
                        avoid.OandN(approach, model, j, s);
                    }
                }
            }
            //cuts.add_cuts(approach, model);
            return model;
        }
    }
    
    public class RAA extends Full{
        @Override
        public String name() {
            return "RAA";
        }
        @Override
        public BlackmoreModel build(BlackmoreApproach approach, CplexExtended cplex, boolean isRelax) throws Exception {
            return new BlackmoreModelRiskAlloc(approach, id_name(), cplex, approach.Delta(), isRelax);
        }
    }
    private class FRR extends Full{
        @Override
        public String name() {
            return "FRR";
        }
        @Override
        public BlackmoreModel build(BlackmoreApproach approach, CplexExtended cplex, boolean isRelax) throws Exception {
            double fixed_delta = approach.Delta();   //FRR
            return new BlackmoreModelRiskFixed(approach, id_name(), cplex, fixed_delta, isRelax);
        }
    }
    private class FRT extends Full{
        @Override
        public String name() {
            return "FRT";
        }
        @Override
        public BlackmoreModel build(BlackmoreApproach approach, CplexExtended cplex, boolean isRelax) throws Exception {
            double fixed_delta = approach.Delta()/(approach.inst.J()*(approach.Waypoints()+1));   //FRT
            return new BlackmoreModelRiskFixed(approach, id_name(), cplex, fixed_delta, isRelax);
        }
    }
    
    private abstract class RebuildFull extends oRebuild<BlackmoreApproach, BlackmoreModel>{
        protected oBlackmoreAvoidance avoid;
        protected oCuts cuts;
        
        @Override
        public void services(LinkerApproaches link) throws Exception {
            super.services(link); //To change body of generated methods, choose Tools | Templates.
            avoid = link.get(fBlackmoreAvoidance.obj, avoid);
            cuts = link.get(fBlackmoreCuts.obj, cuts);
        }
        public abstract BlackmoreModel build(BlackmoreApproach approach, CplexExtended cplex, BlackmoreModel from) throws Exception;

        @Override
        public BlackmoreModel rebuild_model(BlackmoreApproach approach, CplexExtended cplex, BlackmoreModel from) throws Exception {
            BlackmoreModel model = build(approach, cplex, from);
            boolean isO[][] = cuts.add_cuts(approach, model);
            for(int j=0; j<approach.inst.J(); j++){
                for(int s=0; s<approach.Steps()+1; s++){
                    if(isO==null || isO[j][s]){
                        avoid.OandN(approach, model, j, s);
                    }
                }
            }
            //cuts.add_cuts(approach, model);
            return model;
        }
        
    }
    
    private class RebuildRAA extends RebuildFull{
        @Override
        public String name() {
            return "RE-RAA";
        }
        @Override
        public BlackmoreModel build(BlackmoreApproach approach, CplexExtended cplex, BlackmoreModel from) throws Exception {
            return new BlackmoreModelRiskAlloc(approach, "RAA", cplex, from==null ? approach.Delta() : from.delta_to_cut/2, false);
        }
    }
    
    private class CSA extends oCSA<BlackmoreApproach, BlackmoreModel>{
        protected oBlackmoreAvoidance avoid;
        protected oCuts cuts;
        @Override
        public String name() {
            return "CSA";
        }
        @Override
        public void services(LinkerApproaches link) throws Exception {
            super.services(link); //To change body of generated methods, choose Tools | Templates.
            avoid = link.get(fBlackmoreAvoidance.obj, avoid);
            cuts = link.get(fBlackmoreCuts.obj, cuts);
        }
        @Override
        public BlackmoreModel build_RAA(BlackmoreApproach approach, CplexExtended cplex, BlackmoreModel toFix) throws Exception {
            BlackmoreModelRiskAlloc model = new BlackmoreModelRiskAlloc(approach, id_name("RAA"), cplex, approach.Delta(), false);
            for(int j=0; j<approach.inst.J(); j++){
                for(int s=0; s<approach.Steps()+1; s++){
                    avoid.OandN(approach, model, j, s);
                }
            }
            model.fixWith(toFix);
            return model;
        }
        @Override
        public BlackmoreModel build_FRR(BlackmoreApproach approach, CplexExtended cplex) throws Exception {
            double fixed_delta = approach.Delta();
            return build_fixed(id_name("FRR"), approach, cplex, fixed_delta);
        }
        @Override
        public BlackmoreModel build_FRT(BlackmoreApproach approach, CplexExtended cplex) throws Exception {
            double fixed_delta = approach.Delta()/(approach.inst.J()*(approach.Waypoints()+1));   //FRT
            return build_fixed(id_name("FRT"), approach, cplex, fixed_delta);
        }
        
        public BlackmoreModel build_fixed(String name, BlackmoreApproach approach, CplexExtended cplex, double fixed_delta) throws Exception{
            BlackmoreModel model = new BlackmoreModelRiskFixed(approach, name, cplex, fixed_delta, false);
            
            boolean isO[][] = cuts.add_cuts(approach, model);
            
            for(int j=0; j<approach.inst.J(); j++){
                for(int s=0; s<approach.Steps()+1; s++){
                    if(isO==null || isO[j][s]){
                        avoid.O(approach, model, j, s);
                    }
                }
            }
            for(int j=0; j<approach.inst.J(); j++){
                for(int s=0; s<approach.Steps()+1; s++){
                    avoid.N(approach, model, j, s);
                }
            }
            
            return model;
        }
    }
    private class CSA1 extends CSA{
        @Override
        public String name() {
            return "CSA1";
        }
        @Override
        public BlackmoreModel build_fixed(String name, final BlackmoreApproach approach, CplexExtended cplex, double fixed_delta) throws Exception{
            BlackmoreModel model = new BlackmoreModelRiskFixed(approach, name, cplex, fixed_delta, false) { 
                @Override
                public boolean addChanges() throws Exception {
                    int [][][] vZjti = getTrueZjsi();
                    boolean isChange = false;
                    //System.out.println("-------------- add changeO ------------");
                    for (int j = 0; j < approach.inst.J(); j++) {
                        for (int s = 1; s < system.Steps()+ 1; s++) {
                            if(avoid.infeasibleNs(vZjti, j, s, approach)){
                                if(avoid.N(approach, this, j, s)){
                                    isChange = true;
                                }
                            }
                        }
                    }
//                    if(isChange){
//                        System.out.println("is Change: (press key)");
//                        //System.in.read();
//                    }else{
//                        System.out.println("is not Change: (continue)");
//                    }
                    
                    return isChange;
                }
            };
            cuts.add_cuts(approach, model);
            for(int j=0; j<approach.inst.J(); j++){
                for(int s=0; s<approach.Steps()+1; s++){
                    avoid.O(approach, model, j, s);
                }
            }
            return model;
        }
    }
    
    private class CSA2 extends CSA{
        @Override
        public String name() {
            return "CSA2";
        }
        
        protected void addFristBlock(BlackmoreSystem approach, BlackmoreModel model, int j, int s) throws Exception{
            final int N = 2;
            for(int k=s+1; k<=s+N && k<approach.Steps()+1; k++){
                avoid.O(approach, model, j, k);
            }
            for(int k=s-1; k>=s-(N+1) && k>=1; k--){
                avoid.O(approach, model, j, k);
            }
            avoid.N(approach, model, j, s);
            for(int k=s+1; k<=s+N && k<approach.Steps()+1; k++){
                avoid.N(approach, model, j, k);
            }
            for(int k=s-1; k>=s-N && k>=1; k--){
                avoid.N(approach, model, j, k);
            }
        }
        protected void addSecondBlock(BlackmoreSystem approach, BlackmoreModel model, int j, int s) throws Exception{
            final int N = 2;
            for(int k=s+1; k<=s+N && k<approach.Steps()+1; k++){
                avoid.O(approach, model, j, k);
            }
            for(int k=s-1; k>=s-(N+1) && k>=1; k--){
                avoid.O(approach, model, j, k);
            }
            avoid.N(approach, model, j, s);
            for(int k=s+1; k<=s+N && k<approach.Steps()+1; k++){
                avoid.N(approach, model, j, k);
            }
            for(int k=s-1; k>=s-N && k>=1; k--){
                avoid.N(approach, model, j, k);
            }
        }
        
        @Override
        public BlackmoreModel build_fixed(String name, final BlackmoreApproach approach, CplexExtended cplex, final double fixed_delta) throws Exception{
            BlackmoreModel model = new BlackmoreModelRiskFixed(approach, name, cplex, fixed_delta, false) {
                @Override
                public boolean addChanges() throws Exception {
                    if(addChangesO()){
                        return true;
                    }
                    return addChangesN();
                }
                private boolean addChangesO() throws Exception{
                    double Xt[][] = getXt();
                    
                    //System.out.println("-------------- add changeO ------------");
                    boolean addFristBlock[][] = new boolean[system.inst().J()][system.Steps()+1];
                    boolean isChange = false;
                    for (int j = 0; j < system.inst().J(); j++) {
                        for (int t = 0; t < system.Waypoints() + 1; t++) {
                            if(avoid.infeasibleOt(Xt, j, t, fixed_delta, approach)){
                                int s = system.Step(t);
                                if(avoid.O(approach, this, j, s)){
                                    isChange = true;
                                    addFristBlock[j][s] = true;
                                }
                            }
                        }
                    }
                    //System.out.println("-------------- add frist block ------------");
                    for (int j = 0; j < system.inst().J(); j++) {
                        for (int s = 0; s < system.Steps()+ 1; s++) {
                            if(addFristBlock[j][s]){
                                addFristBlock(system, this, j, s);
                            }
                        }
                    }
                    
//                    if(isChange){
//                        System.out.println("is Change: (press key)");
//                        //System.in.read();
//                    }else{
//                        System.out.println("is not Change: (continue)");
//                    }
                    return isChange;
                }
                private boolean addChangesN() throws Exception{
                    //System.out.println("-------------- add changeN ------------");
                    int [][][] vZjti = getTrueZjsi();
                    boolean addSecondBlock[][] = new boolean[system.inst().J()][system.Steps()+1];
                    
                    boolean isChange = false;
                    for (int j = 0; j < system.inst().J(); j++) {
                        for (int s = 1; s < system.Steps()+ 1; s++) {
                            if(avoid.infeasibleNs(vZjti, j, s, approach)){
                                if(avoid.N(approach, this, j, s)){
                                    //System.out.println("*");
                                    isChange = true;
                                    addSecondBlock[j][s] = true;
                                }
                            }
                        }
                    }
                    //System.out.println("-------------- add frist block ------------");
                    for (int j = 0; j < system.inst().J(); j++) {
                        for (int s = 0; s < system.Steps()+ 1; s++) {
                            if(addSecondBlock[j][s]){
                                addSecondBlock(system, this, j, s);
                            }
                        }
                    }
                    return isChange;
                }
            };
            return model;
        }
    }
    
    private class CSA3 extends CSA2{
        @Override
        public String name() {
            return "CSA3";
        }
        @Override
        protected void addFristBlock(BlackmoreSystem approach, BlackmoreModel model, int j, int s) throws Exception {
            
        }
        @Override
        protected void addSecondBlock(BlackmoreSystem approach, BlackmoreModel model, int j, int s) throws Exception {
            avoid.O(approach, model, j, s);
            if(s>0){
                avoid.O(approach, model, j, s-1);
            }
        }
    }
}
