/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.mission.Blackmore;

import ProOF.com.language.Factory;
import ilog.concert.IloException;
import ilog.concert.IloNumExpr;
import ilog.concert.IloNumVar;

/**
 *
 * @author marcio
 */
public class fBlackmoreAvoidance extends Factory<oBlackmoreAvoidance>{
    public static final fBlackmoreAvoidance obj = new fBlackmoreAvoidance();
    @Override
    public String name() {
        return "Blackmore Avoidance";
    }
    @Override
    public oBlackmoreAvoidance build(int index) {  //build the operators
        switch(index){
            case 0: return new Model1();
            case 1: return new Model2();
            case 2: return new Model3();
            //case 3: return null;//new Model4();
            //case 4: return null;//new Model5();
            case 3: return new Model6();
            case 4: return new Model7();
            case 5: return new ModelChengGE();
            case 6: return new ModelChengEQ();
            case 7: return new ModelChengGE2();
            case 8: return new ModelChengEQ2();
        }
        return null;
    }
    private class Model1 extends oBlackmoreAvoidance<BlackmoreSystem, BlackmoreModel>{
        @Override
        public String name() {
            return "Sum{Z} ge 1";
        }
        public void addAvoid(BlackmoreSystem approach, BlackmoreModel model, int j, int s) throws IloException{
            IloNumExpr exp = null;
            for(int i=0; i<approach.inst().Gj(j); i++){
                exp = model.cplex.SumProd(exp, 1.0, model.Zs(j,s,i));
            }
            model.cplex.addGe(exp, 1, "Avoid");
        }
        @Override
        public boolean O(BlackmoreSystem approach, BlackmoreModel model, int j, int s) throws Exception {
            if(model.addOs(j,s)){
                //System.out.printf("add O(%2d %2d)\n", j,t);
                addAvoid(approach, model, j, s);
                return true;
            }
            return false;
        }
        @Override
        public boolean N(BlackmoreSystem approach, BlackmoreModel model, int j, int s) throws Exception {
            return false;
        }
        @Override
        public void OandN(BlackmoreSystem approach, BlackmoreModel model, int j, int s) throws Exception {
            O(approach, model, j, s);
            N(approach, model, j, s);
        }
    }
    private class Model2 extends Model1{
        @Override
        public String name() {
            return "alpha ge |Zi| + |Zl| - 1";
        }
        
        public IloNumExpr module(BlackmoreModel model, int j, int s, int i, int l, int c1, int c2, int c3, int c4, double b) throws IloException{
            //System.out.printf("(%d  %d  %d  %d  |   %d  %d  %d  %d  |   %g)\n",  j,  t,  i,  l,  c1,  c2,  c3,  c4,  b);
            return model.cplex.sumArg(c1,model.Zs(j,s,i), c2,model.Zs(j,s-1,i), c3,model.Zs(j,s,l), c4,model.Zs(j,s-1,l), b);
        }
        
        @Override
        public boolean N(BlackmoreSystem approach, BlackmoreModel model, int j, int s) throws Exception {
            if(model.addNs(j,s)){
                //System.out.printf("add N(%2d %2d)\n", j,t);
                IloNumVar alpha[][] = model.cplex.numVarArray(approach.inst().Gj(j), approach.inst().Gj(j), 0.0, 1.0, "Alpha");
                for(int i=0; i<approach.inst().Gj(j); i++){
                    for(int l=0; l<approach.inst().Gj(j); l++){
                        if(i==l){
                            alpha[i][l].setUB(0);
                        }else{
                            model.cplex.addEq(alpha[i][l], alpha[l][i], "Alpha");
                        }
                    }
                }
                
                for (int i = 0; i < approach.inst().Gj(j); i++) {
                    for (int l = 0; l < approach.inst().Gj(j); l++) {
                        if (l != i) {
                            if(s==0){
                                model.cplex.addGe(alpha[i][l], model.cplex.Sum(model.Zs(j,s,i), model.Zs(j,s,l), -1.0), "Module");
                            }else{
                                model.cplex.addGe(alpha[i][l], module(model, j, s, i, l, +1, -1, +1, -1, -1.0), "Module");
                                model.cplex.addGe(alpha[i][l], module(model, j, s, i, l, +1, -1, -1, +1, -1.0), "Module");
                                model.cplex.addGe(alpha[i][l], module(model, j, s, i, l, -1, +1, +1, -1, -1.0), "Module");
                                model.cplex.addGe(alpha[i][l], module(model, j, s, i, l, -1, +1, -1, +1, -1.0), "Module");
                            }
                        }
                    }
                }
                
                IloNumExpr exp = null;
                for(int i=0; i<approach.inst().Gj(j); i++){
                    exp = model.cplex.SumProd(exp, 1.0, model.Zs(j,s,i));
                }
                for(int i=0; i<approach.inst().Gj(j); i++){
                    for(int l=0; l<approach.inst().Gj(j); l++){
                        exp = model.cplex.SumProd(exp, -1.0, alpha[i][l]);
                    }
                }
                model.cplex.addGe(exp, 1, "Avoid");
                return true;
            }
            return false;
        }
    }
    private class Model3 extends Model1{
        @Override
        public String name() {
            return "1 ge |Zi| + |Zl|";
        }
        public IloNumExpr module(BlackmoreModel model, int j, int s, int i, int l, int c1, int c2, int c3, int c4) throws IloException{
            //System.out.printf("(%d  %d  %d  %d  |   %d  %d  %d  %d  |   %g)\n",  j,  t,  i,  l,  c1,  c2,  c3,  c4,  b);
            return model.cplex.sumArg(c1,model.Zs(j,s,i), c2,model.Zs(j,s-1,i), c3,model.Zs(j,s,l), c4,model.Zs(j,s-1,l));
        }

        @Override
        public boolean N(BlackmoreSystem approach, BlackmoreModel model, int j, int s) throws Exception {
            if(model.addNs(j,s)){
                for (int i = 0; i < approach.inst().Gj(j); i++) {
                    for (int l = 0; l < approach.inst().Gj(j); l++) {
                        if (l != i) {
                            if(s==0){
                                model.cplex.addGe(1, model.cplex.Sum(model.Zs(j,s,i), model.Zs(j,s,l)), "Module");
                            }else{
                                model.cplex.addGe(1, module(model, j, s, i, l, +1, -1, +1, -1), "Module");
                                model.cplex.addGe(1, module(model, j, s, i, l, +1, -1, -1, +1), "Module");
                                model.cplex.addGe(1, module(model, j, s, i, l, -1, +1, +1, -1), "Module");
                                model.cplex.addGe(1, module(model, j, s, i, l, -1, +1, -1, +1), "Module");
                            }
                        }
                    }
                }
                return true;
            }
            return false;
        }
    }
    private class Model6 extends Model1{
        @Override
        public String name() {
            return "sum{ Z(t) and Z(t-1) } = 1";
        }
        @Override
        public boolean N(BlackmoreSystem approach, BlackmoreModel model, int j, int s) throws Exception {
            if(model.addNs(j,s)){
                if(s>0){
                    IloNumExpr sum = null;
                    for (int i = 0; i < approach.inst().Gj(j); i++) {
                        sum = model.cplex.SumProd(sum, 1.0, model.cplex.And("And("+i+")", model.Zs(j,s,i), model.Zs(j,s-1,i)));
                    }
                    model.cplex.addEq(sum, 1, "Or6["+j+"]["+s+"]"); 
                }
                return true;
            }
            return false;
        }
        @Override
        public void addAvoid(BlackmoreSystem approach, BlackmoreModel model, int j, int t) throws IloException{
            
        }
    }
    private class Model7 extends Model1{
        @Override
        public String name() {
            return "sum{ Z(t) and Z(t-1) } ge 1";
        }
        @Override
        public boolean N(BlackmoreSystem approach, BlackmoreModel model, int j, int s) throws Exception {
            if(model.addNs(j,s)){
                if(s>0){
                    IloNumExpr sum = null;
                    for (int i = 0; i < approach.inst().Gj(j); i++) {
                        sum = model.cplex.SumProd(sum, 1.0, model.cplex.And("And("+i+")", model.Zs(j,s,i), model.Zs(j,s-1,i)));
                    }
                    model.cplex.addGe(sum, 1, "Or6["+j+"]["+s+"]"); 
                }
                return true;
            }
            return false;
        }
        @Override
        public void addAvoid(BlackmoreSystem approach, BlackmoreModel model, int j, int t) throws IloException{
            
        }
    }
    
    private class ModelChengGE extends Model1{
        @Override
        public String name() {
            return "Cheng GE";
        }
        @Override
        public boolean N(BlackmoreSystem approach, BlackmoreModel m, int j, int s) throws Exception {
            if(m instanceof BlackmoreModelRiskAlloc){
                BlackmoreModelRiskAlloc model = ((BlackmoreModelRiskAlloc)m);
                if(m.addNs(j,s)){
                    if(s>0){
                        for(int i=0; i<approach.inst().Gj(j); i++){
                            //c_(i,t) (δ_jt ) + b_ji - a_ji^T*μ_t ≤ M*(1- Z_jti )
                            //c_(i,t) (δ_jt ) + M*Z_jti - a_ji^T*μ_t ≤ M - b_ji
                            IloNumExpr exp = model.cplex.sum(model.Cjti[j][s][i], model.cplex.prod(approach.inst().bigM(), model.Zs(j, s, i)));
                            for(int n=0; n<approach.N(); n++){
                                exp = model.cplex.SumProd(exp, -1.0, model.cplex.prod(approach.inst().a(j,i,n), model.states[s-1].x[n])); //The diference is here: from 'model.states[t]' to 'model.states[t-1]'
                            }
                            model.cplex.addLe(exp, approach.inst().bigM()-approach.inst().b(j, i), model.name);
                        }
                    }
                    return true;
                }
                return false;
            }else{
                throw new UnsupportedOperationException("Not supported yet.");
            }
        }
    }
    private class ModelChengEQ extends ModelChengGE{
        @Override
        public String name() {
            return "Cheng EQ";
        }
        @Override
        public void addAvoid(BlackmoreSystem approach, BlackmoreModel model, int j, int s) throws IloException{
            IloNumExpr exp = null;
            for(int i=0; i<approach.inst().Gj(j); i++){
                exp = model.cplex.SumProd(exp, 1.0, model.Zs(j,s,i));
            }
            model.cplex.addEq(exp, 1, "AvoidEQ");
        }
    }
    private class ModelChengGE2 extends Model1{
        @Override
        public String name() {
            return "Cheng' GE";
        }
        @Override
        public boolean N(BlackmoreSystem approach, BlackmoreModel m, int j, int s) throws Exception {
            if(m instanceof BlackmoreModelRiskAlloc){
                BlackmoreModelRiskAlloc model = ((BlackmoreModelRiskAlloc)m);
                if(m.addNs(j,s)){
                    if(s>0){
                        for(int i=0; i<approach.inst().Gj(j); i++){
                            //c_(i,t) (δ_jt ) + b_ji - a_ji^T*μ_t ≤ M*(1- Z_jti )
                            //c_(i,t) (δ_jt ) + M*Z_jti - a_ji^T*μ_t ≤ M - b_ji
                            IloNumExpr exp = model.cplex.sum(model.Cjti[j][s-1][i], model.cplex.prod(approach.inst().bigM(), model.Zs(j, s, i)));
                            for(int n=0; n<approach.N(); n++){
                                exp = model.cplex.SumProd(exp, -1.0, model.cplex.prod(approach.inst().a(j,i,n), model.states[s-1].x[n])); //The diference is here: from 'model.states[t]' to 'model.states[t-1]'
                            }
                            model.cplex.addLe(exp, approach.inst().bigM()-approach.inst().b(j, i), model.name);
                        }
                    }
                    return true;
                }
                return false;
            }else{
                throw new UnsupportedOperationException("Not supported yet.");
            }
        }
    }
    private class ModelChengEQ2 extends ModelChengGE2{
        @Override
        public String name() {
            return "Cheng' EQ";
        }
        @Override
        public void addAvoid(BlackmoreSystem approach, BlackmoreModel model, int j, int s) throws IloException{
            IloNumExpr exp = null;
            for(int i=0; i<approach.inst().Gj(j); i++){
                exp = model.cplex.SumProd(exp, 1.0, model.Zs(j,s,i));
            }
            model.cplex.addEq(exp, 1, "AvoidEQ");
        }
    }
}

