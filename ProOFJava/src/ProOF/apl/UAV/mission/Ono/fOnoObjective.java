/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.mission.Ono;

import ProOF.apl.UAV.gen.linear.mission.parts.*;
import ProOF.apl.UAV.gen.linear.mission.parts.fLinearObjective.Norm1Ut;
import ProOF.apl.UAV.gen.linear.mission.parts.fLinearObjective.Norm2Ut2D;
import ProOF.apl.UAV.gen.linear.mission.parts.fLinearObjective.ScalProdUtAprox;
import ProOF.apl.UAV.gen.linear.mission.parts.fLinearObjective.ScalProdUtCplex;
import ProOF.com.language.Factory;

/**
 *
 * @author marcio
 */
public class fOnoObjective extends Factory<oLinearObjective>{
    public static final fOnoObjective obj = new fOnoObjective();
    @Override
    public String name() {
        return "Objective";
    }
    @Override
    public oLinearObjective build(int index) {  //build the operators
        switch(index){
            case 0: return new Norm1Ut();
            case 1: return new Norm2Ut2D();
            case 2: return new ScalProdUtAprox();  
            case 3: return new ScalProdUtCplex();
            case 4: return new Time();
        }
        return null;
    }
    private class Time extends oLinearObjective<OnoApproach, OnoModel>{
        @Override
        public String name() {
            return "Te";
        }
        @Override
        public void addObjective(OnoApproach approach, OnoModel model) throws Exception {
            model.cplex.addMinimize(model.Te[model.Te.length-1]);
        }
    }
    
}
