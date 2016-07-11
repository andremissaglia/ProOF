/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.problem.Blackmore;

import ProOF.com.language.Factory;
import ProOF.gen.operator.oCrossover;
import ProOF.gen.operator.oInitialization;
import ProOF.gen.operator.oLocalMove;
import ProOF.gen.operator.oMutation;
import ProOF.opt.abst.problem.meta.codification.Operator;
import java.util.Random;

/**
 *
 * @author marcio
 */
public class BlackmoreOperatorProblem extends Factory<Operator>{
    public static final BlackmoreOperatorProblem obj = new BlackmoreOperatorProblem();
    
    @Override
    public String name() {
        return "BlackmoreOperatorProblem";
    }
    @Override
    public Operator build(int index) {  //build the operators
        switch(index){
            case 0: return new RandomTour();    //initialization
            case 1: return new MutExchange();   //mutation
            case 2: return new Uniform();     //crossover
            case 3: return new MovExchange();   //local movement
        }
        return null;
    }
    
    private class RandomTour extends oInitialization<BlackmoreProblem, BlackmoreCodification>{
        @Override
        public String name() {
            return "Random Tour";
        }
        @Override
        public void initialize(BlackmoreProblem prob, BlackmoreCodification ind) throws Exception {
            ind.path.clear();
            int size = prob.rnd.nextInt(prob.map.graph.vertexes.length+1);
            for(int i=0; i<size; i++){
                ind.path.add(prob.rnd.nextInt(prob.map.graph.vertexes.length));
            }
        }
    }
    private class MutExchange extends oMutation<BlackmoreProblem, BlackmoreCodification>{
        @Override
        public String name() {
            return "Mut-Exchange";
        }
        @Override
        public void mutation(BlackmoreProblem prob, BlackmoreCodification ind) throws Exception {
            random_swap(prob.rnd, ind);
        }
    }
    private class MovExchange extends oLocalMove<BlackmoreProblem, BlackmoreCodification>{
        @Override
        public String name() {
            return "Mov-Exchange";
        }
        @Override
        public void local_search(BlackmoreProblem prob, BlackmoreCodification ind) throws Exception {
            random_swap(prob.rnd, ind);
        }
    }
    private class Uniform extends oCrossover<BlackmoreProblem, BlackmoreCodification>{
        @Override
        public String name() {
            return "Uniform";
        }
        @Override
        public BlackmoreCodification crossover(BlackmoreProblem prob, BlackmoreCodification ind1, BlackmoreCodification ind2) throws Exception {
            BlackmoreCodification child = ind1.build(prob);
            
            int min_size = Math.min(ind1.path.size(), ind2.path.size());
            int max_size = Math.max(ind1.path.size(), ind2.path.size());
            
            for(int i=0; i<min_size; i++){
                if(prob.rnd.nextBoolean()){
                    child.path.add(ind1.path.get(i));
                }else{
                    child.path.add(ind2.path.get(i));
                }
            }
            for(int i=min_size; i<max_size; i++){
                if(prob.rnd.nextBoolean()){
                    if(i<ind1.path.size()){
                        child.path.add(ind1.path.get(i));
                    }else{
                        child.path.add(ind2.path.get(i));
                    }
                }
            }
            return child;
        }
    }
    
    private static void random_swap(Random rmd, BlackmoreCodification ind){
        if(ind.path.size()>1){
            int a =  rmd.nextInt(ind.path.size());
            int b =  rmd.nextInt(ind.path.size());
            int v1 = ind.path.get(a);
            int v2 = ind.path.get(b);
            ind.path.set(a, v2);
            ind.path.set(b, v1);
        }
    }
}
