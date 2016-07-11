/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.utilities;

import java.util.ArrayList;
import java.util.Random;

/**
 *
 * @author marcio
 */
public class uRouletteList {
    private final Random rmd;

    private double sum;
    private final ArrayList<Double> weigth;
    private final ArrayList<Integer> indexes;
    
    
    public uRouletteList(Random rmd) {
        this.rmd = rmd;
        this.sum = 0;
        this.weigth = new ArrayList<>();
        this.indexes = new ArrayList<>();
    }
    public void clear(){
        this.sum = 0;
        this.weigth.clear();
        this.indexes.clear();
    }
    public void add(double weigth, Integer index){
        this.weigth.add(weigth);
        this.indexes.add(index);
        this.sum += weigth;
    }
    public double probability(Integer index){
        return weigth.get(indexes.indexOf(index))/sum;
    }
    public Integer roulette_wheel(){
        double x = rmd.nextDouble()*sum;
        for(int i=0; i<weigth.size(); i++){
            if(x>weigth.get(i)){
                x = x - weigth.get(i);
            }else{
                return indexes.get(i);
            }
        }
        throw new ArithmeticException("Algorithm roulette incorrect.");
    }
}
