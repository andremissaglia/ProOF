/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF;

import java.util.LinkedList;

/**
 *
 * @author marcio
 */
public class Lambda {

    static interface Select<T> {
        T operation(T a, T b);
    }
    
    
    public static void main(String[] args) {
        Lambda set[] = new Lambda[]{new Lambda(), new Lambda(), new Lambda()};
        for(Lambda l : set){
            
        }
        
        Select<Double> Smin = (a, b) -> Math.min(a, b);
        
        double min = select(Smin, 2.4, 0.2, 1.3);
        
        System.out.println(min);

    }
    
    
    private static <T> T select(Select<T> oper, T... data){
        T selected = data[0];
        for(int i=1; i<data.length; i++){
            selected = oper.operation(selected, data[i]);
        }
        return selected;       
    }
    
    
}
