/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.com.language;

import ProOF.com.runner.FactoryChoise;
import java.util.ArrayList;
import java.util.LinkedList;

/**
 *
 * @author marcio
 * @param <No>
 */
public abstract class Factory<No extends Approach> {
    
    private Approach[] array;
    public final Approach[] split() throws Exception{
        if(array==null){
            LinkedList<Approach> list = new LinkedList<Approach>();
            int index = 0;
            Approach node = build(index);
            while(node!=null){
                list.addLast(node);
                index++;
                node = build(index);
            }
            for(int i=index; i<2*index; i++){
                if(build(i)!=null){
                    throw new Exception("There is some empty index or jump in factory "+name()+":\n\t"+class_name());
                }
            }
            array = list.toArray(new Approach[list.size()]);
        }
        return array;
    }
    
    public abstract String name();
    
    public final String class_name(){
        return this.getClass().getName();
    }
    public abstract No build(int index) throws Exception;

    private ArrayList<FactoryChoise<No>> choises = new ArrayList<FactoryChoise<No>>();
    public final No build_runner(String class_name, String name, int index) throws Exception{
        No choise = build(index);
        choises.add(new FactoryChoise<No>(class_name, name, index, choise));
        return choise;
    }
    public final ArrayList<FactoryChoise<No>> choises(){
        return choises;
    }
            
    @Override
    public final String toString() {
        return name();
    }
}
