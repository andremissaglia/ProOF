/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV;

import ProOF.CplexExtended.CplexExtended;
import ProOF.apl.UAV.abst.UAVModel;
import ProOF.apl.UAV.abst.UAVSystem;
import ProOF.opt.abst.problem.meta.codification.Operator;
import java.util.ArrayList;

/**
 *
 * @author marcio
 * @param <App>
 * @param <Model>
 */
public abstract class oFull<App extends UAVSystem, Model extends UAVModel> extends Operator{
    public final Model build_model(App approach, CplexExtended cplex) throws Exception{
        return build_model(approach, cplex, false);
    }
    
    public abstract Model build_model(App approach, CplexExtended cplex, boolean isRelax) throws Exception;
    
    private final ArrayList<String> names = new ArrayList<String>();
    private final ArrayList<Integer> ids = new ArrayList<Integer>();
    protected final String id_name(String base){
        int index = names.indexOf(base);
        if(index == -1){
            names.add(base);
            ids.add(1);
            return base;
        }else{
            int new_id = ids.get(index)+1;
            ids.set(index, new_id);
            return base+"("+new_id+")";
        }
    }
    protected final String id_name(){
        return id_name(name());
    }
}
