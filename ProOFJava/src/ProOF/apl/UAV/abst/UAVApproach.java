/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.abst;

import ProOF.com.Linker.LinkerApproaches;
import ProOF.com.Linker.LinkerParameters;
import ProOF.com.Linker.LinkerResults;
import ProOF.com.Linker.LinkerValidations;
import ProOF.com.language.Approach;

/**
 *
 * @author marcio
 * @param <Model>
 */
public abstract class UAVApproach<Model extends UAVModel> extends Approach implements UAVSystem{

    @Override
    public String description() {
        throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }
    @Override
    public void services(LinkerApproaches link) throws Exception {
        
    }
    @Override
    public void parameters(LinkerParameters link) throws Exception {
        
    }
    
    @Override
    public void load() throws Exception {
        
    }
    @Override
    public void start() throws Exception {
        
    }
    @Override
    public boolean validation(LinkerValidations link) throws Exception {
        throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }
    
    @Override
    public void results(LinkerResults link) throws Exception {
        
    }
    public void solve(Model model) throws Exception {
        model.solve();
    }
}
