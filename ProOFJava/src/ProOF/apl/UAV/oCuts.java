/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV;

import ProOF.apl.UAV.Swing.Graphics2DReal;
import ProOF.apl.UAV.abst.UAVApproach;
import ProOF.apl.UAV.abst.UAVModel;
import ProOF.opt.abst.problem.meta.codification.Operator;

/**
 *
 * @author marcio
 * @param <App>
 * @param <Model>
 */
public abstract class oCuts<App extends UAVApproach, Model extends UAVModel> extends Operator{
    public abstract boolean [][] add_cuts(App approach, Model model) throws Exception;
    public abstract void paint(Graphics2DReal gr) throws Exception;
}
