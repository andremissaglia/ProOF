/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.HCCQSP.var;

import ProOF.opt.abst.problem.meta.codification.Operator;

/**
 *
 * @author marcio
 */
public class oControl extends Operator{
    public final String name;
    public oControl(String name) {
        this.name = name;
    }
    @Override
    public final String name() {
        return name;
    }
}
