/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.karla.model.RealeImaginario;

import ProOF.CplexOpt.CplexFull;
import ProOF.com.Linker.LinkerApproaches;
import ProOF.com.Linker.LinkerParameters;
import ilog.concert.IloException;
import java.io.File;
import java.util.Scanner;


/**
 *
 * @author Administrador
 */
public class RICplexFix extends CplexFull{
    public  RIModel model;
    public  RIInstance inst = new RIInstance();
    private File file;
    
    
    public RICplexFix() throws IloException {
       
    }
    
    
    @Override
    public String name() {
        return "RICplexFix";
    }
    
        
    @Override
    public void services(LinkerApproaches link) throws Exception {
        super.services(link); //To change body of generated methods, choose Tools | Templates.
        link.add(inst);
    }

    @Override
    public void parameters(LinkerParameters link) throws Exception {
        super.parameters(link); 
        file = link.File("ConfFinal", null, "txt");
    }
     
    

    @Override
    public void model() throws Exception {
        model = new RIModel(cpx, inst, false);
        for(int i=0; i<inst.Nnos; i++){
            for(int j=0; j<inst.Nnos; j++){
                if(inst.Rr[i][j]>0.0000001){
                    model.x[i][j].setUB(0);
                }
            }
        }
        Scanner sc = new Scanner (file);
        int L = sc.nextInt();
        for(int n=0; n<L; n++){
            int NoInicial = sc.nextInt () - 1;
            int NoFinal = sc.nextInt () - 1;
            model.x[NoInicial][NoFinal].setLB(1);
            model.x[NoInicial][NoFinal].setUB(1);
        }
        sc.close();
    }

    
    
    
    @Override
    public void print() throws Exception {
        super.print(); //To change body of generated methods, choose Tools | Templates
        model.print();
    }

    
    
}
