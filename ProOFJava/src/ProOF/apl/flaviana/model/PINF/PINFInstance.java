/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.flaviana.model.PINF;

import ProOF.com.Linker.LinkerParameters;
import ProOF.opt.abst.problem.Instance;
import ProOF.utilities.uIO;
import java.io.File;
import java.util.Scanner;

/**
 *
 * @author Flaviana
 */
public class PINFInstance extends Instance{

    private File file;
    /**numero de periodos*/
    public int T;
    /**numero de maquinas*/
    public int M;
    /**numero maximos de maquinas a serem instaladas*/
    public int Ma;
    //public int K;//numero de cidades
    /**quantidades de produtos*/
    public int P;
    /**numero de secoes por maquinas*/
    public int NSm[];
    /**tipo de gotas  por maquinas*/
    public double TGm[];
    /**custo de intalacao por maquinas*/
    public double Cm[];
    /**custo de tonelada por fusao do forno*/
    public double CF;
    /**peso do produto*/
    public double Wp[];
    /**rendimento por cavidade do produto*/
    public double Rp[];
    /**aceitaabilidade do produto na maquina*/
    public double ACpm[][];
    /**eficiencia por maquinas*/
    public double Nm[];
    /**demanda do  produto no periodo*/
    public double Dpt[][];

    
    /**Cij = Matriz de custo de ir entre a cidade i para cidade j*/
    //public double Cij[][];//matriz de custo entre as cidades
    
    @Override
    public String name() {
        //throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
        return "PINFInstance";
    }

    @Override
    public void parameters(LinkerParameters link) throws Exception {
        //throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
        file = link.File("File for TSP", null, "dat");
    }

    @Override
    public void load() throws Exception {
        //throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
        Scanner sc  = new Scanner(file);
        //na ordem da leitura
              
        T = uIO.ReadIntOpl(sc);//numero de periodos
        M = uIO.ReadIntOpl(sc);//numero de maquinas
        Ma = uIO.ReadIntOpl(sc);//numero maximos de maquinas a serem instaladas
        System.out.println(sc.nextLine());
        
//uIO.Ignore(sc);//public int K;//numero de cidades
        P = uIO.ReadIntOpl(sc);//quantidades de produtos
        NSm = uIO.ReadVectorIntOpl(sc);//numero de secoes por maquinas
        TGm = uIO.ReadVectorDoubleOpl(sc);//tipo de gotas  por maquinas
        Cm = uIO.ReadVectorDoubleOpl(sc);//custo de intalacao por maquinas
        CF =  uIO.ReadDoubleOpl(sc);//custo de tonelada por fusao do forno
        Wp = uIO.ReadVectorDoubleOpl(sc);//peso do produto
        Rp = uIO.ReadVectorDoubleOpl(sc);//rendimento por cavidade do produto
        ACpm  = uIO.ReadMatrixDoubleOpl(sc);//aceitaabilidade do produto na maquina
        Nm  =  uIO.ReadVectorDoubleOpl(sc);//eficiencia por maquinas
        Dpt = uIO.ReadMatrixDoubleOpl(sc);//demanda do  produto no periodo
        
        //System.out.println(uIO.ReadSetOpl(sc));
        
        sc.close();
    }
    
    
}
