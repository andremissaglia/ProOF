/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.karla.model.RealeImaginario;

import ProOF.com.Linker.LinkerParameters;
import ProOF.opt.abst.problem.Instance;
import ProOF.utilities.uIO;
import java.io.File;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Scanner;

/**
 *
 * @author Administrador
 */
public class RIInstance extends Instance{
    private File file;
    public int Nnos;
    public int w [][];
    public double Rr[][];
    public double Ri[][];
    public double v0fr[];
    public double v0fi[];
    public ArrayList<Integer> F;
    public double Dr[];
    public double Di[];
    public double vmin;
    public double M[][];
    /** matriz de ligações ativas na conf inicial */
    public int x0[][];
    
    
   

    @Override
    public String name() {
        return "RealeImaginario";
    }

    @Override
    public void parameters(LinkerParameters link) throws Exception {
        file = link.File("File for RI", null, "dat", "pos");
    }

    public static void main (String args[]) throws Exception{
        RIInstance inst = new RIInstance ();
        inst.file = new File("C:\\ProOF2.2\\Instances\\ReRI\\bus_13_3.pos");
        inst.load();
    }
    
    
    public void load() throws Exception{
        Scanner sc = new Scanner (file);
        String ext = uIO.ReadTrueName(sc);
        sc.close();
        if (ext.contains(".dat")){
            loadDat();
        }
        else{
            loadPos();
        }
    }
    
    
    public void loadPos() throws Exception {
        //int a [] = new int[1000000];
        Scanner sc = new Scanner (file);
        
        
        //ler dados em .pos
        vmin = LerDouble (sc);
        int PU = LerInt(sc);
        double V_base = LerDouble (sc);
        double S_base = LerDouble (sc);
        int Branch_count = LerInt (sc);
        int Sectionalizing_count = LerInt (sc);
        int Tie_count = LerInt (sc);
        int Nodes = LerInt (sc);
        int Feeders = LerInt(sc);
        int Feeder_node_ids[] = LerVetorInt(sc);
        sc.nextLine();
        
        Nnos = Nodes;
        w = new int[Nnos][Nnos];
        Rr = new double[Nnos] [Nnos];
        Ri = new double[Nnos] [Nnos];
        v0fr = new double [Nnos];
        for(int id: Feeder_node_ids){
            v0fr[id] = V_base;
        }
        v0fi = new double [Nnos];
        F = new ArrayList<Integer>();
        for(int id: Feeder_node_ids){
            F.add(id+1);
        }
        Dr = new double [Nnos];
        Di = new double [Nnos];
        x0 = new int [Nnos][Nnos];
        M = new double [Nnos][Nnos];
        
        LinkedList <Integer> NoPara = new LinkedList<Integer>();
        
        
        for(int i=0; i<Branch_count +1; i++){
            String line = NextLine (sc);
            if(!line.contains(".Tie switches")){
                System.out.println(line);
                String args [] = line.split(" ");
                int NoInicial = Integer.parseInt(args[1]);
                int NoFinal = Integer.parseInt(args[2]);
                double ResistenciaR = Double.parseDouble(args[3]);
                double ReatanciaX = Double.parseDouble(args[4]);
                if(PU==1){
                    ResistenciaR *= S_base*S_base;
                    ReatanciaX *= S_base*S_base;
                }
                if(args.length>6){
                    double PotenciaAtivaP = Double.parseDouble(args[5]);
                    double PotenciaReativaQ = Double.parseDouble(args[6]);
                    Dr [NoFinal] = PotenciaAtivaP/V_base;
                    Di [NoFinal] = - PotenciaReativaQ/V_base;
                    x0 [NoInicial][NoFinal] = 1;
                    NoPara.add(NoFinal);
                }
                
                w[NoInicial][NoFinal] = 1;
                w[NoFinal][NoInicial] = 1;
                Rr[NoInicial][NoFinal] = ResistenciaR;
                Rr[NoFinal][NoInicial] = ResistenciaR;
                Ri[NoInicial][NoFinal] = ReatanciaX;
                Ri[NoFinal][NoInicial] = ReatanciaX;
                
            }
            
        }
        
        double CorrenteMaxima = 0;
        
        for(int i: NoPara){
            double CorrenteAjusante = CorrenteAjusante (i);
            CorrenteMaxima = Math.max(CorrenteMaxima, CorrenteAjusante+Dr[i]);
            System.out.printf("CorrenteAjusante (%d) = %f \n", i, CorrenteAjusante);
        }
      
        
        System.out.printf("CorrenteMaxima = %f \n", CorrenteMaxima);
        for(int i=0; i<Nnos;i++){
            for(int j=0; j<Nnos; j++){
                if(Rr[i][j]>0.0000001){
                    M [i] [j] = CorrenteMaxima;
                }
            }
        }
        sc.close();
        
    }
    
    
    
    
    
    
     private double CorrenteAjusante(int i) {
        double CorrenteAjusante = 0;
        for(int j = 0; j<Nnos; j++){
            if(x0[i][j]==1){
              CorrenteAjusante += Dr[j]+CorrenteAjusante (j);  
            }
        }
        return CorrenteAjusante;
    }
    
    
 
    
    private double LerDouble(Scanner sc) {
         String line = NextLine (sc);
         System.out.println (line);
         String args [] = line.split(" ");
         return Double.parseDouble(args[1]);
    }
    
    private int LerInt(Scanner sc) {
         String line = NextLine (sc);
         System.out.println (line);
         String args [] = line.split(" ");
         return Integer.parseInt(args[1]);
    }
    
    
    private int [] LerVetorInt(Scanner sc) {
         String line = NextLine (sc);
         System.out.println (line);
         String args [] = line.split(" ");
         int [] vetor = new int [args.length -1];
         for(int i=0; i<vetor.length; i++){
             vetor [i] = Integer.parseInt(args[i+1]);
         }
         return vetor;
    }
    
    private String NextLine (Scanner sc){
        String line = sc.nextLine();
            line = line.replaceAll("\t", " ");
            while (line.contains("  ")){
                line = line.replaceAll("  ", " ");
            }
        return line;
    }
    
    public void loadDat() throws Exception {
        Scanner sc = new Scanner(file);
        
        //ler dados em opl no .dat
        Nnos = uIO.ReadIntOpl(sc);
        w = uIO.ReadMatrixIntOpl(sc);
        Rr = uIO.ReadMatrixDoubleOpl(sc);
        Ri = uIO.ReadMatrixDoubleOpl(sc);
        v0fr = uIO.ReadVectorDoubleOpl(sc);
        v0fi = uIO.ReadVectorDoubleOpl(sc);
        F = uIO.ReadSetOpl(sc);
        Dr = uIO.ReadVectorDoubleOpl(sc);
        Di = uIO.ReadVectorDoubleOpl(sc);
        vmin = uIO.ReadDoubleOpl(sc);
        M = uIO.ReadMatrixDoubleOpl(sc);
        x0 = uIO.ReadMatrixIntOpl(sc);
        
        //Nnos = 3;
        
        
        sc.close();
       
    }

   

    

    
    
}
