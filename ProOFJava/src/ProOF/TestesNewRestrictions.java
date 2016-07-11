/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF;

import ProOF.apl.UAV.abst.uncertainty.pStateUncertainty;
import ProOF.apl.UAV.map.Obstacle2D;
import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.geom.Point2D;
import java.util.LinkedList;
import java.util.Random;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.swing.JFrame;
import javax.swing.JPanel;

/**
 *
 * @author marcio
 */
public class TestesNewRestrictions {
    public static final double length = 2.0;
    
    public static final Obstacle2D obst[] = new Obstacle2D[]{
        new Obstacle2D("[1]", new Point2D.Double(0, 0), 1.0, 1.0, 0.0),
    };
    public static final int T = 3;
    public static final double M = 1e2;
    public static final double precision = 1e-6;
    private static int MODE = 4;
    
    public static boolean validCheng(double Xt[][], double Zjti[][][]){
        //------------------------[eq 6]-----------------------------
        for(int t=0; t<Xt.length; t++){
            for(int j=0; j<Zjti.length; j++){
                for(int i=0; i<obst[j].Gj(); i++){
                    double eq6 = M*(1-Zjti[j][t][i]);
                    eq6 -= obst[j].hyperplans[i].b;
                    eq6 += obst[j].hyperplans[i].scalarProd(Xt[t]);
                    if(eq6<-precision){
                        return false;
                    }
                }
            }
        }
        //------------------------[eq 7]-----------------------------
        for(int t=1; t<Xt.length; t++){
            for(int j=0; j<Zjti.length; j++){
                for(int i=0; i<obst[j].Gj(); i++){
                    double eq7 = M*(1-Zjti[j][t][i]);
                    eq7 -= obst[j].hyperplans[i].b;
                    eq7 += obst[j].hyperplans[i].scalarProd(Xt[t-1]);
                    if(eq7<-precision){
                        return false;
                    }
                }
            }
        }
        //------------------------[eq 8]-----------------------------
        for(int t=0; t<Xt.length; t++){
            for(int j=0; j<Zjti.length; j++){
                double eq8 = 1;
                for(int i=0; i<obst[j].Gj(); i++){
                    eq8 -= Zjti[j][t][i];
                }
                if(Math.abs(eq8)>precision){
                    return false;
                }
//                if(eq8>precision){
//                    return false;
//                }
            }
        }
        return true;
    }
    
    public static boolean validClaudio(double Xt[][], double Zjti[][][], boolean logic){
        //------------------------[eq 1]-----------------------------
        for(int t=0; t<Xt.length; t++){
            for(int j=0; j<Zjti.length; j++){
                for(int i=0; i<obst[j].Gj(); i++){
                    double eq1 = M*(1-Zjti[j][t][i]);
                    eq1 -= obst[j].hyperplans[i].b;
                    eq1 += obst[j].hyperplans[i].scalarProd(Xt[t]);
                    if(eq1<-precision){
                        return false;
                    }
                }
            }
        }
        double Pjti[][][] = new double[obst.length][Xt.length][];
        for(int t=1; t<Xt.length; t++){
            for(int j=0; j<Zjti.length; j++){
                Pjti[j][t] = new double[obst[j].Gj()];
            }
        }
        if(logic){
            for(int t=1; t<Xt.length; t++){
                for(int j=0; j<Zjti.length; j++){
                    for(int i=0; i<obst[j].Gj(); i++){
                        Pjti[j][t][i] = ((Zjti[j][t][i]>=1.0-precision) && (Zjti[j][t-1][i]>=1.0-precision)) ? 1.0 : 0.0;
                    }
                }
            }
        }else{   
            //------------------------[eq 3]-----------------------------
            for(int t=1; t<Xt.length; t++){
                for(int j=0; j<Zjti.length; j++){
                    for(int i=0; i<obst[j].Gj(); i++){
                        Pjti[j][t][i] = Math.max(Pjti[j][t][i], Zjti[j][t][i]+Zjti[j][t-1][i]-1);
                    }
                }
            }
            //------------------------[eq 4 and 5]-----------------------------
            for(int t=1; t<Xt.length; t++){
                for(int j=0; j<Zjti.length; j++){
                    for(int i=0; i<obst[j].Gj(); i++){
                        double max = Math.min(Zjti[j][t][i], Zjti[j][t-1][i]);
                        if(Pjti[j][t][i]>max+precision){
                            return false;
                        }
                        Pjti[j][t][i] = Math.max(Pjti[j][t][i], max);
                    }
                }
            }
        }
        //------------------------[eq 2]-----------------------------
        for(int t=1; t<Xt.length; t++){
            for(int j=0; j<Zjti.length; j++){
                double eq2 = 1;
                for(int i=0; i<obst[j].Gj(); i++){
                    eq2 -= Pjti[j][t][i];
                }
                if(eq2>precision){
                    return false;
                }
//                if(Math.abs(eq2)>precision){
//                    return false;
//                }
            }
        }
        return true;
    }
    
    public static boolean A(double Zjti[][][]){
        double Pjti[][][] = new double[obst.length][T][];
        for(int t=1; t<T; t++){
            for(int j=0; j<Zjti.length; j++){
                Pjti[j][t] = new double[obst[j].Gj()];
            }
        }
        for(int t=1; t<T; t++){
            for(int j=0; j<Zjti.length; j++){
                for(int i=0; i<obst[j].Gj(); i++){
                    Pjti[j][t][i] = ((Zjti[j][t][i]>=1.0-precision) && (Zjti[j][t-1][i]>=1.0-precision)) ? 1.0 : 0.0;
                }
            }
        }
        //------------------------[eq 2]-----------------------------
        for(int t=1; t<T; t++){
            for(int j=0; j<Zjti.length; j++){
                double eq2 = 1;
                for(int i=0; i<obst[j].Gj(); i++){
                    eq2 -= Pjti[j][t][i];
                }
//                if(eq2>precision){
//                    return false;
//                }
                if(Math.abs(eq2)>precision){
                    return false;
                }
            }
        }
        return true;
    }
    public static boolean B(double Zjti[][][]){
        double Pjti[][][] = new double[obst.length][T][];
        for(int t=1; t<T; t++){
            for(int j=0; j<Zjti.length; j++){
                Pjti[j][t] = new double[obst[j].Gj()];
            }
        }
         //------------------------[eq 3]-----------------------------
        for(int t=1; t<T; t++){
            for(int j=0; j<Zjti.length; j++){
                for(int i=0; i<obst[j].Gj(); i++){
                    Pjti[j][t][i] = Math.max(Pjti[j][t][i], Zjti[j][t][i]+Zjti[j][t-1][i]-1);
                }
            }
        }
        //------------------------[eq 4 and 5]-----------------------------
        for(int t=1; t<T; t++){
            for(int j=0; j<Zjti.length; j++){
                for(int i=0; i<obst[j].Gj(); i++){
                    double max = Math.min(Zjti[j][t][i], Zjti[j][t-1][i]);
                    if(Pjti[j][t][i]>max+precision){
                        return false;
                    }
                    Pjti[j][t][i] = Math.max(Pjti[j][t][i], max);
                }
            }
        }
        //------------------------[eq 2]-----------------------------
        for(int t=1; t<T; t++){
            for(int j=0; j<Zjti.length; j++){
                double eq2 = 1;
                for(int i=0; i<obst[j].Gj(); i++){
                    eq2 -= Pjti[j][t][i];
                }
//                if(eq2>precision){
//                    return false;
//                }
                if(Math.abs(eq2)>precision){
                    return false;
                }
            }
        }
        return true;
    }
    
    public static final Random rmd = new Random();
    public static void fillRandom(double Xt[][], double Zjti[][][]){
        for(int t=0; t<Xt.length; t++){
            for(int n=0; n<2; n++){
                Xt[t][n] = (rmd.nextDouble()*2-1)*length;
            }
        }
        fillZ(Zjti);
    }
    public static void fillZ(double Zjti[][][]){
        for(int t=0; t<T; t++){
            for(int j=0; j<Zjti.length; j++){
                double sum = 0;
                for(int i=0; i<obst[j].Gj(); i++){
                    //Zjti[j][t][i] = rmd.nextInt(2)==0 ? 1.0 : 0.0;
                    //Zjti[j][t][i] = rmd.nextInt(2);
                    Zjti[j][t][i] = rmd.nextDouble()*(1+precision)-precision;//   + rmd.nextDouble()*0.1;
                    Zjti[j][t][i] = Math.max(Zjti[j][t][i], 0.0);
                    Zjti[j][t][i] = Math.min(Zjti[j][t][i], 1.0);
                    sum += Zjti[j][t][i];
                }
                for(int i=0; i<obst[j].Gj(); i++){
                    Zjti[j][t][i] /= sum;
                }
            }
        }
    }
    public static void main2(String args[]) throws Exception{
        double Zjti[][][] = new double[obst.length][T][];
        for(int t=0; t<T; t++){
            for(int j=0; j<Zjti.length; j++){
                Zjti[j][t] = new double[obst[j].Gj()];
            }
        }
        
        int Aonly = 0;
        int Bonly = 0;
        int AandB = 0;
        int NotAandNotB = 0;
        for(int n=0; n<10000000; n++){
            fillZ(Zjti);
            boolean a = A(Zjti);
            boolean b = B(Zjti);
            if(a && !b){
                Aonly++;
            }else if(!a && b){
                Bonly++;
            }else if(a && b){
                AandB++;
            }else if(!a && !b){
                NotAandNotB++;
            }else{
                throw new Exception("this is not possible");
            }   
        }
        System.out.printf(" A & !B = %d\n", Aonly);
        System.out.printf("!A &  B = %d\n", Bonly);
        System.out.printf(" A &  B = %d\n", AandB);
        System.out.printf("!A & !B = %d\n", NotAandNotB);
    }
    public static void main(String args[]) throws Exception{
        double Xt[][] = new double[T][2];
        double Zjti[][][] = new double[obst.length][T][];
        for(int t=0; t<T; t++){
            for(int j=0; j<Zjti.length; j++){
                Zjti[j][t] = new double[obst[j].Gj()];
            }
        }
        final LinkedList<Integer[]> list = new LinkedList<Integer[]>();
        int ClaudioOnly = 0;
        int ChengOnly = 0;
        int ClaudioAndCheng = 0;
        int NotClaudioAndNotCheng = 0;
        for(int n=0; n<20000000; n++){
            fillRandom(Xt, Zjti);
            boolean claudio = validClaudio(Xt, Zjti, false);
            //boolean cheng = validClaudio(Xt, Zjti, true);
            boolean cheng = validCheng(Xt, Zjti);
            int Class = -1;
            if(claudio && !cheng){
                ClaudioOnly++;
                Class = 0;
            }else if(!claudio && cheng){
                ChengOnly++;
                Class = 1;
            }else if(claudio && cheng){
                ClaudioAndCheng++;
                Class = 2;
            }else if(!claudio && !cheng){
                NotClaudioAndNotCheng++;
                Class = 3;
            }else{
                throw new Exception("this is not possible");
            }
//            if(Class!=3){
//                for(int t=0; t<Xt.length-1; t++){
//                    list.add(new Integer[]{
//                        (int)(Xt[t][0]*300/length),
//                        (int)(Xt[t][1]*300/length),
//                        (int)(Xt[t+1][0]*300/length),
//                        (int)(Xt[t+1][1]*300/length), Class});
//                }
//            }
                
        }
        System.out.printf(" claudio = %d\n", ClaudioOnly+ClaudioAndCheng);
        System.out.printf(" cheng   = %d\n", ChengOnly+ClaudioAndCheng);
        System.out.printf(" claudio & !cheng = %d\n", ClaudioOnly);
        System.out.printf("!claudio &  cheng = %d\n", ChengOnly);
        System.out.printf(" claudio &  cheng = %d\n", ClaudioAndCheng);
        System.out.printf("!claudio & !cheng = %d\n", NotClaudioAndNotCheng);
        
        
        
//        JFrame frame = new JFrame();
//        frame.setSize(620, 670);
//        final JPanel draw = new JPanel(){
//            @Override
//            protected void paintComponent(Graphics g) {
//                super.paintComponent(g); //To change body of generated methods, choose Tools | Templates.
//                g.setColor(Color.WHITE);
//                g.fillRect(0, 0, 600, 600);
//                g.setColor(Color.BLACK);
//                g.drawRect(0, 0, 600, 600);
//                
//                
//                if(MODE==4){
//                    g.setColor(Color.LIGHT_GRAY);  //All
//                    g.fillRect(0, 600, 600, 40);
//                    g.setColor(Color.BLACK);
//                    g.setFont(g.getFont().deriveFont(Font.BOLD));
//                    g.drawString("All situations", 20, 617);
//                }
//                if(MODE==0){
//                    g.setColor(Color.GREEN);  //Bom situação correta detectada, não houve colisão com o obstáculo e a restrição confirmou isso
//                    g.fillRect(0, 600, 600, 40);
//                    g.setColor(Color.BLACK);
//                    g.setFont(g.getFont().deriveFont(Font.BOLD));
//                    g.drawString("Ok, avoidance collision correctly detected by model", 20, 617);
//                }
//                if(MODE==1){
//                    g.setColor(Color.YELLOW);   //Bom situação correta detectada, a restrição deu infactível e a rota também é infactível
//                    g.fillRect(0, 600, 600, 40);
//                    g.setColor(Color.BLACK);
//                    g.setFont(g.getFont().deriveFont(Font.BOLD));
//                    g.drawString("Ok, collision with obstacle correctly detected by model", 20, 617);
//                }
//                if(MODE==2){
//                    g.setColor(Color.MAGENTA);  //Warning, exesso de segurança, a restrição acusou infactibilidade mas a rota não é infactível.
//                    g.fillRect(0, 600, 600, 40);
//                    g.setColor(Color.BLACK);
//                    g.setFont(g.getFont().deriveFont(Font.BOLD));
//                    g.drawString("Warning, do not have collision with the obstacle but the model disagree", 20, 617);
//                }
//                if(MODE==3){
//                    g.setColor(Color.RED);  //Perigo colisão não dectectada, houve colisão com o obstáculo mas não foi calculado pela restrição
//                    g.fillRect(0, 600, 600, 40);
//                    g.setColor(Color.BLACK);
//                    g.setFont(g.getFont().deriveFont(Font.BOLD));
//                    g.drawString("Danger, collision occur but the model did not detect.", 20, 617);
//                }
//                g.setColor(Color.BLACK);
//                g.drawRect(0, 600, 600, 40);
//                
//                g.setColor(Color.LIGHT_GRAY);
//                for(int i=0; i<600; i++){
//                    for(int j=0; j<600; j++){
//                        double y = i - 300;
//                        double x = j - 300;
//                        if(pertence(0.0, x*length/300, y*length/300)){
//                            g.drawRect(j, i, 1, 1);
//                        }
//                    }
//                }
//                
//                
//                
//                
////                g.setColor(new Color(64, 64, 64, 128));
////                g.fillPolygon(poly);
////                g.setColor(Color.BLACK);
////                g.drawPolygon(poly);
//                
//                
//                
////                g.setColor(Color.BLUE);
////                for(int i=0; i<N; i++){
////                    int x0 = (int)(ax(i)*b(i)+300);
////                    int y0 = (int)(ay(i)*b(i)+300);
////                    int x1 = (int)(ax(i)*b(i)*1.2+300);
////                    int y1 = (int)(ay(i)*b(i)*1.2+300);
////                    
////                    g.drawLine(x0, y0, x1, y1);
////                    g.fillOval(x0-5, y0-5, 10, 10);
////                    g.fillRect(x1-5, y1-5, 10, 10);
////                }
//                
//                for(Integer[] p: list){
//                    if(p[4]==0){
//                        if(MODE==0 || MODE==4){
//                            g.setColor(Color.GREEN);  //Bom situação correta detectada, não houve colisão com o obstáculo e a restrição confirmou isso
//                            g.drawLine(p[0]+300, p[1]+300, p[2]+300, p[3]+300);
//                            g.fillOval(p[0]+300-5, p[1]+300-5, 10, 10);
//                            g.fillRect(p[2]+300-5, p[3]+300-5, 10, 10);
//                        }
//                    }else if(p[4]==1){
//                        if(MODE==1 || MODE==4){
//                            g.setColor(Color.YELLOW);   //Bom situação correta detectada, a restrição deu infactível e a rota também é infactível
//                            g.drawLine(p[0]+300, p[1]+300, p[2]+300, p[3]+300);
//                            g.fillOval(p[0]+300-5, p[1]+300-5, 10, 10);
//                            g.fillRect(p[2]+300-5, p[3]+300-5, 10, 10);
//                        }
//                    }else if(p[4]==2){
//                        if(MODE==2 || MODE==4){
//                            g.setColor(Color.MAGENTA);  //Warning, exesso de segurança, a restrição acusou infactibilidade mas a rota não é infactível.
//                            g.drawLine(p[0]+300, p[1]+300, p[2]+300, p[3]+300);
//                            g.fillOval(p[0]+300-5, p[1]+300-5, 10, 10);
//                            g.fillRect(p[2]+300-5, p[3]+300-5, 10, 10);
//                        }
//                    }else if(p[4]==3){
//                        if(MODE==3 || MODE==4){
//                            g.setColor(Color.RED);  //Perigo colisão não dectectada, houve colisão com o obstáculo mas não foi calculado pela restrição
//                            g.drawLine(p[0]+300, p[1]+300, p[2]+300, p[3]+300);
//                            g.fillOval(p[0]+300-5, p[1]+300-5, 10, 10);
//                            g.fillRect(p[2]+300-5, p[3]+300-5, 10, 10);
//                        }
//                    }
////                    if(p[4]>0.5){
////                        if(intercection(p, poly)){
////                            if(MODE==3 || MODE==4){
////                                g.setColor(Color.RED);  //Perigo colisão não dectectada, houve colisão com o obstáculo mas não foi calculado pela restrição
////                                g.drawLine(p[0]+300, p[1]+300, p[2]+300, p[3]+300);
////                                g.fillOval(p[0]+300-5, p[1]+300-5, 10, 10);
////                                g.fillRect(p[2]+300-5, p[3]+300-5, 10, 10);
////                            }
////                        }else{
////                            if(MODE==0 || MODE==4){
////                                g.setColor(Color.GREEN);  //Bom situação correta detectada, não houve colisão com o obstáculo e a restrição confirmou isso
////                                g.drawLine(p[0]+300, p[1]+300, p[2]+300, p[3]+300);
////                                g.fillOval(p[0]+300-5, p[1]+300-5, 10, 10);
////                                g.fillRect(p[2]+300-5, p[3]+300-5, 10, 10);
////                            }
////                        }
////                    }else{
////                        if(intercection(p, poly)){
////                            if(MODE==1 || MODE==4){
////                                g.setColor(Color.YELLOW);   //Bom situação correta detectada, a restrição deu infactível e a rota também é infactível
////                                g.drawLine(p[0]+300, p[1]+300, p[2]+300, p[3]+300);
////                                g.fillOval(p[0]+300-5, p[1]+300-5, 10, 10);
////                                g.fillRect(p[2]+300-5, p[3]+300-5, 10, 10);
////                            }
////                        }else{
////                            if(MODE==2 || MODE==4){
////                                g.setColor(Color.MAGENTA);  //Warning, exesso de segurança, a restrição acusou infactibilidade mas a rota não é infactível.
////                                g.drawLine(p[0]+300, p[1]+300, p[2]+300, p[3]+300);
////                                g.fillOval(p[0]+300-5, p[1]+300-5, 10, 10);
////                                g.fillRect(p[2]+300-5, p[3]+300-5, 10, 10);
////                            }
////                        }
////                    }
//                }
//                
////                for(Integer[] p: dest){
////                    final int r = 5;
////                    
////                    if(p[4]==0){
////                        if(MODE==0 || MODE==4){
////                            g.setColor(Color.GREEN);  //Bom situação correta detectada, não houve colisão com o obstáculo e a restrição confirmou isso
////                            g.fillOval(p[0]+300-r, p[1]+300-r, 2*r, 2*r);
////                            g.fillRect(p[2]+300-r, p[3]+300-r, 2*r, 2*r);
////
////                            g.setColor(Color.BLACK);
////                            g.drawOval(p[0]+300-r, p[1]+300-r, 2*r, 2*r);
////                            g.drawRect(p[2]+300-r, p[3]+300-r, 2*r, 2*r);
////                        }
////                    }else if(p[4]==1){
////                        if(MODE==1 || MODE==4){
////                            g.setColor(Color.YELLOW);   //Bom situação correta detectada, a restrição deu infactível e a rota também é infactível
////                            g.fillOval(p[0]+300-r, p[1]+300-r, 2*r, 2*r);
////                            g.fillRect(p[2]+300-r, p[3]+300-r, 2*r, 2*r);
////
////                            g.setColor(Color.BLACK);
////                            g.drawOval(p[0]+300-r, p[1]+300-r, 2*r, 2*r);
////                            g.drawRect(p[2]+300-r, p[3]+300-r, 2*r, 2*r);
////                        }
////                    }else if(p[4]==2){
////                        if(MODE==2 || MODE==4){
////                            g.setColor(Color.MAGENTA);  //Warning, exesso de segurança, a restrição acusou infactibilidade mas a rota não é infactível.
////                            g.fillOval(p[0]+300-r, p[1]+300-r, 2*r, 2*r);
////                            g.fillRect(p[2]+300-r, p[3]+300-r, 2*r, 2*r);
////
////                            g.setColor(Color.BLACK);
////                            g.drawOval(p[0]+300-r, p[1]+300-r, 2*r, 2*r);
////                            g.drawRect(p[2]+300-r, p[3]+300-r, 2*r, 2*r);
////                        }
////                    }else if(p[4]==3){
////                        if(MODE==3 || MODE==4){
////                            g.setColor(Color.RED);  //Perigo colisão não dectectada, houve colisão com o obstáculo mas não foi calculado pela restrição
////                            g.fillOval(p[0]+300-r, p[1]+300-r, 2*r, 2*r);
////                            g.fillRect(p[2]+300-r, p[3]+300-r, 2*r, 2*r);
////
////                            g.setColor(Color.BLACK);
////                            g.drawOval(p[0]+300-r, p[1]+300-r, 2*r, 2*r);
////                            g.drawRect(p[2]+300-r, p[3]+300-r, 2*r, 2*r);
////                        }
////                    }
////                    
////                   
////                    
////                }
//            }
//        };
//        draw.addMouseListener(new MouseAdapter() {
//            @Override
//            public void mouseClicked(MouseEvent e) {
//                super.mouseClicked(e); //To change body of generated methods, choose Tools | Templates.
//                MODE = (MODE+1)%5;
//                draw.repaint();
//            }
//        });
//        draw.setBackground(Color.BLACK);
//        frame.add(draw);
//        frame.setVisible(true);
//        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
    }
    private static boolean pertence(double R, double... x){
        for(int j=0; j<obst.length; j++){
            try {
                if(obst[j].distance(x)<=R){
                    return true;
                }
            } catch (Exception ex) {
                Logger.getLogger(TestesNewRestrictions.class.getName()).log(Level.SEVERE, null, ex);
            }
        }
        return false;
    }
}
