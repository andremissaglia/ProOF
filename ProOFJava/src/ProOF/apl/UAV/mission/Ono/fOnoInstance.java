/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.mission.Ono;

import ProOF.apl.UAV.gen.linear.vehicle.parts.oLinearDynamic;
import ProOF.apl.UAV.gen.linear.vehicle.parts.pLinearWaypoints;
import ProOF.apl.UAV.map.Obstacle;
import ProOF.apl.UAV.map.Region2D;
import ProOF.com.Linker.LinkerApproaches;
import ProOF.com.Linker.LinkerParameters;
import ProOF.com.Linker.LinkerResults;
import ProOF.com.language.Factory;
import ProOF.utilities.uIO;
import java.io.File;
import java.io.FileNotFoundException;
import java.util.LinkedList;
import java.util.Scanner;

/**
 *
 * @author marcio
 */
public class fOnoInstance extends Factory<OnoInstance>{
    public static final fOnoInstance obj = new fOnoInstance();
    
    @Override
    public String name() {
        return "Instance Reader";
    }
    @Override
    public OnoInstance build(int index) throws Exception {
        switch(index){
            case 0 : return new Instance2D();
            case 1 : return new MissionMap2D();
        }
        return null;
    }

    private static class Instance2D extends OnoInstance {
        private oLinearDynamic dynamic;
        
        private File config;

        private String map_name;
        //private final Font font = new Font(Font.SANS_SERIF, Font.BOLD, 24);

        @Override
        public String name() {
            return "Instance 2D";
        }
        @Override
        public void parameters(LinkerParameters link) throws Exception {
            config = link.File("CCQSP 2D", null, "sgl");
        }
        @Override
        public void services(LinkerApproaches link) throws Exception {
            super.services(link); //To change body of generated methods, choose Tools | Templates.
            dynamic = link.need(oLinearDynamic.class, dynamic);
        }
        
        @Override
        public int N() {
            return 2;
        }
        @Override
        public void load() throws FileNotFoundException, Exception {
            Scanner sc = new Scanner(config);
            start_state = readDoubles(sc, new double[dynamic.n()]);
            chance_constraints = readDoubles(sc, null);
            n_events = uIO.ReadInt(sc);
            int R = uIO.ReadInt(sc);
            temporal_constraints = new TemporalConstraints[R];
            for(int r=0; r<R; r++){
                temporal_constraints[r] = new TemporalConstraints(sc);
            }
            int n_episodes = uIO.ReadInt(sc);
            episodes = new Episode[n_episodes];
            for(int n=0; n<n_episodes; n++){
                episodes[n] = new Episode(n, sc);
                System.out.println(episodes[n]);
            }
            int n_regions = uIO.ReadInt(sc);
            regions = new Obstacle[n_regions];
            for(int n=0; n<n_regions; n++){
                regions[n] = new Region2D(sc, true);
            }
            sc.close();
        }
        @Override
        public String map_mame() {
            return map_name;
        }        
        @Override
        public void results(LinkerResults link) throws Exception {
            super.results(link); //To change body of generated methods, choose Tools | Templates.
            link.writeString("map_name", map_name);
        }
    }
    
    private static class MissionMap2D extends OnoInstance {
        private oLinearDynamic dynamic;
        private pLinearWaypoints waypoint;
        private File map;
        private File mission;

        private String map_name;
        //private final Font font = new Font(Font.SANS_SERIF, Font.BOLD, 24);

        @Override
        public String name() {
            return "Mission Map 2D";
        }
        @Override
        public void parameters(LinkerParameters link) throws Exception {
            map = link.File("CCQSP 2D map", null, "map");
            mission = link.File("CCQSP 2D mission", null, "mission");
        }
        @Override
        public void services(LinkerApproaches link) throws Exception {
            super.services(link); //To change body of generated methods, choose Tools | Templates.
            dynamic = link.need(oLinearDynamic.class, dynamic);
            waypoint = link.need(pLinearWaypoints.class, waypoint);
        }
        
        @Override
        public int N() {
            return 2;
        }
        @Override
        public void load() throws FileNotFoundException, Exception {
            Scanner sc = new Scanner(map);
            int way[] = uIO.ReadVectorInt(sc);
            int obs[] = uIO.ReadVectorInt(sc);
            
            regions = new Obstacle[way.length + obs.length];
            System.out.println(regions.length);
            for(int n=0; n<regions.length; n++){
                regions[n] = new Region2D(sc, true);
            }
            sc.close();
            
            
            sc = new Scanner(mission);
            start_state = readDoubles(sc, new double[dynamic.n()]);
            chance_constraints = readDoubles(sc, null);
            n_events = way.length+1;
            
            
            LinkedList<OnoInstance.TemporalConstraints> listEvent = new LinkedList<OnoInstance.TemporalConstraints>(); 
            int R = uIO.ReadInt(sc);
            for(int r=0; r<R; r++){
                listEvent.addAll(temporalConstraints(sc, n_events, waypoint.timeHorizon()));
            }
            temporal_constraints = listEvent.toArray(new OnoInstance.TemporalConstraints[listEvent.size()]);
            
            System.out.println("----------------[TemporalConstraints]----------------");
            for(OnoInstance.TemporalConstraints t : listEvent){
                System.out.println(t);
            }
            
            
            
            LinkedList<OnoInstance.Episode> listEpisode = new LinkedList<OnoInstance.Episode>();
            int n_episodes = uIO.ReadInt(sc);
            
            for(int n=0; n<n_episodes; n++){
                listEpisode.addAll(episodes(sc, n_events, way, obs));
            }
            episodes = listEpisode.toArray(new OnoInstance.Episode[listEpisode.size()]);
            
            System.out.println("----------------[Episodes]----------------");
            System.out.printf("%8s %8s %2s %2s %2s %s\n", "name", "type", "c", "eS", "eE", "ra");
            for(OnoInstance.Episode ep : listEpisode){
                System.out.println(ep);
            }
            
            sc.close();
        }
        @Override
        public String map_mame() {
            return map_name;
        }        
        @Override
        public void results(LinkerResults link) throws Exception {
            super.results(link); //To change body of generated methods, choose Tools | Templates.
            link.writeString("map_name", map_name);
        }
    }
    
    public static double[] readDoubles(Scanner sc, double vet[]){
        sc.nextLine();
        //System.out.println("["+sc.nextLine()+"]");
        if(vet==null){
            vet = new double[Integer.parseInt(sc.nextLine())];
        }
        for(int i=0; i<vet.length; i++){
            vet[i] = sc.nextDouble();
            //System.out.printf("%g ", vet[i]);
        }
        //System.out.println("["+sc.nextLine()+"]");
        sc.nextLine();
        return vet;
    }
}
