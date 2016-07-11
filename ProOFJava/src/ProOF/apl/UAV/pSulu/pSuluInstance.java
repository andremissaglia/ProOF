/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package ProOF.apl.UAV.pSulu;

import ProOF.apl.UAV.gen.linear.vehicle.parts.oLinearDynamic;
import ProOF.apl.UAV.map.Obstacle;
import ProOF.apl.UAV.map.Region2D;
import ProOF.apl.UAV.mission.Ono.OnoInstance;
import static ProOF.apl.UAV.mission.Ono.fOnoInstance.readDoubles;
import ProOF.com.Linker.LinkerApproaches;
import ProOF.com.Linker.LinkerParameters;
import ProOF.com.Linker.LinkerResults;
import ProOF.utilities.uIO;
import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

/**
 *
 * @author marcio
 */
public class pSuluInstance extends OnoInstance{
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
    public int N() {
        return 2;
    }
    @Override
    public void load() throws FileNotFoundException, Exception {
        Scanner sc = new Scanner(config);
        start_state = readDoubles(sc, new double[2*N()]);
        chance_constraints = readDoubles(sc, null);
        n_events = uIO.ReadInt(sc);
        int R = uIO.ReadInt(sc);
        temporal_constraints = new OnoInstance.TemporalConstraints[R];
        for(int r=0; r<R; r++){
            temporal_constraints[r] = new OnoInstance.TemporalConstraints(sc);
        }
        int n_episodes = uIO.ReadInt(sc);
        episodes = new OnoInstance.Episode[n_episodes];
        for(int n=0; n<n_episodes; n++){
            episodes[n] = new OnoInstance.Episode(n, sc);
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
