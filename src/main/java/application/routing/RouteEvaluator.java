package application.routing;

import application.pollution.PollutionGrid;
import application.routing.heuristic.RoutingHeuristic;
import application.routing.heuristic.SimplePollutionHeuristic;
import iot.Environment;
import iot.networkentity.Mote;
import org.jxmapviewer.viewer.Waypoint;
import util.Connection;
import util.Pair;

import java.lang.ref.WeakReference;
import java.util.HashMap;
import java.util.Map;

public class RouteEvaluator extends Evaluator {

    private Map<Long,Double> totalAccumulatedCost;
    private Map<Long,Long> firstVisitedWaypointByMote;
    private Map<Long, Connection> lastUsedConnectionByMote;
    private RoutingHeuristic heuristic;
    private WeakReference<Environment> environment;
    private Map<Long, Pair<Integer,Connection>> amountConnectionUsed;
    private Map<Long,Double> lastMeasurementOfMote;

    public RouteEvaluator(PollutionGrid pollutionGrid){
        super();
        this.firstVisitedWaypointByMote = new HashMap<Long,Long>();
        this.lastUsedConnectionByMote = new HashMap<Long,Connection>();
        this.totalAccumulatedCost = new HashMap<Long,Double>();
        this.heuristic = new SimplePollutionHeuristic(pollutionGrid);
        this.amountConnectionUsed = new HashMap<>();
        this.lastMeasurementOfMote = new HashMap<>();
    }

    public void setEnvironment(WeakReference<Environment> environment){
        this.environment = environment;
    }

    /**
     *
     * @param mote
     * @param positionMoteInWaypoint
     */
    public void addCostConnectionOfMote(Mote mote, int positionMoteInWaypoint)
    {
        if(firstVisitedWaypointByMote.get(mote.getEUI())==null)
        {
            //System.out.println("ttt");
            firstVisitedWaypointByMote.put(mote.getEUI(),environment.get().getGraph().getClosestWayPoint(mote.getPath().getWayPoints().get(0)));
            this.totalAccumulatedCost.put(mote.getEUI(),0.0);
            //System.out.println(mote.getEUI());
            //System.out.println(firstVisitedWaypointByMote.get(mote.getEUI()));
        }
        if (lastUsedConnectionByMote.get(mote.getEUI()) == null) {
            Connection usedConnection = new Connection(firstVisitedWaypointByMote.get(mote.getEUI()),environment.get().getGraph().getClosestWayPoint(mote.getPath().getWayPoints().get(positionMoteInWaypoint)));
            amountConnectionUsed.put(mote.getEUI(),new Pair<>(1,usedConnection));
            lastUsedConnectionByMote.put(mote.getEUI(),usedConnection);
            this.lastMeasurementOfMote.put(mote.getEUI(),this.heuristic.calculateCostConnection(usedConnection,environment.get().getGraph()));
            //System.out.println("Connection");
            //System.out.println(lastUsedConnectionByMote.get(mote.getEUI()).getFrom());
            //System.out.println(lastUsedConnectionByMote.get(mote.getEUI()).getTo());
        }
        if(!(lastUsedConnectionByMote.get(mote.getEUI()).getTo() == environment.get().getGraph().getClosestWayPoint(mote.getPath().getWayPoints().get(positionMoteInWaypoint))))
        {
            //System.out.println("From  " + amountConnectionUsed.get(mote.getEUI()).getRight().getFrom());
            //System.out.println("To    " + amountConnectionUsed.get(mote.getEUI()).getRight().getTo());
            double accumulatedCost = this.totalAccumulatedCost.get(mote.getEUI()) + lastMeasurementOfMote.get(mote.getEUI())/amountConnectionUsed.get(mote.getEUI()).getLeft();
            this.totalAccumulatedCost.put(mote.getEUI(),accumulatedCost);
            Connection usedConnection = new Connection(lastUsedConnectionByMote.get(mote.getEUI()).getTo(),environment.get().getGraph().getClosestWayPoint(mote.getPath().getWayPoints().get(positionMoteInWaypoint)));
            //getEnvironment().getGraph().deleteWayPoint(lastUsedConnectionByMote.get(mote.getEUI()).getFrom(),getEnvironment());
            amountConnectionUsed.put(mote.getEUI(),new Pair<>(1,usedConnection));
            lastUsedConnectionByMote.put(mote.getEUI(),usedConnection);
            this.lastMeasurementOfMote.put(mote.getEUI(),this.heuristic.calculateCostConnection(usedConnection,environment.get().getGraph()));
        }
        else{
            int amountUsed = amountConnectionUsed.get(mote.getEUI()).getLeft() + 1;
            amountConnectionUsed.put(mote.getEUI(),new Pair<>(amountUsed,amountConnectionUsed.get(mote.getEUI()).getRight()));
            lastMeasurementOfMote.put(mote.getEUI(),lastMeasurementOfMote.get(mote.getEUI())+this.heuristic.calculateCostConnection(lastUsedConnectionByMote.get(mote.getEUI()),environment.get().getGraph()));
        }
    }


    public Double getTotalCostPath(long moteId)
    {
        // Don't forget the last connection
        //System.out.println("From  " + amountConnectionUsed.get(moteId).getRight().getFrom());
        //System.out.println("To    " + amountConnectionUsed.get(moteId).getRight().getTo());
        double accumulatedCost = this.totalAccumulatedCost.get(moteId) + lastMeasurementOfMote.get(moteId)/amountConnectionUsed.get(moteId).getLeft();
        this.totalAccumulatedCost.put(moteId,accumulatedCost);
        return totalAccumulatedCost.get(moteId) ;
    }

    public Double getCostConnection(Connection connection)
    {
        return heuristic.calculateCostConnection(connection,environment.get().getGraph());
    }

    public void reset(){
        this.firstVisitedWaypointByMote = new HashMap<Long,Long>();
        this.lastUsedConnectionByMote = new HashMap<Long,Connection>();
        this.totalAccumulatedCost = new HashMap<Long,Double>();
    }
}
