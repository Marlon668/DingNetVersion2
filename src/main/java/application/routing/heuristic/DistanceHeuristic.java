package application.routing.heuristic;

import org.jxmapviewer.viewer.GeoPosition;
import util.Connection;
import util.GraphStructure;
import util.MapHelper;

import java.util.Map;


/**
 * A routing heuristic based on the shortest path.
 */
public class DistanceHeuristic implements RoutingHeuristic {
    @Override
    public double calculateAccumulatedCost(HeuristicEntry entry){
        GeoPosition begin = entry.graph.getWayPoint(entry.connection.getFrom());
        GeoPosition end = entry.graph.getWayPoint(entry.connection.getTo());

        return MapHelper.distance(begin, end);
    }

    @Override
    public double calculateCostConnection(Connection connection, GraphStructure graph)
    {
        GeoPosition begin = graph.getWayPoint(connection.getFrom());
        GeoPosition end = graph.getWayPoint(connection.getTo());

        return MapHelper.distance(begin,end);
    }

    @Override
    public double calculateHeuristic(HeuristicEntry entry) {
        GeoPosition begin = entry.graph.getWayPoint(entry.connection.getFrom());
        GeoPosition end = entry.graph.getWayPoint(entry.connection.getTo());

        return MapHelper.distance(begin, end);
    }

    @Override
    public double calculateCostBetweenTwoNeighbours(GeoPosition begin, GeoPosition end)
    {
        return MapHelper.distance(begin, end);
    }
}
