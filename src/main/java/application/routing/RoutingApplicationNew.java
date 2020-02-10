package application.routing;

import application.Application;
import application.routing.heuristic.SimplePollutionHeuristic;
import be.kuleuven.cs.som.annotate.Model;
import iot.Environment;
import iot.SimulationRunner;
import iot.lora.LoraTransmission;
import iot.lora.LoraWanPacket;
import iot.lora.MessageType;
import iot.mqtt.BasicMqttMessage;
import iot.mqtt.Topics;
import iot.mqtt.TransmissionWrapper;
import iot.networkentity.Gateway;
import iot.networkentity.Mote;
import org.jetbrains.annotations.NotNull;
import org.jxmapviewer.viewer.GeoPosition;
import selfadaptation.feedbackloop.GenericFeedbackLoop;
import selfadaptation.instrumentation.FeedbackLoopGatewayBuffer;
import selfadaptation.instrumentation.MoteProbe;
import util.Converter;
import util.GraphStructure;
import util.MapHelper;
import util.Pair;

import java.nio.ByteBuffer;
import java.util.*;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

public class RoutingApplicationNew extends Application {
    // The routes stored per device
    private Map<Long, Pair<Double,List<GeoPosition>>> routes;

    // The last recorded positions of the requesting user motes
    private Map<Long, GeoPosition> lastPositions;

    // The graph with waypoints and connections
    private GraphStructure graph;

    // The route finding algorithm that is used to handle routing requests
    private PathFinder pathFinder;

    private MoteProbe moteProbe;

    private RouteAnalyser pathAnalyser;

    private final double BUFFERSIZE = 7;

    /**
     * A HashMap representing the buffers for the approach.
     */
    @Model
    private Map<Long,List<List<Pair<Double,List<GeoPosition>>>>> bufferKBestPaths;

    /**
     * Returns the algorithm buffers.
     * @return The algorithm buffers.
     */
    @Model
    private Map<Long,List<List<Pair<Double,List<GeoPosition>>>>> getBufferKBestPaths(){
        return this.bufferKBestPaths;
    }

    /**
     * Puts an KBestPathBuffer under the KBestPathBuffers under a path.
     * @param kPaths The k best paths according with its accumulated cost
     */
    @Model
    private void putKBestPathsBuffers(long moteID, List<Pair<Double,List<GeoPosition>>> kPaths) {
        if(this.bufferKBestPaths.get(moteID) == null)
        {
            this.bufferKBestPaths.put(moteID, new ArrayList<List<Pair<Double, List<GeoPosition>>>>());
        }
        this.bufferKBestPaths.get(moteID).add(kPaths);
    }



    public RoutingApplicationNew(PathFinder pathFinder, Environment environment) {
        super(List.of(Topics.getNetServerToApp("+", "+")));
        this.routes = new HashMap<>();
        this.lastPositions = new HashMap<>();
        this.graph = environment.getGraph();
        this.pathFinder = pathFinder;
        bufferKBestPaths = new HashMap<>();
        this.moteProbe = new MoteProbe();
        this.pathAnalyser = new RouteAnalyser(this.pathFinder.getHeuristic());


    }


    /**
     * Handle a route request message by replying with (part of) the route to the requesting device.
     * @param message The message which contains the route request.
     */
    private void handleRouteRequest(LoraWanPacket message) {
        //
        //out.println("h");
        var body = Arrays.stream(Converter.toObjectType(message.getPayload()))
            .skip(1) // Skip the first byte since this indicates the message type
            .collect(Collectors.toList());
        long deviceEUI = message.getSenderEUI();

        GeoPosition motePosition;
        GeoPosition destinationPosition;

        if (!lastPositions.containsKey(deviceEUI)) {
            // This is the first request the mote has made for a route
            //  -> both the current position as well as the destination of the mote are transmitted
            byte[] rawPositions = new byte[16];
            IntStream.range(0, 16).forEach(i -> rawPositions[i] = body.get(i));

            ByteBuffer byteBuffer = ByteBuffer.wrap(rawPositions);
            motePosition = new GeoPosition(byteBuffer.getFloat(0), byteBuffer.getFloat(4));
            destinationPosition = new GeoPosition(byteBuffer.getFloat(8), byteBuffer.getFloat(12));
        } else {
            // The mote has already sent the initial request
            //  -> only the current position of the mote is transmitted (the destination has been stored already)
            byte[] rawPositions = new byte[8];
            IntStream.range(0, 8).forEach(i -> rawPositions[i] = body.get(i));
            ByteBuffer byteBuffer = ByteBuffer.wrap(rawPositions);

            motePosition = new GeoPosition(byteBuffer.getFloat(0), byteBuffer.getFloat(4));
            destinationPosition = routes.get(deviceEUI).getRight().get(routes.get(deviceEUI).getRight().size()-1);
        }

        // Use the routing algorithm to calculate the K best paths for the mote
        //List<GeoPosition> routeMote=this.pathFinder.retrievePath(graph,motePosition,destinationPosition);
        List<Pair<Double,List<GeoPosition>>> routeMote = this.pathFinder.retrieveKPaths(graph, motePosition, destinationPosition,3);
        putKBestPathsBuffers(deviceEUI,routeMote);
        Pair<Double,List<GeoPosition>> bestPath;
        if(bufferKBestPaths.get(deviceEUI).size()==BUFFERSIZE) {
            bestPath = takeBestPath(bufferKBestPaths.get(deviceEUI));
            bufferKBestPaths = new HashMap<>();
            if(bestPath.getRight().size()>0) {
                this.routes.put(deviceEUI, bestPath);
            }
            else{
                bestPath = this.routes.get(deviceEUI);
                //if(bestPath.getRight().size()>1)

                    bestPath.getRight().remove(0);

                //bestPath.getRight().remove(0);
                this.routes.put(deviceEUI, bestPath);
            }
        }
        else {
            if (routes.get(deviceEUI) == null) {
                bestPath = this.pathFinder.retrievePath(graph,motePosition, destinationPosition);
                this.routes.put(deviceEUI, bestPath);
            } else {
                bestPath = this.routes.get(deviceEUI);
                //if(bestPath.getRight().size()>1)

                    bestPath.getRight().remove(0);

                this.routes.put(deviceEUI, bestPath);
            }
        }
        //List<GeoPosition> bestPath =
        // Compose the reply packet: up to 24 bytes for now, which can store 1 geoposition (in float)
        int amtPositions = Math.min(bestPath.getRight().size() - 1, 1);
        ByteBuffer payloadRaw = ByteBuffer.allocate(8 * amtPositions);

        for (GeoPosition pos : bestPath.getRight().subList(1, amtPositions+1)) {
            payloadRaw.putFloat((float) pos.getLatitude());
            payloadRaw.putFloat((float) pos.getLongitude());
        }

        List<Byte> payload = new ArrayList<>();
        for (byte b : payloadRaw.array()) {
            payload.add(b);
        }

        // Update the position of the mote if it has changed since the previous time
        if (!lastPositions.containsKey(deviceEUI) || !lastPositions.get(deviceEUI).equals(motePosition)) {
            lastPositions.put(deviceEUI, motePosition);
        }

        // Send the reply (via MQTT) to the requesting device
        //.println("gggggg   " + deviceEUI);
        BasicMqttMessage routeMessage = new BasicMqttMessage(payload);
        this.mqttClient.publish(Topics.getAppToNetServer(message.getReceiverEUI(), deviceEUI), routeMessage);
    }

    /**
     * Handle a route request message by replying with (part of) the route to the requesting device.
     * @param message The message which contains the route request.
     */
    private void handleRouteRequest2(LoraWanPacket message) {
        //System.out.println("h");
        var body = Arrays.stream(Converter.toObjectType(message.getPayload()))
            .skip(1) // Skip the first byte since this indicates the message type
            .collect(Collectors.toList());
        long deviceEUI = message.getSenderEUI();

        GeoPosition motePosition;
        GeoPosition destinationPosition;

        if (!lastPositions.containsKey(deviceEUI)) {
            // This is the first request the mote has made for a route
            //  -> both the current position as well as the destination of the mote are transmitted
            byte[] rawPositions = new byte[16];
            IntStream.range(0, 16).forEach(i -> rawPositions[i] = body.get(i));

            ByteBuffer byteBuffer = ByteBuffer.wrap(rawPositions);
            motePosition = new GeoPosition(byteBuffer.getFloat(0), byteBuffer.getFloat(4));
            destinationPosition = new GeoPosition(byteBuffer.getFloat(8), byteBuffer.getFloat(12));
        } else {
            // The mote has already sent the initial request
            //  -> only the current position of the mote is transmitted (the destination has been stored already)
            byte[] rawPositions = new byte[8];
            IntStream.range(0, 8).forEach(i -> rawPositions[i] = body.get(i));
            ByteBuffer byteBuffer = ByteBuffer.wrap(rawPositions);

            motePosition = new GeoPosition(byteBuffer.getFloat(0), byteBuffer.getFloat(4));
            destinationPosition = routes.get(deviceEUI).getRight().get(routes.get(deviceEUI).getRight().size()-1);
        }

        // Use the routing algorithm to calculate the K best paths for the mote
        //List<GeoPosition> routeMote=this.pathFinder.retrievePath(graph,motePosition,destinationPosition);
        List<Pair<Double,List<GeoPosition>>> routeMote = this.pathFinder.retrieveKPaths(graph, motePosition, destinationPosition,5);
        putKBestPathsBuffers(deviceEUI,routeMote);
        Pair<Double,List<GeoPosition>> bestPath;
        if(bufferKBestPaths.get(deviceEUI).size()==BUFFERSIZE) {
            bestPath = takeBestPath(bufferKBestPaths.get(deviceEUI));
            bufferKBestPaths = new HashMap<>();
            if (bestPath.getRight().size() > 0) {
                if (BUFFERSIZE > 1) {
                    this.routes.remove(0);
                    if (pathAnalyser.isBetterPath(this.routes.get(deviceEUI), bestPath)) {
                        bestPath = this.routes.get(deviceEUI);
                        //bestPath.getRight().remove(0);
                        this.routes.put(deviceEUI, bestPath);
                    }
                }
                else{
                    bestPath = this.routes.get(deviceEUI);
                    //if(bestPath.getRight().size()>1)

                    bestPath.getRight().remove(0);

                    //bestPath.getRight().remove(0);
                    this.routes.put(deviceEUI, bestPath);
                }
            }
            else{
                bestPath = this.routes.get(deviceEUI);
                //if(bestPath.getRight().size()>1)

                bestPath.getRight().remove(0);

                //bestPath.getRight().remove(0);
                this.routes.put(deviceEUI, bestPath);
            }

        }

        else {
            if (routes.get(deviceEUI) == null) {
                bestPath = this.pathFinder.retrievePath(graph,motePosition, destinationPosition);
                this.routes.put(deviceEUI, bestPath);
            } else {
                bestPath = this.routes.get(deviceEUI);
                bestPath.getRight().remove(0);
                this.routes.put(deviceEUI, bestPath);
            }
        }
        //List<GeoPosition> bestPath =
        // Compose the reply packet: up to 24 bytes for now, which can store 1 geoposition (in float)
        int amtPositions = Math.min(bestPath.getRight().size()-1, 2);
        ByteBuffer payloadRaw = ByteBuffer.allocate(8 * amtPositions);

        for (GeoPosition pos : bestPath.getRight().subList(1, amtPositions+1)) {
            payloadRaw.putFloat((float) pos.getLatitude());
            payloadRaw.putFloat((float) pos.getLongitude());
        }

        List<Byte> payload = new ArrayList<>();
        for (byte b : payloadRaw.array()) {
            payload.add(b);
        }

        // Update the position of the mote if it has changed since the previous time
        if (!lastPositions.containsKey(deviceEUI) || !lastPositions.get(deviceEUI).equals(motePosition)) {
            lastPositions.put(deviceEUI, motePosition);
        }

        // Send the reply (via MQTT) to the requesting device
        //.println("gggggg   " + deviceEUI);
        BasicMqttMessage routeMessage = new BasicMqttMessage(payload);
        this.mqttClient.publish(Topics.getAppToNetServer(message.getReceiverEUI(), deviceEUI), routeMessage);
    }




    private Pair<Double,List<GeoPosition>> takeBestPath(List<List<Pair<Double, List<GeoPosition>>>> bufferKBestPaths) {
        List<Pair<Double,List<GeoPosition>>> lastMeasure = bufferKBestPaths.get(bufferKBestPaths.size()-1);
        HashMap<Integer,Pair<Integer,Double>> average = new HashMap<>();
        for(int k = 0;k<= lastMeasure.size()-1;k++){
            Pair<Double,List<GeoPosition>> path = lastMeasure.get(k);
            //System.out.println("Cost" + path.getLeft());
            average.put(k,new  Pair(1,path.getLeft()));
            for (int i =0;i <= bufferKBestPaths.size()-2;i++)
            {
                for (int j = 0; j <= bufferKBestPaths.get(i).size()-1; j++)
                {
                    if(containsList(bufferKBestPaths.get(i).get(j).getRight(),path.getRight()))
                    {
                        //System.out.println(k);
                        int amount = average.get(k).getLeft() + 1;
                        double accumulatedCost = average.get(k).getRight() + bufferKBestPaths.get(i).get(j).getLeft();
                        average.put(k,new Pair(amount,accumulatedCost));
                        j = 10; //there couldn't be two same routes for the same measurement
                    }

                }
            }
        }
        Iterator it = average.entrySet().iterator();
        List<Pair<Integer,Double>> PathsIdWithMostAppearances = new ArrayList<>();
        int mostappearances = 0;
        while (it.hasNext()){
            Map.Entry pair = (Map.Entry)it.next();
            int iD = (Integer)pair.getKey();
            Pair<Integer,Double> value = (Pair<Integer, Double>)pair.getValue();
            if (value.getLeft()>mostappearances)
            {
                mostappearances = value.getLeft();
                PathsIdWithMostAppearances = new ArrayList<Pair<Integer,Double>>();
                Pair<Integer,Double>IdWithPath = new Pair(iD,value.getRight());
                PathsIdWithMostAppearances.add(IdWithPath);
            }
            else if (value.getLeft()==mostappearances)
            {
                Pair<Integer,Double>IdWithPath = new Pair(iD,value.getRight());
                PathsIdWithMostAppearances.add(IdWithPath);
            }
            it.remove();
        }
        double bestAverage = 1000000000;
        Pair<Double,List<GeoPosition>> bestRoute = lastMeasure.get(0);
        for (Pair<Integer,Double> o:PathsIdWithMostAppearances) {
            double averageCost = o.getRight()/mostappearances;
            if(averageCost < bestAverage)
            {
                bestAverage = averageCost;
                int index = o.getLeft();
                bestRoute = lastMeasure.get(index);
            }
        };
        return bestRoute;

    }


    private boolean containsList(List<GeoPosition> list, List<GeoPosition> sublist) {
        return Collections.indexOfSubList(list, sublist) != -1;
    }


    /**
     * Get a stored route for a specific mote.
     * @param mote The mote from which the cached route is requested.
     * @return The route as a list of geo coordinates.
     */
    public List<GeoPosition> getRoute(Mote mote) {
        if (routes.containsKey(mote.getEUI())) {
            return routes.get(mote.getEUI()).getRight();
        }
        return new ArrayList<>();
    }

    @Override
    public void consumePackets(String topicFilter, TransmissionWrapper transmission) {
        var message = transmission.getTransmission().getContent();
        // Only handle packets with a route request
        var messageType = message.getPayload()[0];
        if (messageType == MessageType.REQUEST_PATH.getCode() || messageType == MessageType.REQUEST_UPDATE_PATH.getCode()) {
            handleRouteRequest(message);
        }
    }

    /**
     * Clean the cached routes and mote positions.
     */
    public void clean() {
        this.routes = new HashMap<>();
        this.lastPositions = new HashMap<>();
        this.bufferKBestPaths = new HashMap<>();
    }
}
