package iot;

import application.pollution.PollutionGrid;
import application.routing.RouteEvaluator;
import application.routing.RoutingApplicationNew;
import application.routing.heuristic.RoutingHeuristic;
import application.routing.heuristic.SimplePollutionHeuristic;
import be.kuleuven.cs.som.annotate.Basic;
import datagenerator.SensorDataGenerator;
import iot.networkentity.Gateway;
import iot.networkentity.Mote;
import iot.networkentity.MoteSensor;
import iot.networkentity.UserMote;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.jxmapviewer.viewer.GeoPosition;
import selfadaptation.feedbackloop.GenericFeedbackLoop;
import util.Connection;
import util.TimeHelper;

import javax.swing.*;
import java.lang.ref.WeakReference;
import java.time.LocalTime;
import java.util.*;
import java.util.function.Predicate;

/**
 * A class representing a simulation.
 */
public class Simulation {

    // region fields
    /**
     * The InputProfile used in the simulation.
     */
    private InputProfile inputProfile;
    /**
     * The Environment used in th simulation.
     */
    private WeakReference<Environment> environment;
    /**
     * The GenericFeedbackLoop used in the simulation.
     */
    private GenericFeedbackLoop approach;

    /**
     * A condition which determines if the simulation should continue (should return {@code false} when the simulation is finished).
     */
    private Predicate<Environment> continueSimulation;

    private boolean finished;

    /**
     * Intermediate parameters used during simulation
     */
    private Map<Mote, Integer> wayPointMap;
    private Map<Mote, LocalTime> timeMap;
    private RouteEvaluator routeEvaluator;
    private Map<Long,Integer> amount;
    private RoutingApplicationNew routingApplicationNew;
    private Map<Long,List<Double>> information;
    private Integer time;
    // endregion

    // region constructors

    public Simulation(PollutionGrid pollutionGrid) {
        this.routeEvaluator = new RouteEvaluator(pollutionGrid);
        this.finished = false;
    }


    // endregion

    // region getter/setters

    /**
     * Gets the Environment used in th simulation.
     * @return The Environment used in the simulation.
     */
    @Basic
    public Environment getEnvironment() {
        return environment.get();
    }
    /**
     * Sets the Environment used in th simulation.
     * @param environment  The Environment to use in the simulation.
     */
    @Basic
    public void setEnvironment(WeakReference<Environment> environment) {
        this.environment = environment;
        this.routeEvaluator.setEnvironment(environment);
    }
    @Basic
    public void setRoutingApplicationNew(RoutingApplicationNew routingApplicationNew)
    {
        this.routingApplicationNew = routingApplicationNew;
    }

    @Basic
    public Map<Long,List<Double>> getInformation(){
        return this.information;
    }

    @Basic
    public void setInformation() {
        this.information = new HashMap<>();
    }



    /**
     * Gets the InputProfile used in th simulation.
     * @return The InputProfile used in the simulation.
     */
    @Basic
    public Optional<InputProfile> getInputProfile() {
        return Optional.ofNullable(inputProfile);
    }
    /**
     * Sets the InputProfile used in th simulation.
     * @param inputProfile  The InputProfile to use in the simulation.
     */
    @Basic
    public void setInputProfile(InputProfile inputProfile) {
        this.inputProfile = inputProfile;
    }

    /**
     * Gets the GenericFeedbackLoop used in th simulation.
     * @return The GenericFeedbackLoop used in the simulation.
     */
    @Basic
    public GenericFeedbackLoop getAdaptationAlgorithm() {
        return approach;
    }
    /**
     * Sets the GenericFeedbackLoop used in th simulation.
     * @param approach  The GenericFeedbackLoop to use in the simulation.
     */
    @Basic
    public void setAdaptationAlgorithm(GenericFeedbackLoop approach) {
        this.approach = approach;
    }


    public GenericFeedbackLoop getApproach() {
        return approach;
    }
    /**
     * Sets the GenericFeedbackLoop.
     * @param approach The GenericFeedbackLoop to set.
     */
    @Basic
    void setApproach(GenericFeedbackLoop approach) {
        if (getApproach()!= null) {
            getApproach().stop();
        }
        this.approach = approach;
        getApproach().start();
    }
    // endregion


    /**
     * Gets the probability with which a mote should be active from the input profile of the current simulation.
     * If no probability is specified, the probability is set to one.
     * Then it performs a pseudo-random choice and sets the mote to active/inactive for the next run, based on that probability.
     */
    private void setupMotesActivationStatus() {
        List<Mote> motes = this.getEnvironment().getMotes();
        Set<Integer> moteProbabilities = this.inputProfile.getProbabilitiesForMotesKeys();
        for (int i = 0; i < motes.size(); i++) {
            Mote mote = motes.get(i);
            double activityProbability = 1;
            if (moteProbabilities.contains(i))
                activityProbability = this.inputProfile.getProbabilityForMote(i);
            if (Math.random() >= 1 - activityProbability)
                mote.enable(true);
        }
    }

    /**
     * Check if all motes have arrived at their destination.
     * @return True if the motes are at their destinations.
     */
    private boolean areAllMotesAtDestination() {
        return this.getEnvironment().getMotes().stream()
            .allMatch(m -> !m.isEnabled() || m.isArrivedToDestination());
    }


    /**
     * Simulate a single step in the simulator.
     */
    public void simulateStep() {
        //noinspection SimplifyStreamApiCallChains
        this.getEnvironment().getMotes().stream()
            .filter(Mote::isEnabled)
            .filter(mote-> !(mote.isArrivedToDestination()))
            .map(mote -> { mote.consumePackets();return mote;})
            //DON'T replace with peek because the filtered mote after this line will not do the consume packet
            .filter(mote ->  mote.getPath().getWayPoints().size() > wayPointMap.get(mote))
            //.filter(mote -> !(mote instanceof UserMote) && mote.getPath().getWayPoints().size() > wayPointMap.get(mote))
            .filter(mote -> TimeHelper.secToMili( 1 / mote.getMovementSpeed()) <
                TimeHelper.nanoToMili(this.getEnvironment().getClock().getTime().toNanoOfDay() - timeMap.get(mote).toNanoOfDay()))
            .filter(mote -> TimeHelper.nanoToMili(this.getEnvironment().getClock().getTime().toNanoOfDay()) > TimeHelper.secToMili(Math.abs(mote.getStartMovementOffset())))
            .forEach(mote -> {
                timeMap.put(mote, this.getEnvironment().getClock().getTime());
                //if(mote instanceof UserMote && amount > 1)
                //{
                //    if (!this.getEnvironment().getMapHelper().toMapCoordinate(routingApplicationNew.getRoute(mote).get(3)).equals(mote.getPosInt())) {
                //        this.getEnvironment().moveMote(mote, routingApplicationNew.getRoute(mote).get(3));
                //    }
                //    else{
                //        amount = 0;
                //        }
                //}
                    if (!this.getEnvironment().getMapHelper().toMapCoordinate(mote.getPath().getWayPoints().get(wayPointMap.get(mote))).equals(mote.getPosInt())) {
                        this.getEnvironment().moveMote(mote, mote.getPath().getWayPoints().get(wayPointMap.get(mote)));
                        //List<Connection> usedConnection = environment.get().getGraph().getOutgoingConnections(environment.get().getGraph().getClosestWayPoint(environment.get().getMapHelper().toGeoPosition(mote.getPosInt())));
                        //System.out.println(firstVisitedWaypointByMote.get(mote.getEUI())==null);

                        if (mote instanceof UserMote) {
                            routeEvaluator.addCostConnectionOfMote(mote, wayPointMap.get(mote));
                            //System.out.println(mote.getPath().getWayPoints().get(wayPointMap.get(mote)));
                            //System.out.println(routeEvaluator.getTotalCostPath(mote.getEUI()));
                        }


                        //System.out.println("pppp");
                        //System.out.println(getEnvironment().getGraph().getClosestWayPoint(mote.getPath().getWayPoints().get(wayPointMap.get(mote))));
                        //System.out.println(environment.get().getGraph().getWayPoint(o.getTo()));
                        //if(mote.getPath().getWayPoints().get(wayPointMap.get(mote)).equals(environment.get().getGraph().getWayPoint(o.getFrom())))
                        //{
                        //System.out.println("Connection");
                        //System.out.println(o.getFrom());
                        //
                    } else {
                        int index = wayPointMap.put(mote, wayPointMap.get(mote) + 1);
                        if (mote instanceof UserMote && mote.getPath().getWayPoints().size() > wayPointMap.get(mote)) {
                            try {
                                while (this.getEnvironment().getMapHelper().toMapCoordinate(mote.getPath().getWayPoints().get(wayPointMap.get(mote))).equals(mote.getPosInt())) {
                                    wayPointMap.put(mote, wayPointMap.get(mote) + 1);
                                }
                                this.getEnvironment().moveMote(mote, mote.getPath().getWayPoints().get(wayPointMap.get(mote)));
                                routeEvaluator.addCostConnectionOfMote(mote, wayPointMap.get(mote));
                            }
                            catch(java.lang.IndexOutOfBoundsException e)
                            {
                                wayPointMap.put(mote, index );
                                amount.put(mote.getEUI(),amount.get(mote.getEUI())+1);
                                if (amount.get(mote.getEUI()) > 1) {
                                    //System.out.println(getEnvironment().getGraph().getClosestWayPoint(getEnvironment().getMapHelper().toGeoPosition(mote.getPosInt())));
                                    this.getEnvironment().moveMote(mote, routingApplicationNew.getRoute(mote).get(1));
                                    //System.out.println(mote.getEUI() + " :   " + routingApplicationNew.getRoute(mote).get(1));
                                    //System.out.println(getEnvironment().getGraph().getClosestWayPoint(routingApplicationNew.getRoute(mote).get(1)));
                                    List<GeoPosition> path = mote.getPath().getWayPoints();
                                    path.add(routingApplicationNew.getRoute(mote).get(1));
                                    mote.setPath(path);
                                    amount.put(mote.getEUI(),0);
                                    routeEvaluator.addCostConnectionOfMote(mote, wayPointMap.get(mote));
                                }
                            }

                            //wayPointMap.put(mote,0);
                            //amount = 0;
                        }
                        else {
                            if (mote instanceof UserMote) {
                                amount.put(mote.getEUI(),amount.get(mote.getEUI())+1);
                                if (amount.get(mote.getEUI()) > 1) {
                                    //System.out.println(getEnvironment().getGraph().getClosestWayPoint(getEnvironment().getMapHelper().toGeoPosition(mote.getPosInt())));
                                    this.getEnvironment().moveMote(mote, routingApplicationNew.getRoute(mote).get(1));
                                    //System.out.println(mote.getEUI() + " :   " + routingApplicationNew.getRoute(mote).get(1));
                                    //System.out.println(getEnvironment().getGraph().getClosestWayPoint(routingApplicationNew.getRoute(mote).get(1)));
                                    List<GeoPosition> path = mote.getPath().getWayPoints();
                                    path.add(routingApplicationNew.getRoute(mote).get(1));
                                    mote.setPath(path);
                                    amount.put(mote.getEUI(),0);
                                    routeEvaluator.addCostConnectionOfMote(mote, wayPointMap.get(mote));
                                }
                                //System.out.println(mote.getEUI());
                            }
                        }

                    }

            });
        if(!(getApproach() == null) && getApproach().getName()=="Get Information") {
            if (time % 4000 == 0) {
                updateInformation();
            }
            time += 1;
        }
        this.getEnvironment().getClock().tick(1);

        //amount = 0;
    }

    private void updateInformation()
    {
        environment.get().getGraph().getConnections().entrySet().stream()
            .forEach(entry -> {
                Double cost = routeEvaluator.getCostConnection(entry.getValue());
                if(information.containsKey(entry.getKey())){
                    information.get(entry.getKey()).add(information.get(entry.getKey()).size() - 1, cost);
                    information.put(entry.getKey(), information.get(entry.getKey()));
                }
                else
                {
                    List<Double> list = new ArrayList<>();
                    list.add(0,cost);
                    information.put(entry.getKey(),list);
                    }
            }
            );


    }

    public boolean isFinished() {
        if(!this.continueSimulation.test(this.getEnvironment()) && this.finished == false)
        {
            String message = "Evaluation of the path \n";
            for(Mote mote: getEnvironment().getMotes())
            {
                if(mote instanceof UserMote)
                {
                    message += "EUI: " + mote.getEUI() +  "  :    " + routeEvaluator.getTotalCostPath(mote.getEUI())+ "\n";
                }
            }
            JOptionPane.showMessageDialog(null, message, "Results", JOptionPane.INFORMATION_MESSAGE);
            this.finished = true;
        }
        if(!(getApproach() == null) && getApproach().getName()=="Get Information")
        {
            if(time/1000 >= 4000)
            {
                String message = "Finished \nPress on save to write the information about the evolution of the values for all connections on a xml-file";
                JOptionPane.showMessageDialog(null, message, "Information gathered", JOptionPane.INFORMATION_MESSAGE);
                return true;
            }
        }
        else {
            return !this.continueSimulation.test(this.getEnvironment());
        }
        return false;
    }


    private void setupSimulation(Predicate<Environment> pred) {
        this.finished = false;
        this.time = 0;
        this.information = new HashMap<>();
        this.wayPointMap = new HashMap<>();
        this.timeMap = new HashMap<>();
        this.routeEvaluator.reset();
        amount = new HashMap<>();
        getEnvironment().getMotes().stream()
            .filter(mote-> mote instanceof UserMote)
            .forEach(mote-> amount.put(mote.getEUI(),0));

        setupMotesActivationStatus();

        this.getEnvironment().getGateways().forEach(Gateway::reset);

        this.getEnvironment().getMotes().forEach(mote -> {
            // Reset all the sensors of the mote
            mote.getSensors().stream()
                .map(MoteSensor::getSensorDataGenerator)
                .forEach(SensorDataGenerator::reset);

            // Initialize the mote (e.g. reset starting position)
            mote.reset();

            timeMap.put(mote, this.getEnvironment().getClock().getTime());
            wayPointMap.put(mote,0);

            // Add initial triggers to the clock for mote data transmissions (transmit sensor readings)
            this.getEnvironment().getClock().addTrigger(LocalTime.ofSecondOfDay(mote.getStartSendingOffset()), () -> {
                mote.sendToGateWay(
                    mote.getSensors().stream()
                        .flatMap(s -> s.getValueAsList(mote.getPosInt(), this.getEnvironment().getClock().getTime()).stream())
                        .toArray(Byte[]::new),
                    new HashMap<>());
                return this.getEnvironment().getClock().getTime().plusSeconds(mote.getPeriodSendingPacket());
            });
        });

        this.continueSimulation = pred;
    }

    void setupSingleRun(boolean shouldResetHistory) {
        if (shouldResetHistory) {
            this.getEnvironment().resetHistory();
        }

        this.setupSimulation((env) -> !areAllMotesAtDestination());
    }

    void setupTimedRun() {
        this.getEnvironment().resetHistory();

        var finalTime = this.getEnvironment().getClock().getTime()
            .plus(inputProfile.getSimulationDuration(), inputProfile.getTimeUnit());
        this.setupSimulation((env) -> env.getClock().getTime().isBefore(finalTime));
    }
}
