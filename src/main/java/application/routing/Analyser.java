package application.routing;

import application.routing.heuristic.RoutingHeuristic;
import iot.Environment;
import iot.mqtt.MQTTClientFactory;
import iot.mqtt.MqttClientBasicApi;
import iot.mqtt.TransmissionWrapper;
import iot.networkentity.Mote;
import iot.networkentity.MoteSensor;
import org.jxmapviewer.viewer.GeoPosition;
import util.Pair;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public abstract class Analyser {



    protected Analyser(){};

    /**
     * Destructor which can be used to properly destruct analysers
     */
    public void destruct() {}
}

