package it.emarolab.ros_cagg_node;

import it.emarolab.cagg.interfaces.TestLog;
import junit.framework.Test;
import org.apache.commons.logging.Log;

import org.ros.concurrent.CancellableLoop;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import java.lang.*;
import java.util.List;
import java.util.ArrayList;

// Node name: "CAGG_node"
// Listens a String at: /CAGG/input_text/
// Reply with a String[String[]] at: /CAGG/semantic_tags/
public class CaggNode extends AbstractNodeMain {

    // required parameters
    public static final String PARAM_NAME_LOG_CONFIG = "/cagg_log_config_path";
    public static final String PARAM_NAME_DIRECTIVE_GRAMMAR = "/cagg_serialized_directive_grammar";
    public static final String PARAM_NAME_GO_GRAMMAR = "/cagg_serialized_go_grammar";
    // auxiliary parameter
    public static final String PARAM_NAME_STOPPING_CONFIDENCE = "/cagg_stopping_confidence_threshold";
    public static final String PARAM_NAME_STOPIING_CHECK_FREQUENCY = "/cagg_stopping_check_frequency";
    public static final String PARAM_NAME_CAGG_TIMEOUT_MS = "/cagg_timeout_ms";
    public static final int DEFAULT_CAGG_TIMEOUT = 10000;//in millisec
    public static final double DEFAULT_STOPPING_THRESHOLD = .2f; // [0,1]
    public static final int DEFAULT_CHEKING_FREQUENCY = 1000; // in millisec

    public static final long SPINNING_RATE = 1000;

    private CaggInterface caggDirective, caggGo;
    // buffer to be published at the next spin, (it gets elements on the callback)
    private List< CaggInterface.ROSResult> semanticTags = new ArrayList<>();

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("CAGG_teleop");
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        final Log log = connectedNode.getLog();
        Subscriber<std_msgs.String> subscriber = connectedNode.newSubscriber("/CAGG/input_text/", std_msgs.String._TYPE);

        ParameterTree params = connectedNode.getParameterTree();
        String logFile = params.getString( PARAM_NAME_LOG_CONFIG, "");
        String directiveGrammarFile = params.getString( PARAM_NAME_DIRECTIVE_GRAMMAR, "");
        String goGrammarFile = params.getString( PARAM_NAME_GO_GRAMMAR, "");

        int timeout = params.getInteger( PARAM_NAME_CAGG_TIMEOUT_MS, DEFAULT_CAGG_TIMEOUT);
        int chekingFrequency = params.getInteger( PARAM_NAME_STOPIING_CHECK_FREQUENCY,DEFAULT_CHEKING_FREQUENCY);
        double stoppingThreshold = params.getDouble( PARAM_NAME_STOPPING_CONFIDENCE, DEFAULT_STOPPING_THRESHOLD);

        caggDirective = new CaggInterface(logFile, directiveGrammarFile, timeout, chekingFrequency, stoppingThreshold, "DIREC_EVAL");
        caggGo = new CaggInterface(logFile, goGrammarFile, timeout, chekingFrequency, stoppingThreshold, "GO_EVAL");

        // SUBSCRIBER callback
        subscriber.addMessageListener(message -> {
            // split message by and, or
            List<String> texts = splitText(message.getData());
            synchronized (this) {
                for (String t : texts) {
                    // apply directive grammar: "go" | "sop" | "reset"
                    CaggInterface.ROSResult directive = caggDirective.evaluate(t);
                    if (directive.isEmpty() | directive.isGO()) {
                        CaggInterface.ROSResult go = caggGo.evaluate(t); // {GO} !optional(<qualifier)> <relation> <objectNumber>
                        semanticTags.add(go);
                    } else semanticTags.add(directive); // [RESET] or [STOP]
                }
            }
        });

        // publisher and spin loop
        final Publisher<ros_cagg_msgs.cagg_tags> caggEvaluationPublisher = connectedNode.newPublisher("/CAGG/semantic_tags/", ros_cagg_msgs.cagg_tags._TYPE);
        connectedNode.executeCancellableLoop(new CancellableLoop() {
            private int sequenceNumber;
            @Override
            protected void setup() {sequenceNumber = 0;}

            @Override
            protected void loop() throws InterruptedException {
                sequenceNumber++;

                // publish only if data is available and clean semanticTags msg
                synchronized (this) {
                    List<CaggInterface.ROSResult> toRemove = new ArrayList<>();
                    // publish a String[String[]] created in the callback
                    for( CaggInterface.ROSResult st : semanticTags){
                        ros_cagg_msgs.cagg_tags caggEvaluationMsg = caggEvaluationPublisher.newMessage();
                        caggEvaluationMsg.setConfidence( st.getConfidence());

                        List<ros_cagg_msgs.cagg_tag> tagsMsg = new ArrayList<>();
                        for (List<String> word : st.getTags()) {
                            ros_cagg_msgs.cagg_tag caggSemanticTagsMsg = connectedNode.getTopicMessageFactory().newFromType(ros_cagg_msgs.cagg_tag._TYPE);
                            caggSemanticTagsMsg.setCaggTag(word);
                            tagsMsg.add(caggSemanticTagsMsg);
                        }

                        caggEvaluationMsg.setCaggTags(tagsMsg);
                        Time time = Time.fromMillis(System.currentTimeMillis());
                        caggEvaluationMsg.getHeader().setStamp(time);
                        caggEvaluationPublisher.publish(caggEvaluationMsg);
                        toRemove.add(st);
                    }
                    // clean only the published
                    semanticTags.removeAll( toRemove);
                }

                // spin rate in milliseconds
                Thread.sleep( SPINNING_RATE);
            }
        });
    }

    // split a string when words 'and' and 'also' occur.
    private List< String> splitText(String text) {
        text = text.replaceAll("[^A-Za-z0-9 ]", " ").trim();
        String[] splitByAnd = text.split(" and ");
        List< String> splittedText = new ArrayList<>();
        for( String s : splitByAnd){
            String[] splitByAlso = s.split( " also ");
            for ( String ss: splitByAlso)
                splittedText.add( ss);
        }
        TestLog.info( "Testing sentences: " + splittedText);
        return splittedText;
    }
}
