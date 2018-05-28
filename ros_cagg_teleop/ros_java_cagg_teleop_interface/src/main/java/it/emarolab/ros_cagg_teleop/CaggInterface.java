package it.emarolab.ros_cagg_teleop;

import it.emarolab.cagg.core.evaluation.ThreadedEvaluator;
import it.emarolab.cagg.core.evaluation.inputFormatting.ThreadedInputFormatter;
import it.emarolab.cagg.core.evaluation.interfacing.EvaluatorBase;
import it.emarolab.cagg.core.evaluation.semanticGrammar.SemanticTree;
import it.emarolab.cagg.core.evaluation.semanticGrammar.syntaxCompiler.ActionTagBase;
import it.emarolab.cagg.core.evaluation.semanticGrammar.syntaxCompiler.GrammarBase;
import it.emarolab.cagg.interfaces.CaggCompiler;
import it.emarolab.cagg.interfaces.TestLog;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;


public class CaggInterface {

    private GrammarBase<SemanticTree> grammar;
    private long evaluationTimeout;
    private double stoppingThreshold;
    private long checkingFrequency;
    private String evaluatorName = "";

    // run the cagg-0.1.0.jar to open the Grammar Generator GUI
    public CaggInterface(String logConfig, String absoluteGrammarPath,
                         long timeout, int checkingFrequency, double stoppingThreshold,
                         String evaluatorName) {
        new it.emarolab.cagg.debugging.CaggLoggersManager( logConfig);
        grammar = CaggCompiler.deserialise( absoluteGrammarPath);
        this.evaluatorName = evaluatorName;
        this.evaluationTimeout = timeout;
        this.stoppingThreshold = stoppingThreshold;
        this.checkingFrequency = checkingFrequency;
        TestLog.info( "/" + evaluatorName +"new CAGG interface with: (logConfig:" + logConfig
                + ")(grammar:" + absoluteGrammarPath
                + ")(timeout:" + timeout
                + ")(stoppingThreshold:" + stoppingThreshold
                + ")(checkingFrequency:" + checkingFrequency + ")");
    }


    // called the main procedure after a grammar is compiled from source or loaded from serialisation file
    public ROSResult evaluate(String text) {
        // create the input formatter
        ThreadedInputFormatter formatter = new ThreadedInputFormatter( grammar);
        // create the evaluator (define the action trigger for evaluator)
        ROSEvaluator evaluator = new ROSEvaluator( formatter);
        // evaluate
        TestLog.info( "/" + evaluatorName +"CAGG is testing: \"" + text + "\"");
        evaluator.evaluate( text);

        try {
            long waitingTime = 0;
            while ( waitingTime <= evaluationTimeout){
                synchronized (this) {
                    ROSResult bestSoFar = getBestResult(evaluator.results);
                    if ( !bestSoFar.isEmpty()){
                        break; // stop if you find somethig
                    }
                    if ( !evaluator.isRunning())
                        break;
                    if (bestSoFar.getConfidence() > stoppingThreshold)
                        break; // stop searching if the recognition is good enough
                }
                Thread.sleep(checkingFrequency);
                waitingTime += checkingFrequency;
            }
            if ( waitingTime >= evaluationTimeout)
                TestLog.info( "/" + evaluatorName +" TIME OUT !!!");
        } catch (InterruptedException e) {
            TestLog.error( e);
        }
        evaluator.stop();

        ROSResult bestResult = getBestResult( evaluator.results);

        TestLog.info( "/" + evaluatorName +"CAGG end testing. Best solution: \"" + bestResult + "\" over results: " + evaluator.results);
        return bestResult;
    }

    private ROSResult getBestResult(List<ROSResult> results){
        ROSResult bestResult = new ROSResult(); // initialize as empty List<List<>>, with confidence 0
        for( ROSResult r : results){
            	/* two possible type of measurements:
                    - confidence: rate of recognized words in the whole sentence (i.e., relative
                    - cnt: number of recognized words (i.e., absolute)
			            hint: I used '<=' to get the last result (consider bug with [[]] v.s. [[GO]], namely 1 v.s. 1
                */
            if( bestResult.getRecognizedWorldCnt() <= r.getRecognizedWorldCnt()) //if( bestResult.confidence <= r.confidence)
                bestResult = r;
        }
        return  bestResult;
    }



    //define the behavior of the system when solution are found
    class ROSEvaluator extends ThreadedEvaluator {

        private List<ROSResult> results = new ArrayList<>();

        ///// default constructor
        public ROSEvaluator(ThreadedInputFormatter formatter) {
            super(formatter);
        }

        ///// CAGG evaluation interface
        @Override
        public void activateTrigger(EvaluationResults result) {
            // if the feasible result is true
            if (result.getResultOutcome()) {
                ROSResult rosResult = new ROSResult(result);
                synchronized (this) {
                    results.add(rosResult);
                }
                TestLog.info( "/" + evaluatorName +"CAGG found a new Results: " + rosResult);
            }
        }
    }



    class ROSResult {

        private EvaluatorBase.EvaluationResults result;
        private float confidence = 0.0f; // in [0,1] = #ofTags/#ofWords
        private final List<List<String>> semanticTags;

        public ROSResult(){
            // make empty result
            semanticTags = new ArrayList<>();
            semanticTags.add( new ArrayList<>());
        }

        public ROSResult(EvaluatorBase.EvaluationResults result){
            this.result = result;
            this.confidence = computeConfidence( result);
            this.semanticTags = parseTags( result);
        }

        private float computeConfidence(EvaluatorBase.EvaluationResults result){
            return (float) result.getResultTags().size() / (float) result.getUsedInput().size();
        }

        private List< List< String>> parseTags(EvaluatorBase.EvaluationResults result){

            List< List< String>> tagList = new ArrayList<>();

            Set<ActionTagBase.TagBase<?>> actionByWords = result.getResultTags().getTagsCollector();
            for (ActionTagBase.TagBase<?> wordTags : actionByWords) { // for each collection of tags related to a word
                List< String> tagFields = new ArrayList<>();

                for ( Object tagsString : wordTags.getTagList()) { // for each String tags of the same word
                    String tag = String.valueOf( tagsString);
                    tagFields.add( tag);
                }
                tagList.add( tagFields);
            }

            return tagList;
        }

        /*
        public EvaluatorBase.EvaluationResults getResults(){
            return result;
        }
        */

        public List< List< String>> getTags(){
            return semanticTags;
        }

        public float getConfidence(){
            return confidence;
        }

        public int getRecognizedWorldCnt(){
            return semanticTags.size();
        }

        public boolean isGO(){
            for (List<String> tags: semanticTags)
                for (String t : tags)
                    if (t.equals("GO")) // tags fixed on grammar
                        return true;
            return false;
        }

        public boolean isNotGO(){
            for (List<String> tags: semanticTags)
                for (String t : tags)
                    if (t.equals("STOP") | t.equals("RESET")) // tags fixed on grammar
                        return true;
            return false;
        }

        public boolean isEmpty(){ // check if isEmpty or if first element is ""
            if ( semanticTags.isEmpty()) {
                return true;
            } else {
                if ( semanticTags.size() == 1){
                    List<String> tags = semanticTags.get( 0);
                    if (tags.isEmpty()) {
                        return true;
                    } else if( tags.size() == 1) {
                        String t = tags.get( 0);
                        if ( t.isEmpty())
                           return true;
                    }
                }
                return false;
            }
        }

        public String toString(){
            return "{conf:" + confidence + ":" + semanticTags + "}";
        }

        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (!(o instanceof ROSResult)) return false;

            ROSResult rosResult = (ROSResult) o;

            if (Float.compare(rosResult.getConfidence(), getConfidence()) != 0) return false;
            return semanticTags != null ? semanticTags.equals(rosResult.semanticTags) : rosResult.semanticTags == null;
        }

        @Override
        public int hashCode() {
            int result = (getConfidence() != +0.0f ? Float.floatToIntBits(getConfidence()) : 0);
            result = 31 * result + (semanticTags != null ? semanticTags.hashCode() : 0);
            return result;
        }
    }
}










