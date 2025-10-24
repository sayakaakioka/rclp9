package rclp9;

import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;

import rclp9.testutil.PubSubLauncher;
import java.util.ArrayList;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.assertEquals;

record Case(String className) {
    static Stream<Case> all() {
        return Stream.of(
            new Case("builtin_interfaces.msg.Time"),
            new Case("std_msgs.msg.Float64"),
            new Case("std_msgs.msg.Header"),
            new Case("std_msgs.msg.String"),
            new Case("geometry_msgs.msg.Point"),
            new Case("geometry_msgs.msg.Pose"),
            new Case("geometry_msgs.msg.PoseStamped"),
            new Case("geometry_msgs.msg.Quaternion"),
            new Case("geometry_msgs.msg.Twist"),
            new Case("geometry_msgs.msg.Vector3")
        );
    }
}

public class PubSubTests {
    static Stream<Case> cases() {
        return Case.all();
    }

    @ParameterizedTest(name="[{index}] {0}")
    @MethodSource("cases")
    void test(Case c) throws Exception {
        String msgClsName = c.className().substring(c.className().lastIndexOf('.') + 1);
        String publisherName = "rclp9.examples." + msgClsName + "_publisher";
        String subscriberName = "rclp9.examples." + msgClsName + "_subscriber";

        try {
            var ret = PubSubLauncher.run(
                    publisherName, l -> l.contains("PUBLISHER READY"), l -> l.contains("PUBLISHER DONE"), 10,
                    subscriberName, l -> l.contains("SUBSCRIBER READY"), l -> l.contains("Received:"), 10);

            // check publisher/subscriber completed successfully
            assertEquals(0, ret.subscriberWatcher(), "Subscriber failed:\n" + ret.subscriberLog());
            assertEquals(0, ret.publisherWatcher(), "Publisher failed:\n" + ret.publisherLog());

            List<String> published = parsePublishedValues(ret.publisherLog());
            List<String> received = parseReceivedValues(ret.subscriberLog());

            // check the number if published/received messages
            assertEquals(published.size(), received.size(),
                    () -> "Mismatch in number of messages: published = " + published.size()
                    + ", received = " + received.size()
            );

            // check the contents
            for (String pubStr : published) {
                for (String resvStr : received) {
                    assertEquals(pubStr, resvStr,
                            () -> "Message mismatch: "
                                    + "\n\tpublished: [" + pubStr + "]"
                                    + "\n\treceived:  [" + resvStr + "]");
                }
            }
            
            System.out.println(
                "\n----- Publisher Log -----\n" + ret.publisherLog()
                + "\n----- Subscriber Log -----\n" + ret.subscriberLog()
            );
            System.out.flush();
        }catch(Exception e){
            e.printStackTrace();
        }
    }
    
    private static List<String> parsePublishedValues(String log) {
        List<String> ret = new ArrayList<>();
        Pattern p = Pattern.compile("Publishing:\\s*\\[(.*?)\\]");
        Matcher m = p.matcher(log);
        while (m.find()) {
            ret.add(m.group(1));
        }
        return ret;
    }

    private static List<String> parseReceivedValues(String log) {
        List<String> ret = new ArrayList<>();
        Pattern p = Pattern.compile("Received:\\s*(.*)");
        Matcher m = p.matcher(log);
        while (m.find()) {
            ret.add(m.group(1));
        }
        return ret;
    }
}