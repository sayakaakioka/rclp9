package rclp9.testutil;

import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.util.concurrent.TimeUnit;
import java.util.function.Predicate;

public class PubSubLauncher {
    // set of results
    public record Result(int publisherWatcher, int subscriberWatcher, String publisherLog, String subscriberLog) {
    }

    public static Result run(
            String publisherClassName, Predicate<String> publisherReady, Predicate<String> publisherDone,
            long publisherTimeoutSec, String subscriberClassName, Predicate<String> subscriberReady,
            Predicate<String> subscriberDone, long subscriberTimeoutSec) throws Exception {
        // launch the subscriber first
        Process subscriber = start(subscriberClassName);

        StringBuilder subscriberStrBuilder = new StringBuilder();
        BufferedReader subscriberReader = new BufferedReader(
                new InputStreamReader(subscriber.getInputStream(), StandardCharsets.UTF_8));
        assertTrue(waitFor(subscriberReader, subscriberStrBuilder, subscriberReady, subscriberTimeoutSec),
                "Receiver did not wake up:\n" + subscriberStrBuilder);

        // launch and wait for the publisher
        Process publisher = start(publisherClassName);
        StringBuilder publisherStrBuilder = new StringBuilder();
        BufferedReader publisherReader = new BufferedReader(
                new InputStreamReader(publisher.getInputStream(), StandardCharsets.UTF_8));
        assertTrue(waitFor(publisherReader, publisherStrBuilder, publisherReady, publisherTimeoutSec),
                "Publisher did not wake up:\n" + publisherStrBuilder);
        assertTrue(waitFor(publisherReader, publisherStrBuilder, publisherDone, publisherTimeoutSec),
                "Publisher did not complete:\n" + publisherStrBuilder);
        int publisherWatcher = publisher.waitFor();

        // wait for the subscriber
        assertTrue(waitFor(subscriberReader, subscriberStrBuilder, subscriberDone, subscriberTimeoutSec),
                "Subscriber did not complete:\n" + subscriberStrBuilder);
        int subscriberWatcher = subscriber.waitFor();

        return new Result(publisherWatcher, subscriberWatcher, publisherStrBuilder.toString(),
                subscriberStrBuilder.toString());
    }

    // node launcher
    private static Process start(String className) throws IOException {
        String java = System.getProperty("java.home") + File.separator + "bin" + File.separator + "java";
        String classpath = System.getProperty("java.class.path");
        String libpath = System.getProperty("java.library.path");
        ProcessBuilder processBuilder = new ProcessBuilder();

        processBuilder.command().add(java);

        if (libpath != null && !libpath.isEmpty()) {
            processBuilder.command().add("-Djava.library.path=" + libpath);
        }

        processBuilder.command().add("-cp");
        processBuilder.command().add(classpath);
        processBuilder.command().add(className);

        String display = System.getenv("DISPLAY");
        if (display != null && !display.isEmpty()) {
            processBuilder.environment().put("DISPLAY", display);
        } else {
            processBuilder.environment().put("DISPLAY", "");
        }

        processBuilder.redirectErrorStream(true);

        return processBuilder.start();
    }

    private static boolean waitFor(BufferedReader br, StringBuilder sb, Predicate<String> predicate, long timeoutSec)
            throws Exception {
        long deadline = System.nanoTime() + TimeUnit.SECONDS.toNanos(timeoutSec);
        while (System.nanoTime() < deadline) {
            String ret = pollLine(br, (long) 200);
            if (ret == null) {
                continue;
            }
            //System.err.println("ret: " + ret);

            sb.append(ret).append("\n");
            if (predicate.test(ret)) {
                return true;
            }
        }
        return false;
    }

    private static String pollLine(BufferedReader br, Long timeoutMsec) throws Exception {
        long deadline = System.nanoTime() + TimeUnit.MILLISECONDS.toNanos(timeoutMsec);
        while (System.nanoTime() < deadline) {
            if (br.ready()) {
                return br.readLine();
            }
            Thread.sleep(10);
        }
        return null;
    }

    private static void assertTrue(boolean condition, String msg) {
        if (!condition) {
            throw new AssertionError(msg);
        }
    }

}
