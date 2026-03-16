package frc.robot.Util;

import edu.wpi.first.networktables.*;
import java.util.HashMap;
import java.util.Map;

/*
 * Example code:
 * 
 * NTReader reader = new NTReader(
    new NTReader.Config.Builder("shooter")
        .staleThresholdMs(30.0)
        .build()
);

// In periodic:
reader.refresh();
double angle  = reader.getDouble("angle", 0.0);
double rpm    = reader.getDouble("rpm", 0.0);
boolean ready = reader.getBoolean("ready", false);

if (reader.isFresh("angle")) {
    // solution is recent enough to act on
}
 * 
 */

public class NTFastReader {

    public static class Config {
        public final String tableName;
        public final long staleThresholdMicros;

        private Config(Builder b) {
            this.tableName = b.tableName;
            this.staleThresholdMicros = b.staleThresholdMicros;
        }

        public static class Builder {
            private final String tableName;
            private long staleThresholdMicros = 50_000; // 50ms default

            public Builder(String tableName) {
                this.tableName = tableName;
            }

            public Builder staleThresholdMs(double ms) {
                this.staleThresholdMicros = (long) (ms * 1000);
                return this;
            }

            public Config build() {
                return new Config(this);
            }
        }
    }

    private final NetworkTableInstance nt;
    private final NetworkTable table;
    private final Config config;

    // Subscriber caches — created once on first access, reused forever
    private final Map<String, DoubleSubscriber> doubleSubs = new HashMap<>();
    private final Map<String, BooleanSubscriber> boolSubs = new HashMap<>();
    private final Map<String, StringSubscriber> stringSubs = new HashMap<>();
    private final Map<String, IntegerSubscriber> intSubs = new HashMap<>();
    private final Map<String, DoubleArraySubscriber> doubleArraySubs = new HashMap<>();

    /** Last subscriber touched — used for freshness check */
    private String lastReadKey = null;

    public NTFastReader(Config config) {
        this(NetworkTableInstance.getDefault(), config);
    }

    public NTFastReader(NetworkTableInstance nt, Config config) {
        this.nt = nt;
        this.config = config;
        this.table = nt.getTable(config.tableName);
    }

    // -------------------------------------------------------------------------
    // Reads
    // -------------------------------------------------------------------------

    public double getDouble(String key, double defaultValue) {
        lastReadKey = key;
        return doubleSubs
                .computeIfAbsent(key, k -> table.getDoubleTopic(k).subscribe(defaultValue))
                .get();
    }

    public boolean getBoolean(String key, boolean defaultValue) {
        lastReadKey = key;
        return boolSubs
                .computeIfAbsent(key, k -> table.getBooleanTopic(k).subscribe(defaultValue))
                .get();
    }

    public String getString(String key, String defaultValue) {
        lastReadKey = key;
        return stringSubs
                .computeIfAbsent(key, k -> table.getStringTopic(k).subscribe(defaultValue))
                .get();
    }

    public long getInteger(String key, long defaultValue) {
        lastReadKey = key;
        return intSubs
                .computeIfAbsent(key, k -> table.getIntegerTopic(k).subscribe(defaultValue))
                .get();
    }

    public double[] getDoubleArray(String key, double[] defaultValue) {
        lastReadKey = key;
        return doubleArraySubs
                .computeIfAbsent(key, k -> table.getDoubleArrayTopic(k).subscribe(defaultValue))
                .get();
    }

    // -------------------------------------------------------------------------
    // Freshness — checks the server timestamp of a specific key
    // -------------------------------------------------------------------------

    /**
     * Returns true if the given key's last published value arrived within the stale
     * threshold.
     * Uses NT server timestamp, so it's robust to clock differences between
     * coprocessor and roboRIO.
     */
    public boolean isFresh(String key) {
        DoubleSubscriber sub = doubleSubs.get(key);
        if (sub == null)
            return false;

        long publishedAt = sub.getAtomic().serverTime;
        if (publishedAt == 0)
            return false; // never published

        long nowMicros = (long) (nt.getServerTimeOffset().orElse(0L)
                + System.currentTimeMillis() * 1000L);
        return (nowMicros - publishedAt) < config.staleThresholdMicros;
    }

    /**
     * Convenience overload — checks freshness of the last key that was read.
     */
    public boolean isFresh() {
        if (lastReadKey == null)
            return false;
        return isFresh(lastReadKey);
    }

    // -------------------------------------------------------------------------
    // Flush
    // -------------------------------------------------------------------------

    /**
     * Flush outgoing NT messages. Call at the top of your periodic loop before
     * reading.
     */
    public void refresh() {
        nt.flush();
    }

    // -------------------------------------------------------------------------
    // Cleanup
    // -------------------------------------------------------------------------

    public void close() {
        doubleSubs.values().forEach(DoubleSubscriber::close);
        boolSubs.values().forEach(BooleanSubscriber::close);
        stringSubs.values().forEach(StringSubscriber::close);
        intSubs.values().forEach(IntegerSubscriber::close);
        doubleArraySubs.values().forEach(DoubleArraySubscriber::close);
    }
}