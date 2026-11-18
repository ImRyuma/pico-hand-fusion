package com.ryuma.handfusion;

import android.app.Activity;
import android.graphics.Typeface;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.text.InputType;
import android.view.Gravity;
import android.view.View;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.ScrollView;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.Toast;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/** Settings and live status for the Hand Fusion layer running inside PICO Connect. */
public final class MainActivity extends Activity {
    private static final String HOST = "com.picovr.picostreamassistant";
    private static final String DIR = "/data/data/" + HOST + "/files";
    private static final String CONF = DIR + "/hand_fusion.conf";
    private static final String STATUS = DIR + "/hand_fusion_status.txt";

    private final ExecutorService io = Executors.newSingleThreadExecutor();
    private final Handler ui = new Handler(Looper.getMainLooper());
    /** Last config read from or written to the file; only touched on the io thread. */
    private final Map<String, String> conf = new LinkedHashMap<>();

    private CheckBox enabled;
    private CheckBox fullFrequency;
    private CheckBox lagAuto;
    private SeekBar weight;
    private TextView weightLabel;
    private SeekBar smoothing;
    private TextView smoothingLabel;
    private EditText leftTracker;
    private EditText rightTracker;
    private final List<NumberSetting> numbers = new ArrayList<>();
    private TextView status;
    private boolean visible;

    /** A numeric key of hand_fusion.conf edited in a text field. */
    private static final class NumberSetting {
        final String key;
        final String def;
        final boolean decimal;
        EditText field;

        NumberSetting(String key, String def, boolean decimal) {
            this.key = key;
            this.def = def;
            this.decimal = decimal;
        }
    }

    private final Runnable refresh = new Runnable() {
        @Override
        public void run() {
            if (!visible) return;
            io.execute(() -> {
                Root.Result r = Root.run("cat " + STATUS + " 2>/dev/null", null);
                String text = r.ok() && !r.out.isEmpty() ? formatStatus(r.out)
                        : "No status yet. Start a PICO Connect session with hand tracking.";
                ui.post(() -> status.setText(text));
            });
            ui.postDelayed(this, 1000);
        }
    };

    @Override
    protected void onCreate(Bundle state) {
        super.onCreate(state);
        setContentView(buildUi());
        io.execute(this::loadConfig);
    }

    @Override
    protected void onResume() {
        super.onResume();
        visible = true;
        ui.post(refresh);
    }

    @Override
    protected void onPause() {
        visible = false;
        super.onPause();
    }

    // ------------------------------------------------------------------ UI

    private View buildUi() {
        int pad = dp(16);
        LinearLayout root = new LinearLayout(this);
        root.setOrientation(LinearLayout.VERTICAL);
        root.setPadding(pad, pad, pad, pad);

        TextView title = new TextView(this);
        title.setText("Hand Fusion");
        title.setTextSize(24);
        title.setTypeface(Typeface.DEFAULT_BOLD);
        root.addView(title);

        TextView hint = new TextView(this);
        hint.setText("Motion Trackers on the back of the hands + camera hand tracking, applied inside PICO Connect. "
                + "Put the hand trackers in Object tracking mode in the Motion Tracker app.");
        hint.setPadding(0, dp(4), 0, dp(12));
        root.addView(hint);

        enabled = new CheckBox(this);
        enabled.setText("Fusion enabled");
        root.addView(enabled);

        fullFrequency = new CheckBox(this);
        fullFrequency.setText("Keep trackers at full frequency during sessions");
        root.addView(fullFrequency);

        weightLabel = new TextView(this);
        root.addView(weightLabel);
        weight = slider(root, this::showWeight);

        smoothingLabel = new TextView(this);
        root.addView(smoothingLabel);
        smoothing = slider(root, this::showSmoothing);

        number(root, "hold_timeout_ms", "0", false, "Keep a hand lost by the cameras for (ms, 0 = forever)");
        leftTracker = field(root, "Left tracker id (empty = automatic)", false);
        rightTracker = field(root, "Right tracker id (empty = automatic)", false);

        section(root, "Fingers out of view");
        number(root, "finger_relax_delay_ms", "300", false, "Relax the fingers after (ms)");
        number(root, "finger_relax_ms", "1500", false,
                "Relaxing time toward the usual pose (ms, 0 = keep the last pose)");
        number(root, "reacquire_ms", "150", false, "Blend back to the cameras over (ms)");

        section(root, "Camera latency");
        lagAuto = new CheckBox(this);
        lagAuto.setText("Measure it automatically during fast moves");
        root.addView(lagAuto);
        number(root, "camera_lag_ms", "45", false, "Latency (ms): starting value, or fixed when not measured");

        section(root, "Cameras' field of view");
        number(root, "fov_side_deg", "65", true, "Left / right (deg): beyond this, camera data is ignored");
        number(root, "fov_up_deg", "45", true, "Up (deg)");
        number(root, "fov_down_deg", "65", true, "Down (deg)");

        section(root, "Advanced");
        number(root, "rotation_weight", "1.0", true, "Tracker weight for the orientation (0 to 1)");
        number(root, "outlier_dist", "0.08", true, "Ignore camera frames farther than this from the tracker (m)");
        number(root, "assign_max_dist", "0.12", true, "Max tracker to palm distance for assignment (m)");
        number(root, "min_samples", "120", false, "Frames needed to calibrate a hand");
        number(root, "learn_rate", "0.01", true, "Calibration refinement rate");

        LinearLayout buttons = row(root);
        buttons.addView(button("Save", v -> save()));
        buttons.addView(button("Recalibrate", v -> io.execute(this::recalibrate)));
        buttons.addView(button("Defaults", v -> resetAdvanced()));
        LinearLayout buttons2 = row(root);
        buttons2.addView(button("Record 60 s", v -> io.execute(this::record)));
        buttons2.addView(button("Restart PICO Connect", v -> io.execute(this::restartHost)));

        section(root, "Live status");
        status = new TextView(this);
        status.setTypeface(Typeface.MONOSPACE);
        status.setText("…");
        root.addView(status);

        ScrollView scroll = new ScrollView(this);
        scroll.addView(root);
        return scroll;
    }

    private interface IntConsumer {
        void accept(int v);
    }

    private void showWeight(int v) {
        weightLabel.setText("Tracker weight when the cameras see the hand: " + v + " %");
    }

    private void showSmoothing(int v) {
        smoothingLabel.setText("Tremor smoothing strength: " + v + " %");
    }

    private SeekBar slider(LinearLayout parent, IntConsumer onChange) {
        SeekBar s = new SeekBar(this);
        s.setMax(100);
        s.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar b, int v, boolean fromUser) {
                onChange.accept(v);
            }

            @Override
            public void onStartTrackingTouch(SeekBar b) {}

            @Override
            public void onStopTrackingTouch(SeekBar b) {}
        });
        parent.addView(s);
        return s;
    }

    private void section(LinearLayout parent, String text) {
        TextView t = new TextView(this);
        t.setText(text);
        t.setTypeface(Typeface.DEFAULT_BOLD);
        t.setTextSize(17);
        t.setPadding(0, dp(16), 0, dp(2));
        parent.addView(t);
    }

    private void number(LinearLayout parent, String key, String def, boolean decimal, String label) {
        NumberSetting n = new NumberSetting(key, def, decimal);
        n.field = field(parent, label, decimal);
        numbers.add(n);
    }

    private EditText field(LinearLayout parent, String label, boolean decimal) {
        TextView l = new TextView(this);
        l.setText(label);
        l.setPadding(0, dp(8), 0, 0);
        parent.addView(l);
        EditText e = new EditText(this);
        e.setInputType(InputType.TYPE_CLASS_NUMBER | (decimal ? InputType.TYPE_NUMBER_FLAG_DECIMAL : 0));
        e.setSingleLine(true);
        parent.addView(e);
        return e;
    }

    private LinearLayout row(LinearLayout parent) {
        LinearLayout r = new LinearLayout(this);
        r.setOrientation(LinearLayout.HORIZONTAL);
        r.setPadding(0, dp(8), 0, 0);
        parent.addView(r);
        return r;
    }

    private Button button(String text, View.OnClickListener l) {
        Button b = new Button(this);
        b.setText(text);
        b.setOnClickListener(l);
        b.setGravity(Gravity.CENTER);
        return b;
    }

    private int dp(int v) {
        return (int) (v * getResources().getDisplayMetrics().density + 0.5f);
    }

    private void toast(String msg) {
        ui.post(() -> Toast.makeText(this, msg, Toast.LENGTH_SHORT).show());
    }

    // -------------------------------------------------------------- config

    private void loadConfig() {
        conf.clear();
        Root.Result r = Root.run("cat " + CONF + " 2>/dev/null", null);
        if (r.code == -1) {
            toast("Root access denied: grant su to Hand Fusion in Magisk");
        }
        for (String line : r.out.split("\n")) {
            line = line.trim();
            int eq = line.indexOf('=');
            if (line.isEmpty() || line.startsWith("#") || eq <= 0) continue;
            conf.put(line.substring(0, eq).trim(), line.substring(eq + 1).trim());
        }
        final Map<String, String> c = new LinkedHashMap<>(conf);
        ui.post(() -> {
            enabled.setChecked(!"0".equals(c.get("enabled")));
            fullFrequency.setChecked(!"0".equals(c.get("full_frequency")));
            lagAuto.setChecked(!"0".equals(c.get("camera_lag_auto")));
            int w = Math.round(parseFloat(c.get("tracked_weight"), 1.0f) * 100f);
            weight.setProgress(w);
            showWeight(w);
            int s = smoothingFromCutoff(parseFloat(c.get("finger_min_cutoff"), 1.5f));
            smoothing.setProgress(s);
            showSmoothing(s);
            leftTracker.setText(trackerText(c.get("left_tracker")));
            rightTracker.setText(trackerText(c.get("right_tracker")));
            for (NumberSetting n : numbers) n.field.setText(orDefault(c.get(n.key), n.def));
        });
    }

    /** Reads the form on the UI thread, then writes the file on the io thread. */
    private void save() {
        final Map<String, String> values = new LinkedHashMap<>();
        values.put("enabled", enabled.isChecked() ? "1" : "0");
        values.put("full_frequency", fullFrequency.isChecked() ? "1" : "0");
        values.put("camera_lag_auto", lagAuto.isChecked() ? "1" : "0");
        values.put("tracked_weight", String.valueOf(weight.getProgress() / 100f));
        // One slider drives both filters: stronger smoothing = lower cutoff frequency.
        float s = smoothing.getProgress() / 100f;
        values.put("finger_min_cutoff", String.valueOf(4.0f - 3.5f * s));
        values.put("wrist_min_cutoff", String.valueOf(8.0f - 6.0f * s));
        values.put("left_tracker", trackerValue(leftTracker.getText().toString()));
        values.put("right_tracker", trackerValue(rightTracker.getText().toString()));
        for (NumberSetting n : numbers) {
            String v = n.field.getText().toString().trim();
            if (!isNumber(v, n.decimal)) {
                toast("Invalid value for " + n.key + ": " + (v.isEmpty() ? "(empty)" : v));
                return;
            }
            values.put(n.key, v);
        }
        io.execute(() -> {
            conf.putAll(values);
            toast(writeConfig() ? "Saved (applied within a second)" : "Could not write the settings");
        });
    }

    private void resetAdvanced() {
        for (NumberSetting n : numbers) {
            if (!n.key.equals("hold_timeout_ms")) n.field.setText(n.def);
        }
        lagAuto.setChecked(true);
        toast("Defaults restored in the form: press Save to apply");
    }

    private void recalibrate() {
        long epoch = parseLong(conf.get("calib_epoch"), 0) + 1;
        conf.put("calib_epoch", String.valueOf(epoch));
        // Also re-arm tracker calls the layer disabled after a crash.
        Root.run("rm -f " + DIR + "/hand_fusion_guard_*", null);
        toast(writeConfig() ? "Calibration reset: keep your hands in view for a few seconds"
                : "Could not write the settings");
    }

    private void record() {
        long epoch = parseLong(conf.get("record_epoch"), 0) + 1;
        conf.put("record_epoch", String.valueOf(epoch));
        String seconds = orDefault(conf.get("record_seconds"), "60");
        toast(writeConfig() ? "Recording " + seconds + " s to " + DIR + "/hand_fusion_record.csv"
                : "Could not write the settings");
    }

    private void restartHost() {
        Root.Result r = Root.run("am force-stop " + HOST, null);
        toast(r.ok() ? "PICO Connect stopped: open it again to load the layer" : "Failed: " + r.out);
    }

    /** Writes the file as the host app's owner and SELinux label so the layer can read it. */
    private boolean writeConfig() {
        StringBuilder sb = new StringBuilder("# Written by the Hand Fusion companion app\n");
        for (Map.Entry<String, String> e : conf.entrySet()) {
            sb.append(e.getKey()).append('=').append(e.getValue()).append('\n');
        }
        String script = "set -e; mkdir -p " + DIR + "; "
                + "owner=$(stat -c %u:%g /data/data/" + HOST + "); "
                + "cat > " + CONF + ".tmp; "
                + "chown $owner " + DIR + " " + CONF + ".tmp; chmod 0600 " + CONF + ".tmp; "
                + "mv " + CONF + ".tmp " + CONF + "; restorecon " + DIR + " " + CONF;
        return Root.run(script, sb.toString()).ok();
    }

    // ------------------------------------------------------------- helpers

    private static String formatStatus(String raw) {
        StringBuilder sb = new StringBuilder();
        for (String line : raw.split("\n")) {
            int eq = line.indexOf('=');
            if (eq <= 0 || line.startsWith("updated_ns")) continue;
            String key = line.substring(0, eq);
            String value = line.substring(eq + 1);
            if (key.equals("camera_lag_ms") && value.startsWith("-")) value = "not measured yet";
            sb.append(String.format("%-26s %s%n", key, value));
        }
        return sb.toString();
    }

    private static boolean isNumber(String v, boolean decimal) {
        if (v.isEmpty()) return false;
        try {
            if (decimal) Double.parseDouble(v);
            else Long.parseLong(v);
            return true;
        } catch (NumberFormatException e) {
            return false;
        }
    }

    private static int smoothingFromCutoff(float fingerCutoff) {
        int s = Math.round((4.0f - fingerCutoff) / 3.5f * 100f);
        return Math.max(0, Math.min(100, s));
    }

    private static String trackerText(String v) {
        return v == null || v.startsWith("-") ? "" : v;
    }

    private static String trackerValue(String v) {
        v = v.trim();
        return v.isEmpty() ? "-1" : v;
    }

    private static String orDefault(String v, String d) {
        return v == null || v.isEmpty() ? d : v;
    }

    private static float parseFloat(String v, float d) {
        try {
            return v == null ? d : Float.parseFloat(v);
        } catch (NumberFormatException e) {
            return d;
        }
    }

    private static long parseLong(String v, long d) {
        try {
            return v == null ? d : Long.parseLong(v);
        } catch (NumberFormatException e) {
            return d;
        }
    }
}
