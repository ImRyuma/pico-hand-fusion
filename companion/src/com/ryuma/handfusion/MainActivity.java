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

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/** Settings and live status for the Hand Fusion OpenXR layer running inside PICO Connect. */
public final class MainActivity extends Activity {
    private static final String HOST = "com.picovr.picostreamassistant";
    private static final String DIR = "/data/data/" + HOST + "/files";
    private static final String CONF = DIR + "/hand_fusion.conf";
    private static final String STATUS = DIR + "/hand_fusion_status.txt";

    private final ExecutorService io = Executors.newSingleThreadExecutor();
    private final Handler ui = new Handler(Looper.getMainLooper());
    private final Map<String, String> conf = new LinkedHashMap<>();

    private CheckBox enabled;
    private CheckBox fullFrequency;
    private SeekBar weight;
    private TextView weightLabel;
    private SeekBar smoothing;
    private TextView smoothingLabel;
    private EditText hold;
    private EditText leftTracker;
    private EditText rightTracker;
    private TextView status;
    private boolean visible;

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
        weight = new SeekBar(this);
        weight.setMax(100);
        weight.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar s, int v, boolean fromUser) {
                weightLabel.setText("Tracker weight when the cameras see the hand: " + v + " %");
            }

            @Override
            public void onStartTrackingTouch(SeekBar s) {}

            @Override
            public void onStopTrackingTouch(SeekBar s) {}
        });
        root.addView(weight);

        smoothingLabel = new TextView(this);
        root.addView(smoothingLabel);
        smoothing = new SeekBar(this);
        smoothing.setMax(100);
        smoothing.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar s, int v, boolean fromUser) {
                smoothingLabel.setText("Tremor smoothing strength: " + v + " %");
            }

            @Override
            public void onStartTrackingTouch(SeekBar s) {}

            @Override
            public void onStopTrackingTouch(SeekBar s) {}
        });
        root.addView(smoothing);

        hold = numberField(root, "Keep a hand lost by the cameras for (ms, 0 = forever)");
        leftTracker = numberField(root, "Left tracker id (empty = automatic)");
        rightTracker = numberField(root, "Right tracker id (empty = automatic)");

        LinearLayout buttons = new LinearLayout(this);
        buttons.setOrientation(LinearLayout.HORIZONTAL);
        buttons.setPadding(0, dp(12), 0, dp(12));
        buttons.addView(button("Save", v -> io.execute(this::saveConfig)));
        buttons.addView(button("Recalibrate", v -> io.execute(this::recalibrate)));
        buttons.addView(button("Restart PICO Connect", v -> io.execute(this::restartHost)));
        root.addView(buttons);

        TextView statusTitle = new TextView(this);
        statusTitle.setText("Live status");
        statusTitle.setTypeface(Typeface.DEFAULT_BOLD);
        root.addView(statusTitle);

        status = new TextView(this);
        status.setTypeface(Typeface.MONOSPACE);
        status.setText("…");
        root.addView(status);

        ScrollView scroll = new ScrollView(this);
        scroll.addView(root);
        return scroll;
    }

    private EditText numberField(LinearLayout parent, String label) {
        TextView l = new TextView(this);
        l.setText(label);
        l.setPadding(0, dp(8), 0, 0);
        parent.addView(l);
        EditText e = new EditText(this);
        e.setInputType(InputType.TYPE_CLASS_NUMBER);
        e.setSingleLine(true);
        parent.addView(e);
        return e;
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
        ui.post(() -> {
            enabled.setChecked(!"0".equals(conf.get("enabled")));
            fullFrequency.setChecked(!"0".equals(conf.get("full_frequency")));
            int w = Math.round(parseFloat(conf.get("tracked_weight"), 0.9f) * 100f);
            weight.setProgress(w);
            weightLabel.setText("Tracker weight when the cameras see the hand: " + w + " %");
            int s = smoothingFromCutoff(parseFloat(conf.get("finger_min_cutoff"), 1.5f));
            smoothing.setProgress(s);
            smoothingLabel.setText("Tremor smoothing strength: " + s + " %");
            hold.setText(orDefault(conf.get("hold_timeout_ms"), "0"));
            leftTracker.setText(trackerText(conf.get("left_tracker")));
            rightTracker.setText(trackerText(conf.get("right_tracker")));
        });
    }

    private void saveConfig() {
        final String[] values = new String[5];
        final boolean[] on = new boolean[2];
        final int[] smooth = new int[1];
        final Object lock = new Object();
        ui.post(() -> {
            synchronized (lock) {
                on[0] = enabled.isChecked();
                on[1] = fullFrequency.isChecked();
                smooth[0] = smoothing.getProgress();
                values[0] = String.valueOf(weight.getProgress() / 100f);
                values[1] = orDefault(hold.getText().toString().trim(), "0");
                values[2] = trackerValue(leftTracker.getText().toString());
                values[3] = trackerValue(rightTracker.getText().toString());
                values[4] = "ready";
                lock.notifyAll();
            }
        });
        synchronized (lock) {
            while (values[4] == null) {
                try {
                    lock.wait();
                } catch (InterruptedException e) {
                    return;
                }
            }
        }
        conf.put("enabled", on[0] ? "1" : "0");
        conf.put("full_frequency", on[1] ? "1" : "0");
        // One slider drives both filters: stronger smoothing = lower cutoff frequency.
        float s = smooth[0] / 100f;
        conf.put("finger_min_cutoff", String.valueOf(4.0f - 3.5f * s));
        conf.put("wrist_min_cutoff", String.valueOf(8.0f - 6.0f * s));
        conf.put("tracked_weight", values[0]);
        conf.put("hold_timeout_ms", values[1]);
        conf.put("left_tracker", values[2]);
        conf.put("right_tracker", values[3]);
        toast(writeConfig() ? "Saved (applied within a second)" : "Could not write the settings");
    }

    private void recalibrate() {
        long epoch = parseLong(conf.get("calib_epoch"), 0) + 1;
        conf.put("calib_epoch", String.valueOf(epoch));
        // Also re-arm tracker calls the layer disabled after a crash.
        Root.run("rm -f " + DIR + "/hand_fusion_guard_*", null);
        toast(writeConfig() ? "Calibration reset: keep your hands in view for a few seconds"
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
            sb.append(String.format("%-26s %s%n", line.substring(0, eq), line.substring(eq + 1)));
        }
        return sb.toString();
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
