package com.ryuma.handfusion;

import java.io.ByteArrayOutputStream;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.charset.StandardCharsets;

/** Runs shell commands through Magisk's su. */
final class Root {
    private Root() {}

    static final class Result {
        final int code;
        final String out;

        Result(int code, String out) {
            this.code = code;
            this.out = out;
        }

        boolean ok() {
            return code == 0;
        }
    }

    /** Runs {@code script} in a root shell, optionally feeding {@code stdin}. */
    static Result run(String script, String stdin) {
        try {
            Process p = new ProcessBuilder("su", "-c", script).redirectErrorStream(true).start();
            OutputStream os = p.getOutputStream();
            if (stdin != null) os.write(stdin.getBytes(StandardCharsets.UTF_8));
            os.close();
            InputStream is = p.getInputStream();
            ByteArrayOutputStream buf = new ByteArrayOutputStream();
            byte[] chunk = new byte[4096];
            int n;
            while ((n = is.read(chunk)) > 0) buf.write(chunk, 0, n);
            int code = p.waitFor();
            return new Result(code, buf.toString("UTF-8"));
        } catch (Exception e) {
            return new Result(-1, String.valueOf(e.getMessage()));
        }
    }
}
