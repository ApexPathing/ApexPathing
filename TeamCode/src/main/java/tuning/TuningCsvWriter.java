package tuning;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

import core.ApexStorage;

/** Small failure-tolerant CSV writer for tuner evidence captured on robot or desktop. */
final class TuningCsvWriter {
    private final File file;
    private final FileWriter writer;
    private String error;
    private int rowsSinceFlush;

    static TuningCsvWriter open(String prefix, String... header) {
        try {
            File directory = ApexStorage.getDirectory();
            if (!directory.exists() && !directory.mkdirs()) {
                throw new IOException("Could not create " + directory.getAbsolutePath());
            }
            String timestamp = new SimpleDateFormat(
                    "yyyyMMdd_HHmmss_SSS", Locale.US).format(new Date());
            File file = new File(directory, prefix + "_" + timestamp + ".csv");
            TuningCsvWriter result = new TuningCsvWriter(file, new FileWriter(file));
            result.writeRow((Object[]) header);
            return result;
        } catch (IOException e) {
            return new TuningCsvWriter(e.getMessage());
        }
    }

    private TuningCsvWriter(File file, FileWriter writer) {
        this.file = file;
        this.writer = writer;
    }

    private TuningCsvWriter(String error) {
        this.file = null;
        this.writer = null;
        this.error = error;
    }

    void writeRow(Object... values) {
        if (writer == null) { return; }
        try {
            for (int i = 0; i < values.length; i++) {
                if (i > 0) { writer.write(','); }
                writer.write(escape(values[i]));
            }
            writer.write('\n');
            // Periodic flushing bounds data loss if Stop is pressed without adding filesystem
            // latency to every 50 Hz controller iteration.
            rowsSinceFlush++;
            if (rowsSinceFlush >= 25) {
                writer.flush();
                rowsSinceFlush = 0;
            }
        } catch (IOException e) {
            error = e.getMessage();
        }
    }

    void close() {
        if (writer == null) { return; }
        try {
            writer.close();
        } catch (IOException e) {
            error = e.getMessage();
        }
    }

    String getPath() { return file == null ? "Unavailable" : file.getAbsolutePath(); }

    String getError() { return error; }

    private static String escape(Object value) {
        String text = value == null ? "" : String.valueOf(value);
        if (text.indexOf(',') < 0 && text.indexOf('"') < 0 &&
                text.indexOf('\n') < 0 && text.indexOf('\r') < 0) {
            return text;
        }
        return '"' + text.replace("\"", "\"\"") + '"';
    }
}
