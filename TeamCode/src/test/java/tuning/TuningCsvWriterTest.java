package tuning;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.List;

import core.ApexStorage;

public class TuningCsvWriterTest {
    @Rule
    public TemporaryFolder temporaryFolder = new TemporaryFolder();

    @Test
    public void writesGraphReadyRowsToConfiguredStorage() throws Exception {
        String oldDirectory = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        try {
            System.setProperty(ApexStorage.DIRECTORY_PROPERTY,
                    temporaryFolder.getRoot().getAbsolutePath());
            TuningCsvWriter writer = TuningCsvWriter.open(
                    "test_capture", "time_s", "position", "label");
            writer.writeRow(0.02, 1.5, "forward, run");
            writer.close();

            assertNull(writer.getError());
            assertTrue(writer.getPath().endsWith(".csv"));
            List<String> lines = Files.readAllLines(
                    Paths.get(writer.getPath()), StandardCharsets.UTF_8);
            assertEquals("time_s,position,label", lines.get(0));
            assertEquals("0.02,1.5,\"forward, run\"", lines.get(1));
        } finally {
            if (oldDirectory == null) {
                System.clearProperty(ApexStorage.DIRECTORY_PROPERTY);
            } else {
                System.setProperty(ApexStorage.DIRECTORY_PROPERTY, oldDirectory);
            }
        }
    }
}
