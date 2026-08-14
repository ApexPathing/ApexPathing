package tuning;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

import java.nio.file.Files;
import java.nio.file.Paths;

import core.ApexStorage;

public class ManualResponseMetricsTest {
    @Rule
    public TemporaryFolder temporaryFolder = new TemporaryFolder();

    @Test
    public void recordsResponseQualityAndGraphReadyCsv() throws Exception {
        String oldDirectory = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        try {
            System.setProperty(ApexStorage.DIRECTORY_PROPERTY,
                    temporaryFolder.getRoot().getAbsolutePath());
            ManualResponseMetrics metrics = new ManualResponseMetrics();
            metrics.begin("manual_test", 0.0, 1.0, 0.05, 0.1);
            Thread.sleep(5);
            metrics.sample(0.5, 2.0, 1.0);
            Thread.sleep(5);
            metrics.sample(1.1, 0.5, 0.4);
            metrics.finish();

            assertEquals(0.1, metrics.getOvershoot(), 1e-9);
            assertEquals(-0.1, metrics.getFinalError(), 1e-9);
            assertEquals(2.0, metrics.getPeakVelocity(), 1e-9);
            assertEquals(0.5, metrics.getSaturationFraction(), 1e-9);
            assertTrue(metrics.getRmsError() > 0.0);
            assertTrue(metrics.getTimeWeightedSquaredError() > 0.0);
            assertTrue(Files.size(Paths.get(metrics.getCsvPath())) > 0L);
        } finally {
            if (oldDirectory == null) {
                System.clearProperty(ApexStorage.DIRECTORY_PROPERTY);
            } else {
                System.setProperty(ApexStorage.DIRECTORY_PROPERTY, oldDirectory);
            }
        }
    }
}
