package org.codeblooded.ftcodesim.ascope;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;

import org.junit.Test;

public class ApexAdvantageScopeLayoutTest {
    @Test
    public void addsCurrentPathToDedicatedTwoAndThreeDimensionalTabsOnce() {
        ObjectMapper mapper = new ObjectMapper();
        ObjectNode root = layoutWithThreeDimensionalTab(mapper);

        assertTrue(ApexAdvantageScopeLayout.configure(root, mapper));
        assertFalse(ApexAdvantageScopeLayout.configure(root, mapper));

        ArrayNode tabs = (ArrayNode) root.path("hubs").path(0).path("state")
                .path("tabs").path("tabs");
        assertEquals(2, tabs.size());
        assertSource(tabs.get(0), "trajectory", "Pose2d[]",
                "RealOutputs/Apex/CurrentPath");
        assertSource(tabs.get(1), "robot", "Pose2d",
                "RealOutputs/Drivetrain/position ftc coords (m)");
        assertSource(tabs.get(1), "trajectory", "Pose2d[]",
                "RealOutputs/Apex/CurrentPath");
    }

    private static ObjectNode layoutWithThreeDimensionalTab(ObjectMapper mapper) {
        ObjectNode controller = mapper.createObjectNode();
        controller.put("game", "FTC:2025-2026 Field");
        controller.set("sources", mapper.createArrayNode());

        ObjectNode tab = mapper.createObjectNode();
        tab.put("type", 3);
        tab.put("title", "3D FTCodeSim");
        tab.set("controller", controller);

        ArrayNode tabs = mapper.createArrayNode();
        tabs.add(tab);
        ObjectNode tabsState = mapper.createObjectNode();
        tabsState.set("tabs", tabs);
        ObjectNode state = mapper.createObjectNode();
        state.set("tabs", tabsState);
        ObjectNode hub = mapper.createObjectNode();
        hub.set("state", state);
        ArrayNode hubs = mapper.createArrayNode();
        hubs.add(hub);
        ObjectNode root = mapper.createObjectNode();
        root.set("hubs", hubs);
        return root;
    }

    private static void assertSource(JsonNode tab, String type, String logType, String logKey) {
        for (JsonNode source : tab.path("controller").path("sources")) {
            if (type.equals(source.path("type").asText()) &&
                    logKey.equals(source.path("logKey").asText())) {
                assertEquals(logType, source.path("logType").asText());
                assertTrue(source.path("visible").asBoolean());
                return;
            }
        }
        throw new AssertionError("Missing " + type + " source " + logKey);
    }
}
