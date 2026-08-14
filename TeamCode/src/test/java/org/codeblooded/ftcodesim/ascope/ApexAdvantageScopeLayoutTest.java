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
        assertSource(tabs.get(0), "ghost", "Pose2d[]",
                "RealOutputs/Apex/CurrentPath");
        assertSource(tabs.get(1), "robot", "Pose2d",
                "RealOutputs/Drivetrain/position ftc coords (m)");
        assertSource(tabs.get(1), "ghost", "Pose2d[]",
                "RealOutputs/Apex/CurrentPath");
        assertEquals("robot", tabs.get(1).path("controller").path("sources")
                .path(0).path("type").asText());
        assertEquals("ghost", tabs.get(1).path("controller").path("sources")
                .path(1).path("type").asText());
        assertEquals("CodeBloodedDecode", tabs.get(1).path("controller").path("sources")
                .path(1).path("options").path("model").asText());
    }

    @Test
    public void movesAnExistingPathBelowTheRobot() {
        ObjectMapper mapper = new ObjectMapper();
        ObjectNode root = layoutWithThreeDimensionalTab(mapper);
        ObjectNode tab = (ObjectNode) root.path("hubs").path(0).path("state")
                .path("tabs").path("tabs").path(0);
        ArrayNode sources = (ArrayNode) tab.path("controller").path("sources");
        sources.add(source(mapper, "robot", "robot-key", "Pose2d"));
        sources.add(source(mapper, "trajectory", "RealOutputs/Apex/CurrentPath", "Pose2d[]"));

        assertTrue(ApexAdvantageScopeLayout.configure(root, mapper));
        assertEquals("robot", sources.path(0).path("type").asText());
        assertEquals("ghost", sources.path(1).path("type").asText());
        assertEquals("#00ff00", sources.path(1).path("options").path("color").asText());
    }

    @Test
    public void preservesGhostBoxesInExistingFieldTabs() {
        ObjectMapper mapper = new ObjectMapper();
        ObjectNode root = layoutWithThreeDimensionalTab(mapper);
        ArrayNode tabs = (ArrayNode) root.path("hubs").path(0).path("state")
                .path("tabs").path("tabs");
        ObjectNode controller = mapper.createObjectNode();
        ArrayNode sources = mapper.createArrayNode();
        sources.add(source(mapper, "robot", "robot-key", "Pose2d"));
        sources.add(source(mapper, "ghost", "RealOutputs/Apex/CurrentPath", "Pose2d[]"));
        controller.set("sources", sources);
        ObjectNode existingField = mapper.createObjectNode();
        existingField.put("type", 2);
        existingField.put("title", "2D Field");
        existingField.set("controller", controller);
        tabs.add(existingField);

        assertTrue(ApexAdvantageScopeLayout.configure(root, mapper));
        assertEquals("ghost", sources.path(1).path("type").asText());
        assertEquals("CodeBloodedDecode", sources.path(1).path("options").path("model").asText());
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

    private static ObjectNode source(ObjectMapper mapper, String type, String logKey,
                                     String logType) {
        ObjectNode source = mapper.createObjectNode();
        source.put("type", type);
        source.put("logKey", logKey);
        source.put("logType", logType);
        source.set("options", mapper.createObjectNode());
        return source;
    }
}
