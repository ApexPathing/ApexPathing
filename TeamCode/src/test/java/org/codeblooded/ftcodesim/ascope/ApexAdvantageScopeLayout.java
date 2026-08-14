package org.codeblooded.ftcodesim.ascope;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;

import org.codeblooded.ftcodesim.physics.SeasonField;

/** Adds Apex's active path to FTCodeSim's dedicated AdvantageScope field tabs. */
public final class ApexAdvantageScopeLayout {
    private static final String TWO_D_TITLE = "2D FTCodeSim";
    private static final String THREE_D_TITLE = "3D FTCodeSim";
    private static final String PATH_LOG_KEY =
            "RealOutputs/Apex/CurrentPath";
    private static final String ROBOT_LOG_KEY =
            "RealOutputs/Drivetrain/position ftc coords (m)";

    private ApexAdvantageScopeLayout() { }

    /** Updates the layout created by FTCodeSim. Has no effect when AdvantageScope is unavailable. */
    public static void install() {
        AscopeViewEditor editor = new AscopeViewEditor(SeasonField.DECODE);
        if (configure(editor.root, editor.mapper)) {
            editor.save();
        }
    }

    static boolean configure(ObjectNode root, ObjectMapper mapper) {
        JsonNode tabsNode = root.path("hubs").path(0).path("state").path("tabs").path("tabs");
        if (!(tabsNode instanceof ArrayNode)) { return false; }

        ArrayNode tabs = (ArrayNode) tabsNode;
        ObjectNode threeDimensionalTab = findTab(tabs, 3, THREE_D_TITLE);
        if (threeDimensionalTab == null) { return false; }

        boolean changed = putPathBehindRobot(threeDimensionalTab, mapper);
        String game = threeDimensionalTab.path("controller").path("game").asText(
                "FTC:2025-2026 Field");

        ObjectNode twoDimensionalTab = findTab(tabs, 2, TWO_D_TITLE);
        if (twoDimensionalTab == null) {
            twoDimensionalTab = createTwoDimensionalTab(mapper, game);
            tabs.add(twoDimensionalTab);
            changed = true;
        }
        changed |= addRobotSource(twoDimensionalTab, mapper);
        changed |= putPathBehindRobot(twoDimensionalTab, mapper);

        // Normalize any older Apex path source and keep it ordered beneath the live robot.
        for (JsonNode tab : tabs) {
            if (tab instanceof ObjectNode && (tab.path("type").asInt(-1) == 2 ||
                    tab.path("type").asInt(-1) == 3) &&
                    sourceIndexByLogKey(sources((ObjectNode) tab, mapper), PATH_LOG_KEY) >= 0) {
                changed |= putPathBehindRobot((ObjectNode) tab, mapper);
            }
        }
        return changed;
    }

    private static ObjectNode findTab(ArrayNode tabs, int type, String title) {
        for (JsonNode tab : tabs) {
            if (tab instanceof ObjectNode &&
                    tab.path("type").asInt(-1) == type &&
                    title.equals(tab.path("title").asText())) {
                return (ObjectNode) tab;
            }
        }
        return null;
    }

    private static ObjectNode createTwoDimensionalTab(ObjectMapper mapper, String game) {
        ObjectNode controller = mapper.createObjectNode();
        controller.set("sources", mapper.createArrayNode());
        controller.put("field", game);
        controller.put("orientation", 0);
        controller.put("size", "large");

        ObjectNode tab = mapper.createObjectNode();
        tab.put("type", 2);
        tab.put("title", TWO_D_TITLE);
        tab.set("controller", controller);
        tab.put("controllerUUID", "apexpathing2dftcodesim0000000000");
        tab.putNull("renderer");
        tab.put("controlsHeight", 200);
        return tab;
    }

    private static boolean addRobotSource(ObjectNode tab, ObjectMapper mapper) {
        ArrayNode sources = sources(tab, mapper);
        if (containsSource(sources, "robot", ROBOT_LOG_KEY)) { return false; }

        ObjectNode options = mapper.createObjectNode();
        options.put("bumpers", "");
        sources.add(source(mapper, "robot", ROBOT_LOG_KEY, "Pose2d", options));
        return true;
    }

    /** Earlier sources draw over later ones, so keep sampled robot boxes below the live robot. */
    private static boolean putPathBehindRobot(ObjectNode tab, ObjectMapper mapper) {
        ArrayNode sources = sources(tab, mapper);
        int existingIndex = sourceIndexByLogKey(sources, PATH_LOG_KEY);
        ObjectNode pathSource;
        boolean changed = false;
        if (existingIndex >= 0) {
            pathSource = (ObjectNode) sources.remove(existingIndex);
            changed = existingIndex != sources.size();
        } else {
            ObjectNode options = mapper.createObjectNode();
            options.put("color", "#00ff00");
            options.put("model", "CodeBloodedDecode");
            pathSource = source(mapper, "ghost", PATH_LOG_KEY, "Pose2d[]", options);
            changed = true;
        }
        if (!"ghost".equals(pathSource.path("type").asText())) {
            pathSource.put("type", "ghost");
            changed = true;
        }
        if (!"Pose2d[]".equals(pathSource.path("logType").asText())) {
            pathSource.put("logType", "Pose2d[]");
            changed = true;
        }
        ObjectNode options = pathSource.with("options");
        if (!"#00ff00".equals(options.path("color").asText())) {
            options.put("color", "#00ff00");
            changed = true;
        }
        if (!"CodeBloodedDecode".equals(options.path("model").asText())) {
            options.put("model", "CodeBloodedDecode");
            changed = true;
        }
        if (options.remove("size") != null) { changed = true; }
        sources.add(pathSource);
        return changed;
    }

    private static int sourceIndexByLogKey(ArrayNode sources, String logKey) {
        for (int i = 0; i < sources.size(); i++) {
            if (logKey.equals(sources.get(i).path("logKey").asText())) { return i; }
        }
        return -1;
    }

    private static int sourceIndex(ArrayNode sources, String type, String logKey) {
        for (int i = 0; i < sources.size(); i++) {
            JsonNode source = sources.get(i);
            if (type.equals(source.path("type").asText()) &&
                    logKey.equals(source.path("logKey").asText())) {
                return i;
            }
        }
        return -1;
    }

    private static ArrayNode sources(ObjectNode tab, ObjectMapper mapper) {
        JsonNode controllerNode = tab.path("controller");
        ObjectNode controller;
        if (controllerNode instanceof ObjectNode) {
            controller = (ObjectNode) controllerNode;
        } else {
            controller = mapper.createObjectNode();
            tab.set("controller", controller);
        }

        JsonNode sourcesNode = controller.path("sources");
        if (sourcesNode instanceof ArrayNode) { return (ArrayNode) sourcesNode; }

        ArrayNode sources = mapper.createArrayNode();
        controller.set("sources", sources);
        return sources;
    }

    private static boolean containsSource(ArrayNode sources, String type, String logKey) {
        return sourceIndex(sources, type, logKey) >= 0;
    }

    private static ObjectNode source(ObjectMapper mapper, String type, String logKey,
                                     String logType, ObjectNode options) {
        ObjectNode source = mapper.createObjectNode();
        source.put("type", type);
        source.put("logKey", logKey);
        source.put("logType", logType);
        source.put("visible", true);
        source.set("options", options);
        return source;
    }
}
