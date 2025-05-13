#pragma once
#include <vector>
#include <string>
#include <optional>

// Enhanced struct to hold comprehensive visual properties for a body
struct BodyVisualProperties {
    // Body identifier
    int bodyIndex;

    // Basic visual properties
    float colorR = 0.5f, colorG = 0.5f, colorB = 0.5f;
    float transparency = 1.0f;
    bool smoothShade = false;
    float edgeWidth = 1.0f;

    // Texture properties
    bool showTextureSpace = false;
    std::string textureParamName = "uv_coords";
    std::optional<std::string> texturePath;

    // Material properties
    float shininess = 0.0f;
    float reflectivity = 0.0f;

    // Visualization options
    bool showWireframe = false;
    bool showNormals = false;
    float normalLength = 0.1f;
    bool showBoundingBox = false;
};

// Container to store visual properties for all bodies
class ScenarioVisualProperties {
public:
    std::vector<BodyVisualProperties> bodyProperties;

    void clear() {
        bodyProperties.clear();
    }

    // Enhanced method to add full visual properties
    void addBodyProperties(
        int bodyIndex,
        float r, float g, float b,
        float transparency,
        bool smoothShade,
        float edgeWidth,
        bool showTextureSpace = false,
        const std::string& textureParamName = "uv_coords",
        const std::optional<std::string>& texturePath = std::nullopt,
        float shininess = 0.0f,
        float reflectivity = 0.0f,
        bool showWireframe = false,
        bool showNormals = false,
        float normalLength = 0.1f,
        bool showBoundingBox = false,
        bool showContactPoints = false,
        bool showInertiaEllipsoid = false,
        float debugColorR = 1.0f,
        float debugColorG = 1.0f,
        float debugColorB = 0.0f
    ) {
        BodyVisualProperties props;
        props.bodyIndex = bodyIndex;

        // Basic properties
        props.colorR = r;
        props.colorG = g;
        props.colorB = b;
        props.transparency = transparency;
        props.smoothShade = smoothShade;
        props.edgeWidth = edgeWidth;

        // Texture properties
        props.showTextureSpace = showTextureSpace;
        props.textureParamName = textureParamName;
        props.texturePath = texturePath;

        // Material properties
        props.shininess = shininess;
        props.reflectivity = reflectivity;

        // Visualization options
        props.showWireframe = showWireframe;
        props.showNormals = showNormals;
        props.normalLength = normalLength;
        props.showBoundingBox = showBoundingBox;

        bodyProperties.push_back(props);
    }

    // Simplified method for backward compatibility
    void addBodyProperties(int bodyIndex, float r, float g, float b,
                          float transparency, bool smoothShade, float edgeWidth) {
        addBodyProperties(bodyIndex, r, g, b, transparency, smoothShade, edgeWidth,
                         false, "uv_coords", std::nullopt, 0.0f, 0.0f, false, false,
                         0.1f, false, false, false, 1.0f, 1.0f, 0.0f);
    }

    // Find properties for a body by index
    BodyVisualProperties* findPropertiesForBody(int bodyIndex) {
        for (auto& prop : bodyProperties) {
            if (prop.bodyIndex == bodyIndex) {
                return &prop;
            }
        }
        return nullptr;
    }
};

// Global instance of visual properties
extern ScenarioVisualProperties g_visualProperties;