#version 330 core

in vec3 FragPos;
in vec3 Normal;
in vec2 TexCoord;

uniform vec3 lightDir;        // Direction to sun (normalized)
uniform vec3 viewPos;         // Camera position
uniform vec3 objectColor;     // Color of the celestial body (used if no texture)
uniform sampler2D textureSampler;
uniform bool useTexture;      // Whether to use texture or solid color
uniform float ambientStrength; // Ambient light strength (adjustable per object)
uniform bool isEmissive;      // Whether object is self-illuminated (like stars or sun)

out vec4 FragColor;

void main()
{
    // Get base color from texture or solid color
    vec3 baseColor;
    if (useTexture)
    {
        baseColor = texture(textureSampler, TexCoord).rgb;
    }
    else
    {
        baseColor = objectColor;
    }

    // If object is emissive (stars, sun), skip lighting calculations
    if (isEmissive)
    {
        FragColor = vec4(baseColor, 1.0);
        return;
    }

    // Ambient lighting (controlled per object)
    vec3 ambient = ambientStrength * baseColor;

    // Diffuse lighting
    vec3 norm = normalize(Normal);
    vec3 lightDirection = normalize(lightDir);
    float diff = max(dot(norm, lightDirection), 0.0);
    vec3 diffuse = diff * baseColor;

    // Specular lighting (only for water/ocean)
    float specularStrength = 0.3;
    vec3 viewDir = normalize(viewPos - FragPos);
    vec3 reflectDir = reflect(-lightDirection, norm);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32);
    vec3 specular = specularStrength * spec * vec3(1.0, 1.0, 1.0);

    vec3 result = ambient + diffuse + specular;
    FragColor = vec4(result, 1.0);
}
