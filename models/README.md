# 3D Models for Satellite Constellation Simulation

This directory contains 3D models (.obj files) for different satellite types. Each satellite type has its own subdirectory with the model files and textures.

## Current Models

- `starlink/` - SpaceX Starlink satellite model
- `cubesat1U/` - 1U CubeSat (10cm cube) model
- `cubesat2U/` - 2U CubeSat (10x10x20cm) model

## Directory Structure

Each model directory should follow this structure:
```
model_name/
├── model_name.obj      # The 3D mesh file
├── model_name.mtl      # Material definitions (colors, textures)
└── textures/           # Texture images referenced by .mtl file
    └── *.jpg/png       # Texture files
```

## How to Add a New Satellite Model

### 1. Create Model Directory

Create a new folder in the `models/` directory with your satellite name:
```bash
cd models
mkdir my_satellite
```

### 2. Add Your Model Files

Place your `.obj` file and `.mtl` file in the directory:
```
models/my_satellite/
├── my_satellite.obj
└── my_satellite.mtl
```

**Important**: The `.obj` and `.mtl` files must have the **same base name** as the directory.

### 3. Add Textures (if any)

If your model uses textures, create a `textures/` subdirectory:
```bash
cd my_satellite
mkdir textures
```

Place texture files referenced by your `.mtl` file in this directory.

### 4. Update Code to Use Your Model

In `src/Universe.cpp`, update the satellite creation code:

```cpp
// Add this near the top of the file
OBJMesh mySatelliteMesh;

// In the initialization section
bool mySatelliteMeshLoaded = mySatelliteMesh.load("models/my_satellite/my_satellite.obj");

// In the rendering section
if (satellite->getName().find("MySatellite") != std::string::npos && mySatelliteMeshLoaded)
{
    mySatelliteMesh.draw(*shader);
}
```

## Where to Find Satellite 3D Models

### Free Sources

#### 1. NASA 3D Resources (Recommended)
- **URL**: https://nasa3d.arc.nasa.gov/models
- **Content**: Official NASA spacecraft models
- **Format**: Multiple formats including OBJ
- **License**: Public domain (free for any use)
- **Examples**: ISS, James Webb, Mars rovers, various satellites

#### 2. Sketchfab
- **URL**: https://sketchfab.com/
- **Search**: "satellite", "spacecraft", "cubesat"
- **Filter**: Check "Downloadable" and look for CC licenses
- **Format**: Usually includes OBJ export
- **License**: Varies (check each model - look for CC0 or CC-BY)
- **Quality**: Very high quality, many realistic models

#### 3. TurboSquid Free Section
- **URL**: https://www.turbosquid.com/Search/3D-Models/free/satellite
- **Content**: Professional-quality models
- **Format**: OBJ available
- **License**: Free models with attribution
- **Note**: Check license for each model

#### 4. Free3D
- **URL**: https://free3d.com/3d-models/satellite
- **Content**: Various satellite models
- **Format**: OBJ, FBX
- **License**: Free for personal/educational use (check each model)

#### 5. CGTrader Free Section
- **URL**: https://www.cgtrader.com/free-3d-models/space/satellite
- **Content**: Satellites, spacecraft, space stations
- **Format**: Multiple formats
- **License**: Varies (filter by "Free")

#### 6. OpenGameArt.org
- **URL**: https://opengameart.org/
- **Search**: "satellite" or "spacecraft"
- **Format**: Various, often OBJ available
- **License**: CC0 / Public Domain options available

### Creating Your Own

#### Blender (Free, Recommended)
- **URL**: https://www.blender.org/
- **Export**: File → Export → Wavefront (.obj)
- **Tutorial**: YouTube has excellent Blender spacecraft modeling tutorials
- **Tips**:
  - Keep polygon count reasonable (<50k triangles for performance)
  - Use simple materials for simulation (not critical for realism)
  - Export with "Include UVs", "Write Materials", and "Triangulate Faces"

## Model Requirements

### Format
- **File type**: Wavefront OBJ (.obj)
- **Material file**: MTL (.mtl) - optional but recommended
- **Coordinate system**: Z-up preferred (simulation uses Z-up)
- **Units**: Meters (will be scaled in code as needed)

### Size Guidelines
- **CubeSat 1U**: 0.1m × 0.1m × 0.1m (actual size)
- **CubeSat 2U**: 0.1m × 0.1m × 0.2m (actual size)
- **Starlink**: ~3m × 1.5m (approximate, with solar panels)
- **Scale**: Models will be scaled in code, so exact size is not critical

### Performance
- **Polygon count**: <50,000 triangles recommended
- **Textures**: 2K resolution or less (2048×2048)
- **Complexity**: Simple geometry is better for performance

### Orientation
- **Forward**: +X axis (direction satellite points)
- **Right**: +Y axis
- **Up**: +Z axis (antenna/solar panel normal)
- **Note**: Orientation can be adjusted in code if needed

## Converting Other Formats to OBJ

If you have a model in another format (FBX, STL, DAE, etc.), use Blender to convert:

1. **Open Blender** (free download from blender.org)
2. **Import your model**: File → Import → [your format]
3. **Optional**: Adjust scale, rotation, materials
4. **Export as OBJ**: File → Export → Wavefront (.obj)
5. **Export settings**:
   - ✅ Include UVs
   - ✅ Write Materials
   - ✅ Triangulate Faces
   - ✅ Objects as OBJ Objects
   - Coordinate system: Z up (if option available)

## Example: Adding a GPS Satellite Model

1. Find a GPS satellite model (e.g., from NASA 3D Resources)
2. Download and extract the model files
3. Create directory: `models/gps/`
4. Copy files:
   ```
   models/gps/
   ├── gps.obj
   ├── gps.mtl
   └── textures/
       └── gps_diffuse.jpg
   ```
5. In `include/Renderer.h`, add:
   ```cpp
   OBJMesh gpsMesh;
   bool gpsMeshLoaded;
   ```
6. In `src/Renderer.cpp`, load the model:
   ```cpp
   gpsMeshLoaded = gpsMesh.load("models/gps/gps.obj");
   ```
7. In rendering code, check satellite name and draw:
   ```cpp
   if (satellite->getName().find("GPS") != std::string::npos && gpsMeshLoaded)
   {
       gpsMesh.draw(*sphereShader);
   }
   ```

## Troubleshooting

### Model not appearing
- Check file paths match exactly (case-sensitive on Linux/Mac)
- Verify .obj and .mtl files have same base name
- Check console output for loading errors

### Model appears black/wrong color
- Check .mtl file exists and is properly referenced
- Verify texture paths in .mtl are relative to model directory
- Material colors may need adjustment in .mtl file

### Model appears inside-out
- Some models have reversed face normals
- Fix in Blender: Edit Mode → Select All → Mesh → Normals → Flip

### Model too large/small
- Adjust scale in rendering code (multiply by scale factor)
- Or edit in Blender and re-export

### Performance issues with detailed models
- Reduce polygon count using Blender's Decimate modifier
- Reduce texture resolution
- Use simpler models for distant objects

## Tips for Best Results

1. **Use real satellite references**: NASA images help with proportions
2. **Keep it simple**: Low-poly models run faster
3. **Test in simulation**: Load and view before adding complex details
4. **Consistent scale**: Use real-world dimensions when possible
5. **Materials**: Simple colored materials work well for simulation
6. **Attribution**: Keep track of model sources and licenses

## License Considerations

When downloading models:
- ✅ **Public Domain**: Free for any use (NASA models)
- ✅ **CC0**: Free for any use, no attribution required
- ✅ **CC-BY**: Free with attribution (credit the creator)
- ⚠️ **CC-BY-NC**: Non-commercial only (check if your use qualifies)
- ❌ **All Rights Reserved**: Don't use without permission

Always check the specific license for each model you download.
