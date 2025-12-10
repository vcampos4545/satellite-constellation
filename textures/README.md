# Textures for Satellite Constellation Simulation

Place your texture files in this directory:
- `earth.jpg` - Earth texture
- `moon.jpg` - Moon texture
- `stars.jpg` - Star field background texture

## Recommended Sources for Clean Earth Textures

For a simulation-friendly texture with clear landmass outlines:

### 1. Solar System Scope (Recommended)
- URL: https://www.solarsystemscope.com/textures/
- Download: "Earth 2K" texture
- Look for simple versions without clouds or atmosphere
- Free for non-commercial use

### 2. Planet Pixel Emporium
- URL: http://planetpixelemporium.com/earth.html
- Provides clean map projections
- Good for technical/simulation use

### 3. Natural Earth
- URL: https://www.naturalearthdata.com/downloads/10m-raster-maps/
- Clean vector-style maps
- Perfect for clear landmass distinction

## What to Look For (Earth)

- **Format**: Equirectangular projection (latitude-longitude map)
- **Style**: Simple land/ocean distinction without clouds
- **Resolution**: 2K (2048x1024) or higher works well
- **Colors**: Clear contrast between land and water

Save your texture as `earth.jpg` in this directory.

---

## Recommended Sources for Moon Textures

### 1. Solar System Scope (Recommended)
- URL: https://www.solarsystemscope.com/textures/
- Download: "Moon 2K" texture
- Clean, high-resolution lunar surface
- Free for non-commercial use

### 2. NASA CGI Moon Kit
- URL: https://svs.gsfc.nasa.gov/cgi-bin/details.cgi?aid=4720
- Official NASA textures
- Very detailed with real topography
- Public domain

### 3. Planet Pixel Emporium
- URL: http://planetpixelemporium.com/moon.html
- Clean lunar surface maps
- Good for simulation use

## What to Look For (Moon)

- **Format**: Equirectangular projection (latitude-longitude map)
- **Style**: Lunar surface with visible craters
- **Resolution**: 2K (2048x1024) or higher works well
- **Colors**: Grayscale or natural lunar coloring

Save your texture as `moon.jpg` in this directory.

---

## Recommended Sources for Star Field Textures

### 1. ESO (European Southern Observatory)
- URL: https://www.eso.org/public/images/
- Search for "milky way" or "star field"
- High-quality astronomical photography
- Free for educational/non-commercial use (CC BY 4.0)

### 2. NASA Image Library
- URL: https://images.nasa.gov/
- Search for "stars panorama" or "milky way 360"
- Real space photography
- Public domain (no copyright restrictions)

### 3. Solar System Scope
- URL: https://www.solarsystemscope.com/textures/
- Download: "Stars" or "Milky Way" texture
- Specifically designed for 3D simulations
- Free for non-commercial use

### 4. OpenGameArt.org
- URL: https://opengameart.org/
- Search for "stars" or "skybox" or "space background"
- Many free CC0/public domain options
- Good variety of styles

### 5. Generate Your Own
- **Spacescape** (Free tool): https://github.com/petrocket/spacescape
  - Open-source skybox/star field generator
  - Fully customizable star density, colors, nebulae
  - Can export equirectangular format

## What to Look For (Stars)

- **Format**: Equirectangular projection (360° panorama)
- **Style**: Star field with visible Milky Way (optional)
- **Resolution**: 4K (4096x2048) or higher recommended for quality
- **Colors**: Natural star colors with dark space background
- **Coverage**: Should be seamless when wrapped around a sphere
- **Brightness**: Not too bright - stars should be subtle background

## Tips for Star Textures

- **Equirectangular is key**: The texture must be a rectangular image that wraps around a sphere (like Earth textures)
- **Avoid HDR-only formats**: Use JPG or PNG that displays properly
- **Dark background**: Stars should be on a dark/black background
- **Check orientation**: Some panoramas may need rotation to align properly
- **Resolution**: Higher resolution = better quality when zoomed in, but 4K is usually sufficient

Save your texture as `stars.jpg` in this directory.
