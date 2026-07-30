# tacoRender — renderer map

`ext/tacoRender` (git submodule, own repo). A **C23** library wrapping a customized
**raylib** build (`CUSTOMIZE_BUILD`, HDR + JPG formats on). It's a **deferred PBR
renderer**: geometry → G-buffer, then screen-space lighting/shadows/SSAO/post, all done by
drawing a full-screen plane through GLSL fragment shaders. Uses raw GL via raylib's bundled
`glad`, and the C23 `#embed` directive to bake shaders + a BRDF LUT into the binary.

`extern "C"` throughout — consumed from C++ (the engine) directly. `src` is a public
include dir.

## Files

| File | Role |
|------|------|
| `tacoRender.{h,c}` | `Init3D`, G-buffer + presenter FBO lifecycle, `BeginGBufferMode`, blit helpers |
| `tr_effects.{h,c}` | the pipeline: lighting passes, shadow maps, SSAO, skybox, tone map, gamma, blur |
| `tr_shaders.{h,c}` | embeds every `.glsl` (via `#embed`), compiles them, `GetShader(enum)` |
| `tr_generators.{h,c}` | IBL cubemap generation: irradiance + prefiltered radiance |
| `tr_math.{h,c}` | `TransformAABB`, `Frustum` from camera, AABB-in-frustum (CPU culling) |
| `tr_timing.{h,c}` | GL `GL_TIME_ELAPSED` query wrapper → milliseconds (`Timer`) |
| `src/shaders/*.glsl` | the embedded GLSL (see enum below) |
| `src/assets/brdf.png` | precomputed BRDF integration LUT, embedded for IBL |
| `examples/*.cpp` | standalone demos, one per feature (pbr_sphere, ibl_*, shadow_map, …) |

## G-buffer layout (`GBuffers`)

One FBO, 6 colour attachments + a depth texture (all screen-sized):

| # | Attachment | Format | Contents |
|---|-----------|--------|----------|
| 0 | albedo | RGBA8 | base colour, **alpha used to discard** empty texels |
| 1 | normal | RGB16 | world-space normal |
| 2 | metallic | R8 | |
| 3 | roughness | R8 | |
| 4 | emission | RGB8 | (bilinear-filtered) |
| 5 | ao | R8 | ambient occlusion (also SSAO target, clamp-wrapped) |
| – | depth | 24-bit depth texture | |

`GBufferPresenter` adds the working set: `back[0]`/`back[1]` — two **RGBA16F** ping-pong
`RenderTexture`s that accumulate the lit image — plus an `occlusion` FBO that renders back
into the G-buffer's `ao` attachment (used by SSAO min-blend). `LoadPresenter` /
`UnloadPresenter` manage them; the engine reloads both on window resize.

## Draw model

- `Init3D()` compiles all shaders and enables seamless cubemaps. `Uninit3D()` frees them.
- **Geometry pass:** `BeginGBufferMode(gbuffers)` binds the G-buffer FBO with color blend
  off; you draw meshes with `GetGBufferShader()` (raylib `DrawMesh`); `EndGBufferMode()`.
  `gbuf.vs/fs` writes the 6 MRT outputs, builds the normal from a normal map + TBN
  (approximating the tangent from screen-space derivatives if none is uploaded), and
  `discard`s where `albedo.a == 0`.
- **Screen-space passes** all go through `RunLightShaderPro` (and its `…Ex` / plain
  wrappers): it draws a unit plane under an **orthographic top-down camera**, binding the
  G-buffer textures to fixed sampler slots
  `0 albedo, 1 metallic, 2 normal, 3 roughness, 4 ao, 5 emission, 6 depth`, then cubemaps at
  slot 7+, then extra textures. It uploads `invView`, `invProj`, `camView`, `camProj`,
  `screenSize`, and the camera position, so shaders can reconstruct world position from
  depth. Lights are **additive**: `BeginLightingPass` sets `BLEND_ADD` into `back[0]` and
  each light shader adds its contribution.
- `RunPostProcessShader` ping-pongs `back[0]→back[1]→back[0]` through a shader (used by
  gamma and tone-map). `Present`/the engine's blit just draws `back[0]` to the screen.

## Lighting

- **`LightSun`** (`sun.fs`) — directional light with **cascaded shadow maps** (CSM). Full
  Cook-Torrance PBR: GGX distribution, Smith geometry, Fresnel-Schlick, **Burley diffuse**.
  World position is reconstructed from depth; the cascade is chosen by linearized depth; the
  shadow term is an **exponential shadow map** (`exp(-c·(d−z))`) sampled with `textureProj`.
  `c` is scaled per-cascade by `cascadeSpans[i]` (the world distance that cascade's [0,1]
  depth range covers, `= (maxZ−minZ)·2.5` from the ortho fit) so the falloff is constant in
  world-space across cascades. Two tuning knobs at the top of `CalcShadowFactor`:
  `SHADOW_HARDNESS` (1/world-units, crispness) and `SHADOW_BIAS` (world-units, kills acne).
- **`LightIBL`** (`ibl.fs`) — image-based lighting from an `Environment`: prefiltered
  radiance cubemap + irradiance cubemap + the embedded BRDF LUT. `radianceMaps` uniform =
  mip count of the radiance cubemap.
- **`LightPoint`** (`point.fs`) — point light. **Exists but the engine never calls it.**

## Shadow maps (`ShadowMap`, `tr_effects.c`)

`LoadShadowMap(size, cascades, cascadeDistance)`: one `DEPTH_COMPONENT32` texture per
cascade (border-clamped to white so outside-frustum = lit), plus two `R32F` ping-pong
buffers for filtering. Cascade split distances come from the `EndOfCascade` heuristic.

`BeginShadowMap(map, camera, lightDir, cascade)`: builds the light view (`LookAt` along the
light direction) and a **tight orthographic projection fitted to that cascade's slice of the
camera frustum** — it takes the sub-frustum's 8 corners, transforms them into light space,
and takes the min/max box. Stores `projection = lightView · lightProj` for the sun shader.
Then you draw all shadow casters. `FilterShadowMap(map, iterations, maxCascade)` gaussian-
blurs each cascade's depth into the R32 buffers and writes it back via the `tex_to_depth`
shader (ESM-style). `iterations == 0` skips filtering.

## SSAO (`ApplySSAO`)

Runs `ssao.fs` (needs normal + depth) into `back[0]`, box-depth-blurs it, then **min-blends**
the result into the G-buffer's `ao` attachment through the `occlusion` FBO. Later lighting
passes read that combined AO.

## IBL generation (`tr_generators.c`)

Both driven by `GenTextureCubemap`, which renders a cube's 6 faces (90° perspective per
face) through a shader, once per mip level:
- **`IrradianceCubemap(cubemap)`** → 32px diffuse irradiance convolution (`irradiance.fs`).
- **`PrefilterCubemap(cubemap)`** → full mip chain, roughness-prefiltered specular radiance
  (`prefilter.fs`), `ilog2(width)+1` mips.

The engine's `Environment` (comp/Lights.cpp) calls these when you construct it from an
equirect/HDR `Image`.

## Embedded shaders (`EmbeddedShader` enum)

`SHADER_GBUF`, `SHADER_DEPTH_DISPLAY`, `SHADER_ADD`, `SHADER_BLUR_GAUSS`, `SHADER_BLUR_BOX`,
`SHADER_BLUR_BOX_DEPTH`, `SHADER_SKYBOX`, `SHADER_FLIP_Y`, `SHADER_POINT`, `SHADER_SUN`,
`SHADER_IBL`, `SHADER_GAMMA`, `SHADER_TONE_MAP_REINHARD`, `SHADER_IRRADIANCE`,
`SHADER_PREFILTER`, `SHADER_SSAO`, `SHADER_TEX_TO_DEPTH`, `SHADER_COPY_BACKGROUND`.

`FLIP_Y` is the shared vertex shader for full-screen passes (`flip.vs`); several passes use
`NULL` vertex → raylib's default.

## Blur (`BlurTexture`)

`BLUR_GAUSS`, `BLUR_BOX`, `BLUR_BOX_DEPTH` — separable, ping-pong horizontal/vertical over
`iterations` passes. Used by shadow filtering and SSAO.
