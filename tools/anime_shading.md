# Character shading

The desktop uses `assets/shaders/model.vert` / `model.frag`, an OpenGL 3.3
forward renderer, linear lighting with sRGB texture/framebuffer conversion,
skinned geometry, an inverted-hull outline, and an alpha-aware directional
self-shadow map. There is no environment reflection or PBR reflection probe.

## Causes and changes

The previous profiles fed every surface into the same near-white half-vector
highlight. Even a small highlight on a dark textile could read as plastic.
The replacement uses a separate response for each material while retaining the
original albedo samples and factors:

| Profile | Highlight and shading |
| --- | --- |
| Fabric | Dyed grazing sheen proportional to albedo; no mirror lobe |
| Skin | Warm shadow tint, wrapped soft terminator, bounded broad highlight |
| Hair/fur | Colored strand highlight, with a per-material groom direction |
| Metal | Tinted broad shoulder and smaller bright cel highlight |
| Leather | Broad subdued highlight that retains the base color |
| Eye | Small bright glint; no inverted-hull outline |
| Ink | Original brow/lash color independent of illumination |
| Generic | Conservative matte response |

Highlight filtering uses the pixel footprint of the normal field and
attenuates the widened highlight to reduce subpixel flashing. The environmental
fill is a smooth world-space hemisphere. A light-side, material-colored rim
replaces the continuous blue edge halo.

Both bundled rigged exports, including the desktop default
`Raphtalia_prepared.fbx`, contain faceted normals. Repair now runs after scene
transforms and skin weights are available. It uses corner-angle weighted normals
on connected geometric surface fans, joins UV splits, and splits vertices at
creases of at least 45 degrees. The rest normals can be shared across different
bone influences; each vertex still skins its normal with its own weights.
Outline normals continue to respect deformation boundaries. Coincident
disconnected and non-manifold layers do not share a smoothing fan. Positions, UVs, material
factors, texture pixels, bones and animation keys are unchanged. Repair is
explicitly enabled for the two known rigged assets; external models retain
authored normals by default.

For skin, a light-oriented receiver offset and a wider 5x5 shadow filter reduce
triangular self-shadowing from the coarse depth surface. Both are measured in
shadow texels in world/light space, independent of the viewing camera. Other
materials keep a crisper 3x3 filter. Cast shadows remain enabled, including
hair onto the face and contact shadows between nearby surfaces.
Receiver-plane depth gradients are derived from the original depth geometry,
so the smooth normal offset does not create a different comparison plane on
each triangle.

## Material controls

`GLMeshMaterial` exposes `toonKind`, `lightResponse`, `specularStrength`,
`specularPower`, `rimStrength`, `outlineScale`, and:

- `shadowSoftness`: half-width of the cel terminator in N.L space, range
  `[0.005, 0.5]`. Imported skin uses `0.14`, fabric `0.10`, hair `0.075`.
- `strandDirection`: finite nonzero imported-scene-space groom axis. It is
  normalized on upload, follows skeletal/model transforms, and is projected
  onto the smooth surface normal. Default is +Y. This is an axis-based groom,
  not an authored per-strand tangent map.

Mesh names take precedence over shared atlas material names when choosing a
profile. Unknown materials remain Generic. Public material controls can override
the inferred profile without editing the original textures.

Derivatives and implicit texture sampling execute before nonuniform alpha
discard, following the derivative rules in the
[GLSL 3.30 specification](https://registry.khronos.org/OpenGL/specs/gl/GLSLangSpec.3.30.pdf).

## Verification

`tools/build.ps1 -UseInstalledDependencies` builds the desktop bundle, refreshes
its assets, and runs CTest. GPU regressions exercise actual shader pixels:
material separation at exaggerated strength, grazing fabric sheen, directional
lighting, camera/model rotations, light-sweep replay, shadow acne, masked
casters, moving skeletal casters, uniform upload, and invalid parameters.
CPU geometry regressions cover smooth fans, UV seams, equivalent/different
bone influences, hard-edge splitting, and unchanged vertex data.

See [shader_visual_audit.md](shader_visual_audit.md) for the reproducible visual
audit. It renders all three bundled models, four camera views, four extra light
directions, and complete camera/model/light rotations. Repeated views and reset
frames must be byte-identical; every draw/readback checks OpenGL errors.
Generated comparisons and reports are under `build/anime-audit/`.

Validated on 2026-11-07: Release build and all 15 CTest suites passed, including
desktop animation/eye checks and the skin contact-shadow regression. The final
1000x1000, 8x MSAA audit passed on all three models with 48 frames for each of
camera, model and light rotation, zero GL errors and exact reset images.
`build/anime-audit/final-comparison.png` shows the original and final face views
with default and overhead lighting; `after/report.txt` contains the audit results.

Visual checks complement these deterministic tests: image equality alone does
not establish perceptual quality. Smoothing improves surface illumination;
it cannot add geometric silhouette detail that is absent from the asset.
