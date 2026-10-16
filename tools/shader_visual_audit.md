# Anime shader visual audit

Build the `julia_shader_visual_audit` CMake target (available when
`JULIA_BUILD_TESTS=ON`), then run from the repository root:

```powershell
build/vs-release/tests/julia_shader_visual_audit.exe --shader-assets assets --model-assets assets --output-dir build/anime-audit/after
```

`--shader-assets` is the resource root containing `shaders/model.vert` and
`shaders/model.frag`. It permits comparing compatible shader snapshots with the
same executable. To include renderer/importer changes, render the baseline before
rebuilding and save its executable separately. `--model-assets` contains `models/`.
All paths default to the repository assets except the output directory, which
defaults to `build/anime-audit/current`.

Each run requires a real OpenGL 3.3 context with an sRGB-capable, 1000 x 1000,
8-sample framebuffer. An unsupported configuration fails instead of silently
changing the comparison. Vsync is disabled. Both FBX models are normalized to
height 2.2. `Raphtalia1.fbx` uses default import options and a static draw;
`Raphtalia.fbx` rebuilds smooth normals and uses its imported bind-pose skinning
palette.

The four fixed views use +Z for the front, +X for the profile, -Z for the back,
and a front face closeup. The same camera coordinates, viewport, fit transform,
and renderer defaults are used for both models and every run. PPM files store
the sRGB framebuffer bytes with a top-left image origin. External conversion is
optional, for example `python -c "from PIL import Image; Image.open('frame.ppm').save('frame.png')"`.

`report.txt` records the GPU/driver, geometry, settings, image hashes, and checks.
`timing.csv` records GPU elapsed-query and wall-time averages for camera orbits
and model turns. Defaults are 30 warmup and 90 measured frames per sequence;
override with `--warmup N --frames N`. Pixel readback, image writing, import,
shader compilation, and warmup are outside the measured interval. Timings remain
sensitive to GPU clocks, other GPU activity, and driver compilation behavior;
compare several runs under similar conditions before claiming a speedup.

Each static view must repeat byte-for-byte. A complete camera orbit and model
turn (`--motion-frames N`, default 48) exercise changing views and restore the
initial state, which must again reproduce exactly. Four snapshots per sequence
are saved. Every draw/readback and the final state check OpenGL errors. These
checks establish deterministic rendering and valid GL execution; they do not
automatically prove the absence of perceptual flicker, subpixel line changes,
self-shadowing artifacts, or skeletal animation artifacts. Inspect the saved
views and motion snapshots and use the engine for final animated review.
