# Vision-Adjacent Spot Monitoring

This BluePilot port is based on **StarPilot pull request #75, “Vision-Adjacent Spot
Monitoring,”** authored by Prabhaav Pillai. The detector model and the original
camera-region, smoothing, CPU-throttling, and blind-spot integration concepts come
from that work:

- https://github.com/firestar5683/StarPilot/pull/75
- https://github.com/prabhaavp/vasm-op

The bundled `v_asm_model.onnx` is the model from that pull request (SHA-256
`ca089be316c7346ce37bb28b8185f178620cb91e20cbcef52d5a97817520e0c2`).

BluePilot adaptations include typed SunnyPilot parameters, BluePilot Portal window
annotation, SunnyConnect and on-device Vehicle settings, stale-state protection, and
integration with BluePilot/SunnyPilot lane-change and visualization paths.
