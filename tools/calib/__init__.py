"""Cross-device calibration: how a camera pixel relates to an arm millimetre.

This layer belongs to neither device. The mapping cannot be measured without
moving the arm and cannot be interpreted without the camera, so putting it under
either one would make that package responsible for the other's geometry.

Two modules, two measurement paradigms over the same underlying Jacobian --
keep it that way rather than adding a third:

``pixel_to_arm``  Tracks a *feature* across three poses (base, +x, +y) and solves
                  J from where the marker landed. Direct, and it fails loudly
                  when the marker is lost. Carries the sign contract for turning
                  a pixel offset into the arm correction that cancels it.

``pixel_scale``   Measures *whole-frame* displacement by phase correlation, so it
                  is unbiased by a marker that is clipped or partly unlit, and it
                  additionally fits the height model s(z) = A/(B - z). Use this
                  when the scale is needed at more than one height.

The sign convention is the thing to be careful with. ``PixelToArm.apply``
returns ``-(J^-1) @ offset_px``: the arm move that *cancels* the observed
offset, not the one that would have produced it. Dropping that negation inverts
the control loop and drives the marker away from centre instead of toward it.
Both modules document it; neither should be "simplified".
"""
from . import pixel_scale, pixel_to_arm
from .pixel_to_arm import PixelToArm, solve_from_probes

__all__ = ["PixelToArm", "pixel_scale", "pixel_to_arm", "solve_from_probes"]
