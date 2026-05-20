from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _revolved_profile(points: list[tuple[float, float]]) -> cq.Workplane:
    """Revolve an X/Z cross-section around the bottle's Z axis."""
    return (
        cq.Workplane("XZ")
        .polyline(points)
        .close()
        .revolve(360.0, axisStart=(0.0, 0.0, 0.0), axisEnd=(0.0, 0.0, 1.0))
    )


def _bottle_shell() -> cq.Workplane:
    # A single revolved hollow profile: thick push-up base, cylindrical body,
    # sloping shoulder, narrow neck, and open mouth lip.
    profile = [
        (0.000, 0.000),
        (0.038, 0.000),
        (0.045, 0.006),
        (0.046, 0.122),
        (0.043, 0.140),
        (0.033, 0.160),
        (0.020, 0.180),
        (0.018, 0.230),
        (0.016, 0.237),
        (0.0105, 0.237),
        (0.0105, 0.188),
        (0.019, 0.168),
        (0.036, 0.143),
        (0.039, 0.020),
        (0.006, 0.020),
        (0.000, 0.000),
    ]
    return _revolved_profile(profile)


def _helix(radius: float, radial_eps: float, pitch: float, height: float, z_fade: float):
    def func(t: float) -> tuple[float, float, float]:
        turns = height / pitch
        # Ease the start and end of the thread into the neck instead of leaving
        # squared-off floating ends.
        fade = 0.10
        if t < fade:
            s = math.sin(math.pi * 0.5 * t / fade)
        elif t > 1.0 - fade:
            s = math.sin(math.pi * 0.5 * (1.0 - t) / fade)
        else:
            s = 1.0
        r = radius + radial_eps * s
        z = height * t + z_fade * s
        a = 2.0 * math.pi * turns * t
        return (r * math.cos(a), r * math.sin(a), z)

    return func


def _external_thread() -> cq.Solid:
    # Small rounded helical rib on the neck; sized to clear the cap's hollow
    # bore while still reading unmistakably as a screw thread.
    radius = 0.0181
    pitch = 0.011
    height = 0.033
    half_width_z = pitch * 0.18
    radial_height = 0.0021

    root_low = cq.Workplane("XY").parametricCurve(
        _helix(radius, 0.0, pitch, height, -half_width_z)
    ).val()
    root_high = cq.Workplane("XY").parametricCurve(
        _helix(radius, 0.0, pitch, height, half_width_z)
    ).val()
    crest_low = cq.Workplane("XY").parametricCurve(
        _helix(radius, radial_height, pitch, height, -half_width_z * 0.45)
    ).val()
    crest_high = cq.Workplane("XY").parametricCurve(
        _helix(radius, radial_height, pitch, height, half_width_z * 0.45)
    ).val()

    faces = [
        cq.Face.makeRuledSurface(root_low, root_high),
        cq.Face.makeRuledSurface(crest_low, crest_high),
        cq.Face.makeRuledSurface(root_low, crest_low),
        cq.Face.makeRuledSurface(root_high, crest_high),
    ]
    return cq.Solid.makeSolid(cq.Shell.makeShell(faces))


def _cap_shell() -> cq.Workplane:
    height = 0.052
    top_thickness = 0.006
    inner_radius = 0.023
    root_radius = 0.027
    rib_depth = 0.0022
    tooth_count = 36
    points = []
    for i in range(tooth_count * 2):
        angle = 2.0 * math.pi * i / (tooth_count * 2)
        radius = root_radius + (rib_depth if i % 2 == 0 else 0.0002)
        points.append((radius * math.cos(angle), radius * math.sin(angle)))

    outer = cq.Workplane("XY").polyline(points).close().extrude(height)
    cavity = (
        cq.Workplane("XY", origin=(0.0, 0.0, -0.001))
        .circle(inner_radius)
        .extrude(height - top_thickness + 0.001)
    )
    return outer.cut(cavity)


def _annular_band(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    outer = cq.Workplane("XY").circle(outer_radius).extrude(height)
    inner = (
        cq.Workplane("XY", origin=(0.0, 0.0, -0.001))
        .circle(inner_radius)
        .extrude(height + 0.002)
    )
    return outer.cut(inner)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="screw_cap_bottle")

    clear_plastic = Material("clear_aqua_plastic", rgba=(0.55, 0.88, 1.0, 0.42))
    thread_plastic = Material("thread_highlight", rgba=(0.70, 0.95, 1.0, 0.70))
    label_paper = Material("cream_label", rgba=(0.94, 0.90, 0.78, 1.0))
    cap_blue = Material("ribbed_blue_cap", rgba=(0.02, 0.20, 0.72, 1.0))
    collar_dark = Material("dark_rotating_collar", rgba=(0.02, 0.035, 0.08, 1.0))
    marker_red = Material("red_rotation_marker", rgba=(0.95, 0.04, 0.02, 1.0))

    bottle = model.part("bottle")
    bottle.visual(
        mesh_from_cadquery(_bottle_shell(), "hollow_bottle_body", tolerance=0.0008),
        material=clear_plastic,
        name="body_shell",
    )
    bottle.visual(
        mesh_from_cadquery(_external_thread(), "external_neck_thread", tolerance=0.0005),
        origin=Origin(xyz=(0.0, 0.0, 0.188)),
        material=thread_plastic,
        name="neck_thread",
    )
    bottle.visual(
        mesh_from_cadquery(_annular_band(0.0470, 0.0454, 0.048), "wrap_label"),
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        material=label_paper,
        name="label_band",
    )

    cap = model.part("cap")
    cap.visual(
        mesh_from_cadquery(_cap_shell(), "ribbed_cap_shell", tolerance=0.0007),
        material=cap_blue,
        name="cap_shell",
    )
    cap.visual(
        mesh_from_cadquery(_annular_band(0.031, 0.026, 0.013), "exposed_rotating_collar"),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=collar_dark,
        name="rotating_collar",
    )
    cap.visual(
        Box((0.0035, 0.010, 0.034)),
        origin=Origin(xyz=(0.0307, 0.0, 0.027)),
        material=marker_red,
        name="rotation_marker",
    )

    model.articulation(
        "bottle_to_cap",
        ArticulationType.CONTINUOUS,
        parent=bottle,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.203)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=8.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    bottle = object_model.get_part("bottle")
    cap = object_model.get_part("cap")
    joint = object_model.get_articulation("bottle_to_cap")

    ctx.check(
        "cap uses continuous screw axis",
        joint.articulation_type == ArticulationType.CONTINUOUS and tuple(joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={joint.articulation_type}, axis={joint.axis}",
    )
    ctx.expect_within(
        bottle,
        cap,
        axes="xy",
        inner_elem="neck_thread",
        outer_elem="cap_shell",
        margin=0.001,
        name="threaded neck sits inside cap collar footprint",
    )
    ctx.expect_overlap(
        cap,
        bottle,
        axes="z",
        elem_a="cap_shell",
        elem_b="neck_thread",
        min_overlap=0.010,
        name="cap skirt covers the threaded neck height",
    )

    marker_rest = ctx.part_element_world_aabb(cap, elem="rotation_marker")
    with ctx.pose({joint: math.pi / 2.0}):
        marker_turned = ctx.part_element_world_aabb(cap, elem="rotation_marker")

    def _center_xy(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return ((lo[0] + hi[0]) * 0.5, (lo[1] + hi[1]) * 0.5)

    rest_xy = _center_xy(marker_rest)
    turned_xy = _center_xy(marker_turned)
    ctx.check(
        "visible marker orbits around neck axis",
        rest_xy is not None
        and turned_xy is not None
        and rest_xy[0] > 0.025
        and abs(rest_xy[1]) < 0.006
        and turned_xy[1] > 0.025
        and abs(turned_xy[0]) < 0.006,
        details=f"rest={rest_xy}, turned={turned_xy}",
    )

    return ctx.report()


object_model = build_object_model()
