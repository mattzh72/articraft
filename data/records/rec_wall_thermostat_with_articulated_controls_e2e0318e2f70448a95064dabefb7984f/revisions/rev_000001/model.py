from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_thermostat_large_dial")

    wall_white = model.material("wall_white", rgba=(0.95, 0.95, 0.94, 1.0))
    shell_white = model.material("shell_white", rgba=(0.93, 0.93, 0.91, 1.0))
    bezel_gray = model.material("bezel_gray", rgba=(0.82, 0.84, 0.86, 1.0))
    dial_aluminum = model.material("dial_aluminum", rgba=(0.71, 0.74, 0.78, 1.0))
    dark_glass = model.material("dark_glass", rgba=(0.17, 0.21, 0.24, 1.0))
    charcoal = model.material("charcoal", rgba=(0.22, 0.24, 0.27, 1.0))

    plate_width = 0.145
    plate_height = 0.105
    plate_thickness = 0.0045
    dial_outer_radius = 0.0428

    wall_plate = model.part("wall_plate")
    plate_profile = rounded_rect_profile(plate_width, plate_height, 0.016, corner_segments=8)
    wall_plate.visual(
        mesh_from_geometry(
            ExtrudeGeometry.from_z0(plate_profile, plate_thickness),
            "thermostat_wall_plate",
        ),
        material=wall_white,
        name="plate_shell",
    )
    wall_plate.inertial = Inertial.from_geometry(
        Box((plate_width, plate_height, plate_thickness)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, plate_thickness * 0.5)),
    )

    body = model.part("body")
    body_profile = rounded_rect_profile(0.118, 0.090, 0.018, corner_segments=8)
    body.visual(
        mesh_from_geometry(
            ExtrudeGeometry.from_z0(body_profile, 0.011),
            "thermostat_body_pod",
        ),
        origin=Origin(xyz=(0.008, 0.0, 0.0)),
        material=shell_white,
        name="body_pod",
    )
    body.visual(
        Box((0.028, 0.060, 0.018)),
        origin=Origin(xyz=(0.058, 0.0, 0.009)),
        material=shell_white,
        name="right_bulge",
    )
    body.visual(
        Cylinder(radius=0.0405, length=0.0105),
        origin=Origin(xyz=(0.0, 0.0, 0.00525)),
        material=bezel_gray,
        name="bezel_shoulder",
    )
    body.visual(
        Cylinder(radius=0.024, length=0.007),
        origin=Origin(xyz=(0.0, 0.0, 0.0145)),
        material=charcoal,
        name="display_cap",
    )
    body.visual(
        Box((0.020, 0.020, 0.004)),
        origin=Origin(xyz=(0.055, 0.0, 0.017)),
        material=dark_glass,
        name="display_window",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.118, 0.090, 0.022)),
        mass=0.32,
        origin=Origin(xyz=(0.008, 0.0, 0.011)),
    )

    dial = model.part("dial")
    dial_profile = [
        (0.0310, 0.0000),
        (0.0400, 0.0000),
        (0.0420, 0.0012),
        (0.0428, 0.0046),
        (0.0415, 0.0080),
        (0.0385, 0.0092),
        (0.0320, 0.0092),
        (0.0310, 0.0070),
        (0.0310, 0.0000),
    ]
    dial.visual(
        mesh_from_geometry(
            LatheGeometry(dial_profile, segments=72),
            "thermostat_dial_ring",
        ),
        material=dial_aluminum,
        name="dial_ring",
    )
    dial.visual(
        Box((0.006, 0.010, 0.0024)),
        origin=Origin(xyz=(0.0, 0.0365, 0.0104)),
        material=charcoal,
        name="grip_marker",
    )
    dial.inertial = Inertial.from_geometry(
        Cylinder(radius=dial_outer_radius, length=0.012),
        mass=0.09,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
    )

    model.articulation(
        "plate_to_body",
        ArticulationType.FIXED,
        parent=wall_plate,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, plate_thickness)),
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, 0.0, 0.0105)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=6.0),
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
    wall_plate = object_model.get_part("wall_plate")
    body = object_model.get_part("body")
    dial = object_model.get_part("dial")
    dial_joint = object_model.get_articulation("body_to_dial")

    ctx.expect_gap(
        body,
        wall_plate,
        axis="z",
        max_gap=0.001,
        max_penetration=1e-6,
        name="body seats directly on the wall plate",
    )
    ctx.expect_overlap(
        body,
        wall_plate,
        axes="xy",
        min_overlap=0.085,
        name="body remains carried by the wall plate footprint",
    )
    ctx.expect_within(
        dial,
        body,
        axes="xy",
        inner_elem="dial_ring",
        outer_elem="body_pod",
        margin=0.0,
        name="dial ring stays within the shallow body footprint",
    )
    ctx.expect_gap(
        dial,
        body,
        axis="z",
        positive_elem="dial_ring",
        negative_elem="bezel_shoulder",
        max_gap=0.0003,
        max_penetration=1e-6,
        name="dial ring seats on the bezel shoulder",
    )

    body_pod_aabb = ctx.part_element_world_aabb(body, elem="body_pod")
    dial_ring_aabb = ctx.part_element_world_aabb(dial, elem="dial_ring")
    body_center_x = None
    dial_center_x = None
    if body_pod_aabb is not None:
        body_center_x = (body_pod_aabb[0][0] + body_pod_aabb[1][0]) * 0.5
    if dial_ring_aabb is not None:
        dial_center_x = (dial_ring_aabb[0][0] + dial_ring_aabb[1][0]) * 0.5
    ctx.check(
        "fixed housing stays offset to the right of the dial axis",
        body_center_x is not None
        and dial_center_x is not None
        and body_center_x > dial_center_x + 0.006,
        details=f"body_center_x={body_center_x}, dial_center_x={dial_center_x}",
    )

    rest_marker = ctx.part_element_world_aabb(dial, elem="grip_marker")
    rest_center = None
    if rest_marker is not None:
        rest_center = tuple((lo + hi) * 0.5 for lo, hi in zip(rest_marker[0], rest_marker[1]))

    quarter_center = None
    with ctx.pose({dial_joint: math.pi / 2.0}):
        quarter_marker = ctx.part_element_world_aabb(dial, elem="grip_marker")
        if quarter_marker is not None:
            quarter_center = tuple(
                (lo + hi) * 0.5 for lo, hi in zip(quarter_marker[0], quarter_marker[1])
            )

    ctx.check(
        "dial marker moves around the thermostat face",
        rest_center is not None
        and quarter_center is not None
        and rest_center[1] > 0.03
        and quarter_center[0] < -0.03,
        details=f"rest_center={rest_center}, quarter_center={quarter_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
