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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_thermostat_with_large_dial")

    plate_paint = model.material("plate_paint", rgba=(0.94, 0.94, 0.92, 1.0))
    screw_finish = model.material("screw_finish", rgba=(0.78, 0.79, 0.80, 1.0))
    body_shell = model.material("body_shell", rgba=(0.86, 0.87, 0.86, 1.0))
    bezel_finish = model.material("bezel_finish", rgba=(0.73, 0.75, 0.77, 1.0))
    dial_face = model.material("dial_face", rgba=(0.97, 0.97, 0.95, 1.0))
    dial_marker = model.material("dial_marker", rgba=(0.30, 0.31, 0.33, 1.0))

    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        Box((0.140, 0.140, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=plate_paint,
        name="plate_panel",
    )
    wall_plate.visual(
        Box((0.050, 0.016, 0.010)),
        origin=Origin(xyz=(0.0, -0.062, 0.005)),
        material=plate_paint,
        name="mounting_foot",
    )
    wall_plate.visual(
        Cylinder(radius=0.006, length=0.0012),
        origin=Origin(xyz=(-0.032, 0.0, 0.0046)),
        material=screw_finish,
        name="left_screw_cap",
    )
    wall_plate.visual(
        Cylinder(radius=0.006, length=0.0012),
        origin=Origin(xyz=(0.032, 0.0, 0.0046)),
        material=screw_finish,
        name="right_screw_cap",
    )
    wall_plate.inertial = Inertial.from_geometry(
        Box((0.140, 0.140, 0.014)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
    )

    body = model.part("body")
    body.visual(
        Cylinder(radius=0.050, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=body_shell,
        name="rear_housing",
    )
    body.visual(
        Cylinder(radius=0.056, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=bezel_finish,
        name="front_bezel",
    )
    body.visual(
        Box((0.010, 0.004, 0.002)),
        origin=Origin(xyz=(0.0, 0.054, 0.021)),
        material=dial_marker,
        name="reference_tick",
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.056, length=0.024),
        mass=0.34,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.053, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=bezel_finish,
        name="dial_ring",
    )
    dial.visual(
        Cylinder(radius=0.037, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=dial_face,
        name="dial_face",
    )
    dial.visual(
        Box((0.004, 0.016, 0.0015)),
        origin=Origin(xyz=(0.0, 0.030, 0.01125)),
        material=dial_marker,
        name="dial_grip_marker",
    )
    dial.inertial = Inertial.from_geometry(
        Cylinder(radius=0.053, length=0.012),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
    )

    model.articulation(
        "wall_plate_to_body",
        ArticulationType.FIXED,
        parent=wall_plate,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.35, velocity=4.0),
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
        max_gap=0.0001,
        max_penetration=0.0,
        negative_elem="plate_panel",
        name="body seats directly on wall plate",
    )
    ctx.expect_within(
        body,
        wall_plate,
        axes="xy",
        margin=0.0,
        outer_elem="plate_panel",
        name="body footprint stays within wall plate",
    )
    ctx.expect_origin_distance(
        dial,
        body,
        axes="xy",
        max_dist=0.0001,
        name="dial stays centered on body",
    )
    ctx.expect_gap(
        dial,
        body,
        axis="z",
        max_gap=0.0001,
        max_penetration=0.0,
        name="dial sits flush against bezel plane",
    )
    ctx.expect_overlap(
        dial,
        body,
        axes="xy",
        min_overlap=0.100,
        name="dial covers central body opening",
    )

    with ctx.pose({dial_joint: math.pi * 0.75}):
        ctx.expect_origin_distance(
            dial,
            body,
            axes="xy",
            max_dist=0.0001,
            name="rotated dial remains centered",
        )
        ctx.expect_gap(
            dial,
            body,
            axis="z",
            max_gap=0.0001,
            max_penetration=0.0,
            name="rotated dial keeps front clearance",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
