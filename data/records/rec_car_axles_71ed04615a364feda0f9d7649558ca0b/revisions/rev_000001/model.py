from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tractor_front_driven_axle")

    cast_iron = model.material("cast_iron", rgba=(0.23, 0.24, 0.26, 1.0))
    dark_cast = model.material("dark_cast", rgba=(0.16, 0.17, 0.18, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.47, 0.49, 0.52, 1.0))

    track_half = 0.82
    spindle_z = -0.04

    beam = model.part("beam")
    beam.visual(
        Box((0.72, 0.20, 0.16)),
        material=cast_iron,
        name="main_beam",
    )
    beam.visual(
        Box((0.44, 0.17, 0.12)),
        origin=Origin(xyz=(0.46, 0.0, -0.01)),
        material=cast_iron,
        name="left_beam_arm",
    )
    beam.visual(
        Box((0.44, 0.17, 0.12)),
        origin=Origin(xyz=(-0.46, 0.0, -0.01)),
        material=cast_iron,
        name="right_beam_arm",
    )
    beam.visual(
        Cylinder(radius=0.19, length=0.26),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0), xyz=(0.0, 0.0, -0.01)),
        material=dark_cast,
        name="differential_housing",
    )
    beam.visual(
        Sphere(radius=0.13),
        origin=Origin(xyz=(0.0, 0.0, -0.11)),
        material=dark_cast,
        name="lower_center_bulge",
    )
    beam.visual(
        Box((0.50, 0.12, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, -0.09)),
        material=dark_cast,
        name="lower_center_rib",
    )
    beam.visual(
        Box((0.22, 0.22, 0.18)),
        origin=Origin(xyz=(0.67, 0.0, 0.0)),
        material=cast_iron,
        name="left_outer_shoulder",
    )
    beam.visual(
        Box((0.08, 0.18, 0.08)),
        origin=Origin(xyz=(0.72, 0.0, 0.05)),
        material=cast_iron,
        name="left_yoke_gusset",
    )
    beam.visual(
        Box((0.12, 0.18, 0.04)),
        origin=Origin(xyz=(track_half, 0.0, 0.10)),
        material=cast_iron,
        name="left_kingpin_cap",
    )
    beam.visual(
        Box((0.08, 0.03, 0.20)),
        origin=Origin(xyz=(track_half, 0.065, 0.0)),
        material=cast_iron,
        name="left_front_plate",
    )
    beam.visual(
        Box((0.08, 0.03, 0.20)),
        origin=Origin(xyz=(track_half, -0.065, 0.0)),
        material=cast_iron,
        name="left_rear_plate",
    )
    beam.visual(
        Box((0.22, 0.22, 0.18)),
        origin=Origin(xyz=(-0.67, 0.0, 0.0)),
        material=cast_iron,
        name="right_outer_shoulder",
    )
    beam.visual(
        Box((0.08, 0.18, 0.08)),
        origin=Origin(xyz=(-0.72, 0.0, 0.05)),
        material=cast_iron,
        name="right_yoke_gusset",
    )
    beam.visual(
        Box((0.12, 0.18, 0.04)),
        origin=Origin(xyz=(-track_half, 0.0, 0.10)),
        material=cast_iron,
        name="right_kingpin_cap",
    )
    beam.visual(
        Box((0.08, 0.03, 0.20)),
        origin=Origin(xyz=(-track_half, 0.065, 0.0)),
        material=cast_iron,
        name="right_front_plate",
    )
    beam.visual(
        Box((0.08, 0.03, 0.20)),
        origin=Origin(xyz=(-track_half, -0.065, 0.0)),
        material=cast_iron,
        name="right_rear_plate",
    )

    left_knuckle = model.part("left_knuckle")
    right_knuckle = model.part("right_knuckle")
    for knuckle in (left_knuckle, right_knuckle):
        knuckle.visual(
            Cylinder(radius=0.039, length=0.16),
            material=machined_steel,
            name="kingpin_barrel",
        )
        knuckle.visual(
            Box((0.10, 0.07, 0.10)),
            origin=Origin(xyz=(0.05, 0.0, spindle_z)),
            material=cast_iron,
            name="barrel_connector",
        )
        knuckle.visual(
            Box((0.22, 0.12, 0.10)),
            origin=Origin(xyz=(0.17, 0.0, spindle_z)),
            material=cast_iron,
            name="spindle_arm",
        )
        knuckle.visual(
            Box((0.14, 0.08, 0.08)),
            origin=Origin(xyz=(0.09, 0.0, -0.11)),
            material=dark_cast,
            name="lower_web",
        )
        knuckle.visual(
            Cylinder(radius=0.10, length=0.04),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0), xyz=(0.30, 0.0, spindle_z)),
            material=machined_steel,
            name="bearing_flange",
        )

    left_hub = model.part("left_hub")
    right_hub = model.part("right_hub")
    for hub in (left_hub, right_hub):
        hub.visual(
            Cylinder(radius=0.12, length=0.04),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0), xyz=(0.02, 0.0, 0.0)),
            material=machined_steel,
            name="inboard_collar",
        )
        hub.visual(
            Cylinder(radius=0.18, length=0.20),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0), xyz=(0.10, 0.0, 0.0)),
            material=cast_iron,
            name="hub_drum",
        )
        hub.visual(
            Cylinder(radius=0.24, length=0.05),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0), xyz=(0.16, 0.0, 0.0)),
            material=cast_iron,
            name="wheel_flange",
        )
        hub.visual(
            Cylinder(radius=0.11, length=0.09),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0), xyz=(0.20, 0.0, 0.0)),
            material=machined_steel,
            name="hub_cap",
        )

    left_kingpin = model.articulation(
        "left_kingpin",
        ArticulationType.REVOLUTE,
        parent=beam,
        child=left_knuckle,
        origin=Origin(xyz=(track_half, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3000.0,
            velocity=1.2,
            lower=-0.65,
            upper=0.65,
        ),
    )
    right_kingpin = model.articulation(
        "right_kingpin",
        ArticulationType.REVOLUTE,
        parent=beam,
        child=right_knuckle,
        origin=Origin(xyz=(-track_half, 0.0, 0.0), rpy=(0.0, 0.0, pi)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3000.0,
            velocity=1.2,
            lower=-0.65,
            upper=0.65,
        ),
    )
    model.articulation(
        "left_hub_spin",
        ArticulationType.CONTINUOUS,
        parent=left_knuckle,
        child=left_hub,
        origin=Origin(xyz=(0.32, 0.0, spindle_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=800.0, velocity=25.0),
    )
    model.articulation(
        "right_hub_spin",
        ArticulationType.CONTINUOUS,
        parent=right_knuckle,
        child=right_hub,
        origin=Origin(xyz=(0.32, 0.0, spindle_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=800.0, velocity=25.0),
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

    beam = object_model.get_part("beam")
    left_knuckle = object_model.get_part("left_knuckle")
    right_knuckle = object_model.get_part("right_knuckle")
    left_hub = object_model.get_part("left_hub")
    right_hub = object_model.get_part("right_hub")

    left_kingpin = object_model.get_articulation("left_kingpin")
    right_kingpin = object_model.get_articulation("right_kingpin")
    left_hub_spin = object_model.get_articulation("left_hub_spin")
    right_hub_spin = object_model.get_articulation("right_hub_spin")

    ctx.check(
        "kingpin joints are revolute",
        left_kingpin.articulation_type == ArticulationType.REVOLUTE
        and right_kingpin.articulation_type == ArticulationType.REVOLUTE,
        details=(
            f"left={left_kingpin.articulation_type}, "
            f"right={right_kingpin.articulation_type}"
        ),
    )
    ctx.check(
        "hub joints are continuous",
        left_hub_spin.articulation_type == ArticulationType.CONTINUOUS
        and right_hub_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=(
            f"left={left_hub_spin.articulation_type}, "
            f"right={right_hub_spin.articulation_type}"
        ),
    )
    ctx.check(
        "all articulation axes follow the axle geometry",
        left_kingpin.axis == (0.0, 0.0, 1.0)
        and right_kingpin.axis == (0.0, 0.0, 1.0)
        and left_hub_spin.axis == (1.0, 0.0, 0.0)
        and right_hub_spin.axis == (1.0, 0.0, 0.0),
        details=(
            f"left_kingpin={left_kingpin.axis}, right_kingpin={right_kingpin.axis}, "
            f"left_hub={left_hub_spin.axis}, right_hub={right_hub_spin.axis}"
        ),
    )

    with ctx.pose({left_kingpin: 0.0, right_kingpin: 0.0}):
        ctx.expect_contact(
            beam,
            left_knuckle,
            elem_a="left_kingpin_cap",
            elem_b="kingpin_barrel",
            contact_tol=0.001,
            name="left kingpin barrel seats under beam cap",
        )
        ctx.expect_contact(
            beam,
            right_knuckle,
            elem_a="right_kingpin_cap",
            elem_b="kingpin_barrel",
            contact_tol=0.001,
            name="right kingpin barrel seats under beam cap",
        )
        ctx.expect_contact(
            left_knuckle,
            left_hub,
            elem_a="bearing_flange",
            elem_b="inboard_collar",
            contact_tol=0.001,
            name="left hub mounts to knuckle bearing flange",
        )
        ctx.expect_contact(
            right_knuckle,
            right_hub,
            elem_a="bearing_flange",
            elem_b="inboard_collar",
            contact_tol=0.001,
            name="right hub mounts to knuckle bearing flange",
        )

    left_rest = ctx.part_world_position(left_hub)
    right_rest = ctx.part_world_position(right_hub)
    with ctx.pose({left_kingpin: 0.45, right_kingpin: 0.45}):
        left_steered = ctx.part_world_position(left_hub)
        right_steered = ctx.part_world_position(right_hub)

    ctx.check(
        "left hub moves forward when the left knuckle steers positively",
        left_rest is not None
        and left_steered is not None
        and left_steered[1] > left_rest[1] + 0.05,
        details=f"rest={left_rest}, steered={left_steered}",
    )
    ctx.check(
        "right hub moves rearward when the right knuckle steers positively",
        right_rest is not None
        and right_steered is not None
        and right_steered[1] < right_rest[1] - 0.05,
        details=f"rest={right_rest}, steered={right_steered}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
