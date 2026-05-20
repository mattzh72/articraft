from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TrunnionYokeGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_mounted_yaw_pitch_module")

    cast_aluminum = model.material("cast_aluminum", rgba=(0.58, 0.60, 0.62, 1.0))
    dark_bearing = model.material("dark_bearing", rgba=(0.05, 0.055, 0.06, 1.0))
    blue_anodized = model.material("blue_anodized", rgba=(0.08, 0.22, 0.55, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    brass_pin = model.material("brass_pin", rgba=(0.82, 0.62, 0.28, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.18, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_bearing,
        name="floor_flange",
    )
    pedestal.visual(
        Cylinder(radius=0.065, length=0.220),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material=cast_aluminum,
        name="short_tower",
    )
    pedestal.visual(
        Cylinder(radius=0.125, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.275)),
        material=dark_bearing,
        name="fixed_bearing_race",
    )
    pedestal.visual(
        Cylinder(radius=0.082, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.286)),
        material=black_rubber,
        name="slew_seal",
    )

    yaw_base = model.part("yaw_base")
    yaw_base.visual(
        Cylinder(radius=0.130, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=blue_anodized,
        name="rotating_turntable",
    )
    yaw_base.visual(
        Cylinder(radius=0.050, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.066)),
        material=cast_aluminum,
        name="turret_neck",
    )
    yaw_base.visual(
        Box((0.050, 0.028, 0.012)),
        origin=Origin(xyz=(0.0, 0.120, 0.0415)),
        material=brass_pin,
        name="front_index",
    )

    pitch_bearing = TrunnionYokeGeometry(
        (0.240, 0.086, 0.165),
        span_width=0.140,
        trunnion_diameter=0.030,
        trunnion_center_z=0.106,
        base_thickness=0.026,
        corner_radius=0.006,
        center=False,
    )
    yaw_base.visual(
        mesh_from_geometry(pitch_bearing, "pitch_bearing_yoke"),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=cast_aluminum,
        name="pitch_bearing_yoke",
    )

    pitch_yoke = model.part("pitch_yoke")
    pitch_yoke.visual(
        Cylinder(radius=0.015, length=0.220),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass_pin,
        name="trunnion_shaft",
    )
    pitch_yoke.visual(
        Box((0.092, 0.046, 0.046)),
        origin=Origin(xyz=(0.0, 0.016, 0.0)),
        material=blue_anodized,
        name="central_hub",
    )
    pitch_yoke.visual(
        Box((0.018, 0.185, 0.078)),
        origin=Origin(xyz=(-0.052, 0.104, 0.0)),
        material=blue_anodized,
        name="yoke_arm_0",
    )
    pitch_yoke.visual(
        Box((0.018, 0.185, 0.078)),
        origin=Origin(xyz=(0.052, 0.104, 0.0)),
        material=blue_anodized,
        name="yoke_arm_1",
    )
    pitch_yoke.visual(
        Box((0.122, 0.026, 0.078)),
        origin=Origin(xyz=(0.0, 0.191, 0.0)),
        material=blue_anodized,
        name="front_crossbar",
    )
    pitch_yoke.visual(
        Box((0.076, 0.018, 0.052)),
        origin=Origin(xyz=(0.0, 0.211, 0.0)),
        material=black_rubber,
        name="payload_pad",
    )

    model.articulation(
        "pedestal_to_yaw_base",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=yaw_base,
        origin=Origin(xyz=(0.0, 0.0, 0.295)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.6, lower=-math.pi, upper=math.pi),
    )

    model.articulation(
        "yaw_base_to_pitch_yoke",
        ArticulationType.REVOLUTE,
        parent=yaw_base,
        child=pitch_yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.140)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=-0.70, upper=0.85),
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

    pedestal = object_model.get_part("pedestal")
    yaw_base = object_model.get_part("yaw_base")
    pitch_yoke = object_model.get_part("pitch_yoke")
    yaw_joint = object_model.get_articulation("pedestal_to_yaw_base")
    pitch_joint = object_model.get_articulation("yaw_base_to_pitch_yoke")

    ctx.check(
        "yaw joint is vertical revolute",
        yaw_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(yaw_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={yaw_joint.articulation_type}, axis={yaw_joint.axis}",
    )
    ctx.check(
        "pitch joint is horizontal revolute",
        pitch_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(pitch_joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={pitch_joint.articulation_type}, axis={pitch_joint.axis}",
    )

    ctx.expect_gap(
        yaw_base,
        pedestal,
        axis="z",
        max_gap=0.001,
        max_penetration=0.000001,
        positive_elem="rotating_turntable",
        negative_elem="fixed_bearing_race",
        name="turntable sits on fixed bearing race",
    )
    ctx.expect_overlap(
        yaw_base,
        pedestal,
        axes="xy",
        min_overlap=0.12,
        elem_a="rotating_turntable",
        elem_b="fixed_bearing_race",
        name="yaw bearing footprints overlap",
    )
    ctx.expect_within(
        pitch_yoke,
        yaw_base,
        axes="x",
        margin=0.012,
        inner_elem="trunnion_shaft",
        outer_elem="pitch_bearing_yoke",
        name="pitch shaft is captured between yoke cheeks",
    )
    ctx.allow_overlap(
        pitch_yoke,
        yaw_base,
        elem_a="trunnion_shaft",
        elem_b="pitch_bearing_yoke",
        reason=(
            "The pitch shaft is intentionally modeled as a captured pin running "
            "through the simplified bearing yoke."
        ),
    )
    ctx.expect_overlap(
        pitch_yoke,
        yaw_base,
        axes="x",
        min_overlap=0.20,
        elem_a="trunnion_shaft",
        elem_b="pitch_bearing_yoke",
        name="pitch shaft spans both bearing cheeks",
    )

    def element_z_center(part, elem: str) -> float | None:
        box = ctx.part_element_world_aabb(part, elem=elem)
        if box is None:
            return None
        return 0.5 * (box[0][2] + box[1][2])

    def element_x_center(part, elem: str) -> float | None:
        box = ctx.part_element_world_aabb(part, elem=elem)
        if box is None:
            return None
        return 0.5 * (box[0][0] + box[1][0])

    rest_front_z = element_z_center(pitch_yoke, "front_crossbar")
    with ctx.pose({pitch_joint: 0.60}):
        raised_front_z = element_z_center(pitch_yoke, "front_crossbar")
    ctx.check(
        "positive pitch raises carried yoke front",
        rest_front_z is not None
        and raised_front_z is not None
        and raised_front_z > rest_front_z + 0.06,
        details=f"rest_z={rest_front_z}, raised_z={raised_front_z}",
    )

    rest_index_x = element_x_center(yaw_base, "front_index")
    with ctx.pose({yaw_joint: 0.70}):
        yawed_index_x = element_x_center(yaw_base, "front_index")
    ctx.check(
        "positive yaw slews the indexed base",
        rest_index_x is not None
        and yawed_index_x is not None
        and yawed_index_x < rest_index_x - 0.04,
        details=f"rest_x={rest_index_x}, yawed_x={yawed_index_x}",
    )

    return ctx.report()


object_model = build_object_model()
