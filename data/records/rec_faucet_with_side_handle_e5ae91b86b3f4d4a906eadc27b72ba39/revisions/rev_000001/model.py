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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="surgical_scrub_faucet")

    brushed_steel = model.material("brushed_steel", rgba=(0.78, 0.80, 0.82, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.66, 0.69, 0.72, 1.0))
    dark_gasket = model.material("dark_gasket", rgba=(0.17, 0.18, 0.19, 1.0))

    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    valve_body = model.part("valve_body")

    body_geom = ExtrudeGeometry(
        rounded_rect_profile(0.095, 0.070, 0.012, corner_segments=8),
        0.400,
        center=True,
    ).rotate_x(math.pi / 2.0)
    valve_body.visual(
        save_mesh("scrub_faucet_body", body_geom),
        material=brushed_steel,
        name="body_shell",
    )

    valve_body.visual(
        Box((0.014, 0.260, 0.098)),
        origin=Origin(xyz=(-0.054, 0.0, 0.0)),
        material=satin_steel,
        name="wall_plate",
    )
    valve_body.visual(
        Box((0.040, 0.070, 0.040)),
        origin=Origin(xyz=(0.056, 0.0, -0.015)),
        material=brushed_steel,
        name="spout_mount",
    )

    spout_geom = tube_from_spline_points(
        [
            (0.046, 0.0, -0.012),
            (0.105, 0.0, -0.015),
            (0.180, 0.0, -0.040),
            (0.265, 0.0, -0.090),
        ],
        radius=0.014,
        samples_per_segment=18,
        radial_segments=20,
        cap_ends=True,
    )
    valve_body.visual(
        save_mesh("scrub_faucet_spout_tube", spout_geom),
        material=brushed_steel,
        name="spout_tube",
    )
    valve_body.visual(
        Cylinder(radius=0.016, length=0.040),
        origin=Origin(xyz=(0.265, 0.0, -0.110)),
        material=brushed_steel,
        name="spout_nozzle",
    )
    valve_body.visual(
        Cylinder(radius=0.0105, length=0.010),
        origin=Origin(xyz=(0.265, 0.0, -0.135)),
        material=dark_gasket,
        name="aerator_tip",
    )

    valve_body.visual(
        Box((0.030, 0.024, 0.044)),
        origin=Origin(xyz=(0.0, 0.212, -0.006)),
        material=satin_steel,
        name="lever_bracket_base",
    )
    valve_body.visual(
        Box((0.006, 0.018, 0.040)),
        origin=Origin(xyz=(-0.010, 0.229, -0.006)),
        material=satin_steel,
        name="lever_bracket_front_ear",
    )
    valve_body.visual(
        Box((0.006, 0.018, 0.040)),
        origin=Origin(xyz=(0.010, 0.229, -0.006)),
        material=satin_steel,
        name="lever_bracket_rear_ear",
    )
    valve_body.inertial = Inertial.from_geometry(
        Box((0.350, 0.460, 0.240)),
        mass=4.8,
        origin=Origin(xyz=(0.110, 0.0, -0.045)),
    )

    knee_lever = model.part("knee_lever")
    knee_lever.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="lever_barrel",
    )
    lever_arm = tube_from_spline_points(
        [
            (0.0, 0.0, 0.0),
            (0.0, 0.018, -0.035),
            (0.0, 0.040, -0.075),
            (0.0, 0.052, -0.098),
        ],
        radius=0.007,
        samples_per_segment=16,
        radial_segments=18,
        cap_ends=True,
    )
    knee_lever.visual(
        save_mesh("scrub_faucet_lever_arm", lever_arm),
        material=satin_steel,
        name="lever_arm",
    )
    knee_lever.visual(
        Box((0.095, 0.016, 0.120)),
        origin=Origin(xyz=(0.0, 0.060, -0.126), rpy=(0.18, 0.0, 0.0)),
        material=brushed_steel,
        name="knee_pad",
    )
    knee_lever.visual(
        Box((0.062, 0.008, 0.086)),
        origin=Origin(xyz=(0.0, 0.067, -0.126), rpy=(0.18, 0.0, 0.0)),
        material=dark_gasket,
        name="knee_pad_grip",
    )
    knee_lever.inertial = Inertial.from_geometry(
        Box((0.100, 0.090, 0.165)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.052, -0.090)),
    )

    model.articulation(
        "body_to_knee_lever",
        ArticulationType.REVOLUTE,
        parent=valve_body,
        child=knee_lever,
        origin=Origin(xyz=(0.0, 0.228, 0.010)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.0,
            lower=-0.20,
            upper=0.70,
        ),
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
    valve_body = object_model.get_part("valve_body")
    knee_lever = object_model.get_part("knee_lever")
    lever_joint = object_model.get_articulation("body_to_knee_lever")
    body_shell = valve_body.get_visual("body_shell")
    spout_tube = valve_body.get_visual("spout_tube")
    knee_pad = knee_lever.get_visual("knee_pad")

    limits = lever_joint.motion_limits
    ctx.check(
        "knee lever rotates about a horizontal side axis",
        lever_joint.axis == (1.0, 0.0, 0.0),
        details=f"axis={lever_joint.axis}",
    )
    ctx.check(
        "knee lever travel is realistic for scrub faucet actuation",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and -0.35 <= limits.lower <= -0.05
        and 0.45 <= limits.upper <= 0.90,
        details=f"limits={limits}",
    )
    ctx.expect_gap(
        valve_body,
        knee_lever,
        axis="z",
        min_gap=0.010,
        max_gap=0.120,
        positive_elem=body_shell,
        negative_elem=knee_pad,
        name="knee paddle hangs below the rectangular valve body",
    )
    ctx.expect_gap(
        knee_lever,
        valve_body,
        axis="y",
        min_gap=0.012,
        max_gap=0.120,
        positive_elem=knee_pad,
        negative_elem=body_shell,
        name="knee paddle sits outboard of the valve body side",
    )

    body_aabb = ctx.part_element_world_aabb(valve_body, elem=body_shell)
    spout_aabb = ctx.part_element_world_aabb(valve_body, elem=spout_tube)
    ctx.check(
        "fixed spout projects forward from the valve body",
        body_aabb is not None
        and spout_aabb is not None
        and spout_aabb[1][0] > body_aabb[1][0] + 0.14
        and spout_aabb[0][2] < body_aabb[0][2] + 0.01,
        details=f"body_aabb={body_aabb}, spout_aabb={spout_aabb}",
    )

    rest_pad_aabb = ctx.part_element_world_aabb(knee_lever, elem=knee_pad)
    with ctx.pose({lever_joint: limits.upper if limits is not None and limits.upper is not None else 0.70}):
        open_pad_aabb = ctx.part_element_world_aabb(knee_lever, elem=knee_pad)
        ctx.expect_gap(
            knee_lever,
            valve_body,
            axis="y",
            min_gap=0.020,
            max_gap=0.170,
            positive_elem=knee_pad,
            negative_elem=body_shell,
            name="opened knee paddle still clears the body side",
        )
    if rest_pad_aabb is not None and open_pad_aabb is not None:
        rest_center_z = 0.5 * (rest_pad_aabb[0][2] + rest_pad_aabb[1][2])
        open_center_z = 0.5 * (open_pad_aabb[0][2] + open_pad_aabb[1][2])
        ctx.check(
            "positive knee lever motion lifts the paddle",
            open_center_z > rest_center_z + 0.035,
            details=f"rest_center_z={rest_center_z}, open_center_z={open_center_z}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
