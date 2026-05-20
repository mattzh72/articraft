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
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="floor_pendulum_metronome")

    wood = model.material("walnut_case", rgba=(0.36, 0.24, 0.14, 1.0))
    wood_dark = model.material("walnut_base", rgba=(0.26, 0.17, 0.10, 1.0))
    brass = model.material("aged_brass", rgba=(0.77, 0.66, 0.32, 1.0))
    steel = model.material("darkened_steel", rgba=(0.20, 0.20, 0.22, 1.0))

    case_width = 0.24
    case_depth = 0.18
    case_height = 1.02
    plinth_width = 0.38
    plinth_depth = 0.28
    plinth_height = 0.06
    wall = 0.015
    front_opening_bottom = 0.16
    front_opening_top = 0.98
    pivot_height = 0.19
    pendulum_front_offset = 0.11
    rod_length = 1.50

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((plinth_width, plinth_depth, plinth_height)),
        origin=Origin(xyz=(0.0, 0.0, plinth_height * 0.5)),
        material=wood_dark,
        name="base_plinth",
    )
    cabinet.visual(
        Box((case_width, case_depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, plinth_height + wall * 0.5)),
        material=wood,
        name="case_floor",
    )
    cabinet.visual(
        Box((wall, case_depth, case_height)),
        origin=Origin(
            xyz=(-(case_width * 0.5) + wall * 0.5, 0.0, plinth_height + case_height * 0.5)
        ),
        material=wood,
        name="left_wall",
    )
    cabinet.visual(
        Box((wall, case_depth, case_height)),
        origin=Origin(
            xyz=((case_width * 0.5) - wall * 0.5, 0.0, plinth_height + case_height * 0.5)
        ),
        material=wood,
        name="right_wall",
    )
    cabinet.visual(
        Box((case_width - 2.0 * wall, wall, case_height)),
        origin=Origin(
            xyz=(0.0, -(case_depth * 0.5) + wall * 0.5, plinth_height + case_height * 0.5)
        ),
        material=wood,
        name="back_wall",
    )
    cabinet.visual(
        Box((case_width - 2.0 * wall, case_depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, plinth_height + case_height - wall * 0.5)),
        material=wood,
        name="roof_panel",
    )
    cabinet.visual(
        Box((case_width - 2.0 * wall, wall, front_opening_bottom - plinth_height)),
        origin=Origin(
            xyz=(
                0.0,
                (case_depth * 0.5) - wall * 0.5,
                plinth_height + (front_opening_bottom - plinth_height) * 0.5,
            )
        ),
        material=wood,
        name="front_lower_apron",
    )
    cabinet.visual(
        Box((case_width - 2.0 * wall, wall, plinth_height + case_height - front_opening_top)),
        origin=Origin(
            xyz=(
                0.0,
                (case_depth * 0.5) - wall * 0.5,
                front_opening_top
                + (plinth_height + case_height - front_opening_top) * 0.5,
            )
        ),
        material=wood,
        name="front_upper_lintel",
    )
    cabinet.visual(
        Box((0.04, wall, front_opening_top - front_opening_bottom)),
        origin=Origin(
            xyz=(
                -0.0825,
                (case_depth * 0.5) - wall * 0.5,
                (front_opening_bottom + front_opening_top) * 0.5,
            )
        ),
        material=wood,
        name="front_left_jamb",
    )
    cabinet.visual(
        Box((0.04, wall, front_opening_top - front_opening_bottom)),
        origin=Origin(
            xyz=(
                0.0825,
                (case_depth * 0.5) - wall * 0.5,
                (front_opening_bottom + front_opening_top) * 0.5,
            )
        ),
        material=wood,
        name="front_right_jamb",
    )
    cabinet.visual(
        Box((0.016, 0.06, 0.145)),
        origin=Origin(xyz=(-0.022, 0.0, plinth_height + 0.0825)),
        material=wood_dark,
        name="left_bearing_pedestal",
    )
    cabinet.visual(
        Box((0.016, 0.06, 0.145)),
        origin=Origin(xyz=(0.022, 0.0, plinth_height + 0.0825)),
        material=wood_dark,
        name="right_bearing_pedestal",
    )
    cabinet.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(
            xyz=((case_width * 0.5), 0.03, 0.30),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=brass,
        name="key_bushing",
    )

    pendulum = model.part("pendulum")
    pendulum.visual(
        Cylinder(radius=0.014, length=0.06),
        origin=Origin(rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=steel,
        name="pivot_hub",
    )
    pendulum.visual(
        Box((0.016, 0.08, 0.016)),
        origin=Origin(xyz=(0.0, 0.07, 0.0)),
        material=steel,
        name="pendulum_arm",
    )
    pendulum.visual(
        Cylinder(radius=0.010, length=0.06),
        origin=Origin(xyz=(0.0, pendulum_front_offset, 0.03)),
        material=brass,
        name="rod_collar",
    )
    pendulum.visual(
        Cylinder(radius=0.006, length=rod_length),
        origin=Origin(xyz=(0.0, pendulum_front_offset, rod_length * 0.5)),
        material=steel,
        name="pendulum_rod",
    )
    pendulum.visual(
        Sphere(radius=0.012),
        origin=Origin(xyz=(0.0, pendulum_front_offset, rod_length + 0.012)),
        material=brass,
        name="rod_tip",
    )

    sliding_weight = model.part("sliding_weight")
    sliding_weight.visual(
        Box((0.018, 0.012, 0.05)),
        origin=Origin(xyz=(0.0, 0.012, 0.0)),
        material=steel,
        name="weight_clamp",
    )
    sliding_weight.visual(
        Cylinder(radius=0.045, length=0.06),
        origin=Origin(xyz=(0.0, 0.062, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=brass,
        name="weight_body",
    )

    winding_key = model.part("winding_key")
    winding_key.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=(0.003, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=brass,
        name="key_escutcheon",
    )
    winding_key.visual(
        Cylinder(radius=0.006, length=0.05),
        origin=Origin(xyz=(0.025, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=steel,
        name="key_stem",
    )
    winding_key.visual(
        Sphere(radius=0.010),
        origin=Origin(xyz=(0.058, 0.0, 0.0)),
        material=brass,
        name="key_boss",
    )
    winding_key.visual(
        Cylinder(radius=0.005, length=0.084),
        origin=Origin(xyz=(0.058, 0.0, 0.0)),
        material=brass,
        name="key_bar",
    )
    winding_key.visual(
        Sphere(radius=0.008),
        origin=Origin(xyz=(0.058, 0.0, 0.042)),
        material=brass,
        name="key_grip_top",
    )
    winding_key.visual(
        Sphere(radius=0.008),
        origin=Origin(xyz=(0.058, 0.0, -0.042)),
        material=brass,
        name="key_grip_bottom",
    )

    model.articulation(
        "cabinet_to_pendulum",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=pendulum,
        origin=Origin(xyz=(0.0, 0.0, pivot_height)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.2,
            lower=-0.24,
            upper=0.24,
        ),
    )
    model.articulation(
        "pendulum_to_sliding_weight",
        ArticulationType.PRISMATIC,
        parent=pendulum,
        child=sliding_weight,
        origin=Origin(xyz=(0.0, pendulum_front_offset, 1.05)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.25,
            lower=-0.60,
            upper=0.10,
        ),
    )
    model.articulation(
        "cabinet_to_winding_key",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=winding_key,
        origin=Origin(xyz=((case_width * 0.5), 0.03, 0.30)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=10.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    def aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))

    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    pendulum = object_model.get_part("pendulum")
    sliding_weight = object_model.get_part("sliding_weight")
    winding_key = object_model.get_part("winding_key")

    pendulum_joint = object_model.get_articulation("cabinet_to_pendulum")
    weight_joint = object_model.get_articulation("pendulum_to_sliding_weight")
    key_joint = object_model.get_articulation("cabinet_to_winding_key")

    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    ctx.check(
        "pendulum articulation is revolute about the cabinet's y-axis",
        pendulum_joint.joint_type == ArticulationType.REVOLUTE
        and tuple(pendulum_joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={pendulum_joint.joint_type}, axis={pendulum_joint.axis}",
    )
    ctx.check(
        "sliding weight articulation is prismatic along the rod",
        weight_joint.joint_type == ArticulationType.PRISMATIC
        and tuple(weight_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={weight_joint.joint_type}, axis={weight_joint.axis}",
    )
    ctx.check(
        "winding key articulation is continuous about the side-mounted x-axis",
        key_joint.joint_type == ArticulationType.CONTINUOUS
        and tuple(key_joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={key_joint.joint_type}, axis={key_joint.axis}",
    )

    ctx.expect_contact(
        pendulum,
        cabinet,
        elem_a="pivot_hub",
        elem_b="left_bearing_pedestal",
        contact_tol=0.0005,
        name="pendulum hub bears on the left internal support",
    )
    ctx.expect_contact(
        pendulum,
        cabinet,
        elem_a="pivot_hub",
        elem_b="right_bearing_pedestal",
        contact_tol=0.0005,
        name="pendulum hub bears on the right internal support",
    )
    ctx.expect_contact(
        winding_key,
        cabinet,
        elem_a="key_escutcheon",
        elem_b="key_bushing",
        contact_tol=0.0005,
        name="winding key seats against the cabinet bushing",
    )
    ctx.expect_gap(
        pendulum,
        cabinet,
        axis="z",
        positive_elem="rod_tip",
        min_gap=0.50,
        name="pendulum rod rises well above the housing top",
    )

    rest_tip = aabb_center(ctx.part_element_world_aabb(pendulum, elem="rod_tip"))
    with ctx.pose({pendulum_joint: pendulum_joint.motion_limits.upper}):
        swung_tip = aabb_center(ctx.part_element_world_aabb(pendulum, elem="rod_tip"))
    ctx.check(
        "positive pendulum motion swings the rod tip to the right",
        rest_tip is not None and swung_tip is not None and swung_tip[0] > rest_tip[0] + 0.10,
        details=f"rest_tip={rest_tip}, swung_tip={swung_tip}",
    )

    with ctx.pose({weight_joint: weight_joint.motion_limits.lower}):
        weight_low = ctx.part_world_position(sliding_weight)
    with ctx.pose({weight_joint: weight_joint.motion_limits.upper}):
        weight_high = ctx.part_world_position(sliding_weight)
    ctx.check(
        "positive slider travel raises the pendulum weight",
        weight_low is not None and weight_high is not None and weight_high[2] > weight_low[2] + 0.50,
        details=f"low={weight_low}, high={weight_high}",
    )

    rest_grip = aabb_center(ctx.part_element_world_aabb(winding_key, elem="key_grip_top"))
    with ctx.pose({key_joint: math.pi * 0.5}):
        turned_grip = aabb_center(ctx.part_element_world_aabb(winding_key, elem="key_grip_top"))
    ctx.check(
        "winding key actually rotates around its stem axis",
        rest_grip is not None
        and turned_grip is not None
        and abs(turned_grip[1] - rest_grip[1]) > 0.03,
        details=f"rest_grip={rest_grip}, turned_grip={turned_grip}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
