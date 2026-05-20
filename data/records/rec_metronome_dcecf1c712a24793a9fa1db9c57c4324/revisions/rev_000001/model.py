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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="box_mechanical_metronome")

    wood = model.material("wood_case", rgba=(0.39, 0.25, 0.14, 1.0))
    brass = model.material("brass", rgba=(0.79, 0.68, 0.32, 1.0))
    steel = model.material("steel", rgba=(0.76, 0.77, 0.79, 1.0))
    dark = model.material("dark_hardware", rgba=(0.12, 0.12, 0.13, 1.0))

    case_width = 0.130
    case_depth = 0.095
    case_height = 0.240
    base_height = 0.016
    wall_height = 0.214
    wall_thickness = 0.006
    top_thickness = 0.010
    front_thickness = 0.004
    half_width = case_width * 0.5
    half_depth = case_depth * 0.5

    housing = model.part("housing")
    housing.visual(
        Box((0.134, 0.100, base_height)),
        origin=Origin(xyz=(0.0, 0.0, base_height * 0.5)),
        material=wood,
        name="base_plinth",
    )
    housing.visual(
        Box((wall_thickness, case_depth, wall_height)),
        origin=Origin(
            xyz=(
                -(half_width - wall_thickness * 0.5),
                0.0,
                base_height + wall_height * 0.5,
            )
        ),
        material=wood,
        name="left_wall",
    )
    housing.visual(
        Box((wall_thickness, case_depth, wall_height)),
        origin=Origin(
            xyz=(
                half_width - wall_thickness * 0.5,
                0.0,
                base_height + wall_height * 0.5,
            )
        ),
        material=wood,
        name="right_wall",
    )
    housing.visual(
        Box((case_width - 2.0 * wall_thickness, 0.005, wall_height)),
        origin=Origin(
            xyz=(
                0.0,
                -(half_depth - 0.0025),
                base_height + wall_height * 0.5,
            )
        ),
        material=wood,
        name="rear_panel",
    )
    housing.visual(
        Box((case_width - 2.0 * wall_thickness, case_depth, top_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                case_height - top_thickness * 0.5,
            )
        ),
        material=wood,
        name="top_cap",
    )

    slot_width = 0.016
    slot_bottom = 0.040
    slot_top = 0.214
    front_face_height = wall_height
    front_left_width = (case_width - slot_width) * 0.5
    front_y = half_depth - front_thickness * 0.5
    housing.visual(
        Box((front_left_width, front_thickness, front_face_height)),
        origin=Origin(
            xyz=(
                -(slot_width * 0.5 + front_left_width * 0.5),
                front_y,
                base_height + front_face_height * 0.5,
            )
        ),
        material=wood,
        name="front_left_face",
    )
    housing.visual(
        Box((front_left_width, front_thickness, front_face_height)),
        origin=Origin(
            xyz=(
                slot_width * 0.5 + front_left_width * 0.5,
                front_y,
                base_height + front_face_height * 0.5,
            )
        ),
        material=wood,
        name="front_right_face",
    )
    housing.visual(
        Box((slot_width, front_thickness, slot_bottom - base_height)),
        origin=Origin(
            xyz=(
                0.0,
                front_y,
                base_height + (slot_bottom - base_height) * 0.5,
            )
        ),
        material=wood,
        name="front_lower_face",
    )
    housing.visual(
        Box((slot_width, front_thickness, case_height - slot_top)),
        origin=Origin(
            xyz=(
                0.0,
                front_y,
                slot_top + (case_height - slot_top) * 0.5,
            )
        ),
        material=wood,
        name="front_upper_face",
    )
    housing.visual(
        Box((case_width - 2.0 * wall_thickness, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, 0.029, 0.206)),
        material=dark,
        name="pivot_crossbeam",
    )
    housing.visual(
        Cylinder(radius=0.0028, length=0.046),
        origin=Origin(
            xyz=(-0.036, 0.040, 0.206),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=steel,
        name="left_pivot_shaft",
    )
    housing.visual(
        Cylinder(radius=0.0028, length=0.046),
        origin=Origin(
            xyz=(0.036, 0.040, 0.206),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=steel,
        name="right_pivot_shaft",
    )

    pendulum = model.part("pendulum")
    pendulum.visual(
        Cylinder(radius=0.0045, length=0.026),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=steel,
        name="pivot_hub",
    )
    pendulum.visual(
        Box((0.010, 0.020, 0.006)),
        origin=Origin(xyz=(0.0, 0.010, -0.003)),
        material=steel,
        name="front_bridge",
    )
    pendulum.visual(
        Box((0.004, 0.002, 0.195)),
        origin=Origin(xyz=(0.0, 0.020, -0.1005)),
        material=steel,
        name="rod",
    )
    pendulum.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(
            xyz=(0.0, 0.020, -0.192),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=brass,
        name="lower_bob",
    )

    slider_weight = model.part("slider_weight")
    slider_weight.visual(
        Box((0.004, 0.010, 0.028)),
        origin=Origin(xyz=(-0.005, 0.0, 0.0)),
        material=brass,
        name="left_cheek",
    )
    slider_weight.visual(
        Box((0.004, 0.010, 0.028)),
        origin=Origin(xyz=(0.005, 0.0, 0.0)),
        material=brass,
        name="right_cheek",
    )
    slider_weight.visual(
        Box((0.018, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, 0.006, 0.006)),
        material=brass,
        name="wedge_upper",
    )
    slider_weight.visual(
        Box((0.013, 0.005, 0.010)),
        origin=Origin(xyz=(0.0, 0.0045, -0.005)),
        material=brass,
        name="wedge_lower",
    )
    slider_weight.visual(
        Box((0.018, 0.004, 0.006)),
        origin=Origin(xyz=(0.0, -0.003, 0.011)),
        material=brass,
        name="top_bridge",
    )

    winding_key = model.part("winding_key")
    winding_key.visual(
        Cylinder(radius=0.004, length=0.012),
        origin=Origin(
            xyz=(0.006, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=steel,
        name="key_spindle",
    )
    winding_key.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(
            xyz=(0.017, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=brass,
        name="key_barrel",
    )
    winding_key.visual(
        Cylinder(radius=0.0022, length=0.024),
        origin=Origin(xyz=(0.017, 0.0, 0.0)),
        material=brass,
        name="key_handle",
    )

    model.articulation(
        "housing_to_pendulum",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=pendulum,
        origin=Origin(xyz=(0.0, 0.040, 0.206)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=2.5,
            lower=-0.42,
            upper=0.42,
        ),
    )
    model.articulation(
        "pendulum_to_slider_weight",
        ArticulationType.PRISMATIC,
        parent=pendulum,
        child=slider_weight,
        origin=Origin(xyz=(0.0, 0.020, -0.108)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.10,
            lower=-0.050,
            upper=0.040,
        ),
    )
    model.articulation(
        "housing_to_winding_key",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=winding_key,
        origin=Origin(xyz=(0.065, 0.0, 0.082)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=12.0,
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

    housing = object_model.get_part("housing")
    pendulum = object_model.get_part("pendulum")
    slider_weight = object_model.get_part("slider_weight")
    winding_key = object_model.get_part("winding_key")

    pendulum_joint = object_model.get_articulation("housing_to_pendulum")
    slider_joint = object_model.get_articulation("pendulum_to_slider_weight")
    key_joint = object_model.get_articulation("housing_to_winding_key")

    rod = pendulum.get_visual("rod")
    spindle = winding_key.get_visual("key_spindle")
    right_wall = housing.get_visual("right_wall")

    with ctx.pose({pendulum_joint: 0.0, slider_joint: 0.0}):
        ctx.expect_gap(
            pendulum,
            housing,
            axis="y",
            positive_elem=rod,
            max_gap=0.012,
            max_penetration=0.0,
            name="pendulum rod hangs just proud of the front face",
        )
        ctx.expect_gap(
            winding_key,
            housing,
            axis="x",
            positive_elem=spindle,
            negative_elem=right_wall,
            max_gap=0.001,
            max_penetration=0.0,
            name="winding key spindle emerges flush from the right side face",
        )

    with ctx.pose({pendulum_joint: -0.30}):
        left_pose = ctx.part_world_aabb(pendulum)
    with ctx.pose({pendulum_joint: 0.30}):
        right_pose = ctx.part_world_aabb(pendulum)

    left_center_x = None
    right_center_x = None
    if left_pose is not None:
        left_center_x = (left_pose[0][0] + left_pose[1][0]) * 0.5
    if right_pose is not None:
        right_center_x = (right_pose[0][0] + right_pose[1][0]) * 0.5
    ctx.check(
        "pendulum sweeps side to side about its shaft",
        left_center_x is not None
        and right_center_x is not None
        and right_center_x > left_center_x + 0.04,
        details=f"left_center_x={left_center_x}, right_center_x={right_center_x}",
    )

    lower_limit = slider_joint.motion_limits.lower if slider_joint.motion_limits is not None else None
    upper_limit = slider_joint.motion_limits.upper if slider_joint.motion_limits is not None else None
    with ctx.pose({pendulum_joint: 0.0, slider_joint: lower_limit}):
        low_pos = ctx.part_world_position(slider_weight)
    with ctx.pose({pendulum_joint: 0.0, slider_joint: upper_limit}):
        high_pos = ctx.part_world_position(slider_weight)
    ctx.check(
        "slider weight travels upward along the pendulum rod",
        low_pos is not None
        and high_pos is not None
        and high_pos[2] > low_pos[2] + 0.08
        and abs(high_pos[0] - low_pos[0]) < 0.002
        and abs(high_pos[1] - low_pos[1]) < 0.002,
        details=f"low_pos={low_pos}, high_pos={high_pos}",
    )

    ctx.check(
        "winding key uses a continuous right-side rotation axis",
        tuple(round(value, 4) for value in key_joint.axis) == (1.0, 0.0, 0.0)
        and key_joint.motion_limits is not None
        and key_joint.motion_limits.lower is None
        and key_joint.motion_limits.upper is None,
        details=f"axis={key_joint.axis}, limits={key_joint.motion_limits}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
