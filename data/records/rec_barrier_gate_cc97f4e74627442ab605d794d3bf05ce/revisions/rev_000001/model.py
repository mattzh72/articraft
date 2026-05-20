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
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sliding_arm_parking_barrier")

    concrete = model.material("concrete", rgba=(0.70, 0.70, 0.72, 1.0))
    cabinet_yellow = model.material("cabinet_yellow", rgba=(0.88, 0.73, 0.18, 1.0))
    cabinet_dark = model.material("cabinet_dark", rgba=(0.18, 0.20, 0.22, 1.0))
    arm_white = model.material("arm_white", rgba=(0.95, 0.95, 0.94, 1.0))
    flap_red = model.material("flap_red", rgba=(0.76, 0.16, 0.14, 1.0))

    pad_size = (1.20, 0.70, 0.12)
    pad_center = (-0.35, 0.0, pad_size[2] / 2.0)

    cabinet_depth = 0.95
    cabinet_width = 0.34
    cabinet_height = 1.00
    wall_thickness = 0.02
    cabinet_bottom_z = pad_size[2]
    cabinet_top_z = cabinet_bottom_z + cabinet_height
    cabinet_center_x = -cabinet_depth / 2.0
    cabinet_center_z = cabinet_bottom_z + cabinet_height / 2.0

    arm_length = 2.75
    arm_width = 0.085
    arm_height = 0.055
    deployed_insertion = 0.60
    arm_visible_length = arm_length - deployed_insertion
    arm_center_x = (arm_visible_length - deployed_insertion) / 2.0
    slide_travel = 0.25
    arm_axis_z = 0.95

    slot_width = 0.115
    slot_height = 0.075
    slot_bottom_z = arm_axis_z - slot_height / 2.0
    slot_top_z = arm_axis_z + slot_height / 2.0
    side_cheek_width = (cabinet_width - slot_width) / 2.0
    guide_length = 0.10
    guide_thickness = 0.01

    flap_width = 0.10
    flap_height = 0.18
    flap_thickness = 0.015
    hinge_barrel_radius = 0.008

    base = model.part("base")
    base.visual(
        Box(pad_size),
        origin=Origin(xyz=pad_center),
        material=concrete,
        name="pad",
    )
    base.visual(
        Box((cabinet_depth, wall_thickness, cabinet_height)),
        origin=Origin(xyz=(cabinet_center_x, cabinet_width / 2.0 - wall_thickness / 2.0, cabinet_center_z)),
        material=cabinet_yellow,
        name="cabinet_right_wall",
    )
    base.visual(
        Box((cabinet_depth, wall_thickness, cabinet_height)),
        origin=Origin(xyz=(cabinet_center_x, -cabinet_width / 2.0 + wall_thickness / 2.0, cabinet_center_z)),
        material=cabinet_yellow,
        name="cabinet_left_wall",
    )
    base.visual(
        Box((wall_thickness, cabinet_width, cabinet_height)),
        origin=Origin(xyz=(-cabinet_depth + wall_thickness / 2.0, 0.0, cabinet_center_z)),
        material=cabinet_yellow,
        name="cabinet_back_wall",
    )
    base.visual(
        Box((cabinet_depth, cabinet_width, wall_thickness)),
        origin=Origin(xyz=(cabinet_center_x, 0.0, cabinet_bottom_z + wall_thickness / 2.0)),
        material=cabinet_dark,
        name="cabinet_floor",
    )
    base.visual(
        Box((cabinet_depth, cabinet_width, wall_thickness)),
        origin=Origin(xyz=(cabinet_center_x, 0.0, cabinet_top_z - wall_thickness / 2.0)),
        material=cabinet_yellow,
        name="cabinet_roof",
    )
    base.visual(
        Box((wall_thickness, cabinet_width, slot_bottom_z - cabinet_bottom_z)),
        origin=Origin(
            xyz=(
                -wall_thickness / 2.0,
                0.0,
                (cabinet_bottom_z + slot_bottom_z) / 2.0,
            )
        ),
        material=cabinet_yellow,
        name="cabinet_front_lower",
    )
    base.visual(
        Box((wall_thickness, cabinet_width, cabinet_top_z - slot_top_z)),
        origin=Origin(
            xyz=(
                -wall_thickness / 2.0,
                0.0,
                (slot_top_z + cabinet_top_z) / 2.0,
            )
        ),
        material=cabinet_yellow,
        name="cabinet_front_upper",
    )
    base.visual(
        Box((wall_thickness, side_cheek_width, slot_height)),
        origin=Origin(
            xyz=(
                -wall_thickness / 2.0,
                -cabinet_width / 2.0 + side_cheek_width / 2.0,
                arm_axis_z,
            )
        ),
        material=cabinet_yellow,
        name="cabinet_front_left_cheek",
    )
    base.visual(
        Box((wall_thickness, side_cheek_width, slot_height)),
        origin=Origin(
            xyz=(
                -wall_thickness / 2.0,
                cabinet_width / 2.0 - side_cheek_width / 2.0,
                arm_axis_z,
            )
        ),
        material=cabinet_yellow,
        name="cabinet_front_right_cheek",
    )
    base.visual(
        Box((guide_length, arm_width, guide_thickness)),
        origin=Origin(
            xyz=(
                -guide_length / 2.0,
                0.0,
                arm_axis_z + arm_height / 2.0 + guide_thickness / 2.0,
            )
        ),
        material=cabinet_dark,
        name="arm_top_guide",
    )

    arm = model.part("arm")
    arm.visual(
        Box((arm_length, arm_width, arm_height)),
        origin=Origin(xyz=(arm_center_x, 0.0, 0.0)),
        material=arm_white,
        name="arm_beam",
    )
    arm.visual(
        Box((0.01, arm_width + 0.01, arm_height + 0.01)),
        origin=Origin(xyz=(arm_visible_length - 0.005, 0.0, 0.0)),
        material=cabinet_dark,
        name="arm_end_cap",
    )

    flap = model.part("warning_flap")
    flap.visual(
        Box((flap_thickness, flap_width, flap_height)),
        origin=Origin(xyz=(hinge_barrel_radius, 0.0, -flap_height / 2.0)),
        material=flap_red,
        name="flap_panel",
    )
    flap.visual(
        Box((0.02, flap_width + 0.01, 0.02)),
        origin=Origin(xyz=(hinge_barrel_radius + 0.002, 0.0, -0.03)),
        material=cabinet_dark,
        name="flap_weight",
    )
    flap.visual(
        Cylinder(radius=hinge_barrel_radius, length=flap_width),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=cabinet_dark,
        name="flap_hinge_barrel",
    )

    slide = model.articulation(
        "base_to_arm",
        ArticulationType.PRISMATIC,
        parent=base,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, arm_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=400.0,
            velocity=0.25,
            lower=-slide_travel,
            upper=0.0,
        ),
    )
    model.articulation(
        "arm_to_warning_flap",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=flap,
        origin=Origin(
            xyz=(arm_visible_length + hinge_barrel_radius, 0.0, -arm_height / 2.0)
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.0,
            lower=-0.2,
            upper=1.2,
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

    base = object_model.get_part("base")
    arm = object_model.get_part("arm")
    flap = object_model.get_part("warning_flap")
    slide = object_model.get_articulation("base_to_arm")
    flap_hinge = object_model.get_articulation("arm_to_warning_flap")

    def extents(aabb):
        return (
            aabb[1][0] - aabb[0][0],
            aabb[1][1] - aabb[0][1],
            aabb[1][2] - aabb[0][2],
        )

    with ctx.pose({slide: 0.0, flap_hinge: 0.0}):
        ctx.expect_gap(
            arm,
            base,
            axis="z",
            positive_elem="arm_beam",
            negative_elem="cabinet_front_lower",
            min_gap=0.009,
            max_gap=0.011,
            name="arm clears the lower slot lip",
        )
        ctx.expect_gap(
            base,
            arm,
            axis="z",
            positive_elem="cabinet_front_upper",
            negative_elem="arm_beam",
            min_gap=0.009,
            max_gap=0.011,
            name="arm clears the upper slot lip",
        )
        ctx.expect_gap(
            arm,
            base,
            axis="y",
            positive_elem="arm_beam",
            negative_elem="cabinet_front_left_cheek",
            min_gap=0.014,
            max_gap=0.016,
            name="arm clears the left slot cheek",
        )
        ctx.expect_gap(
            base,
            arm,
            axis="y",
            positive_elem="cabinet_front_right_cheek",
            negative_elem="arm_beam",
            min_gap=0.014,
            max_gap=0.016,
            name="arm clears the right slot cheek",
        )
        ctx.expect_gap(
            flap,
            arm,
            axis="x",
            positive_elem="flap_panel",
            negative_elem="arm_beam",
            min_gap=0.008,
            max_gap=0.010,
            name="warning flap hangs just beyond the boom tip",
        )
        ctx.expect_overlap(
            flap,
            arm,
            axes="y",
            elem_a="flap_panel",
            elem_b="arm_beam",
            min_overlap=0.08,
            name="warning flap stays centered across the arm width",
        )

        flap_vertical_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
        flap_barrel_aabb = ctx.part_element_world_aabb(flap, elem="flap_hinge_barrel")

    with ctx.pose({slide: slide.motion_limits.lower, flap_hinge: 0.0}):
        ctx.expect_gap(
            arm,
            base,
            axis="x",
            positive_elem="arm_beam",
            negative_elem="cabinet_back_wall",
            min_gap=0.07,
            max_gap=0.09,
            name="retracted arm still clears the cabinet back wall",
        )
        arm_retracted_position = ctx.part_world_position(arm)

    with ctx.pose({slide: slide.motion_limits.upper, flap_hinge: 1.0}):
        flap_swung_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
        arm_deployed_position = ctx.part_world_position(arm)

    vertical_dims = extents(flap_vertical_aabb) if flap_vertical_aabb is not None else None
    barrel_dims = extents(flap_barrel_aabb) if flap_barrel_aabb is not None else None
    swung_dims = extents(flap_swung_aabb) if flap_swung_aabb is not None else None

    ctx.check(
        "arm extends outward from the cabinet",
        arm_deployed_position is not None
        and arm_retracted_position is not None
        and arm_deployed_position[0] > arm_retracted_position[0] + 0.20,
        details=f"retracted={arm_retracted_position}, deployed={arm_deployed_position}",
    )
    ctx.check(
        "warning flap hangs vertically when deployed",
        vertical_dims is not None
        and barrel_dims is not None
        and vertical_dims[2] > 0.17
        and vertical_dims[0] < 0.03
        and barrel_dims[1] > 0.095,
        details=f"panel_dims={vertical_dims}, barrel_dims={barrel_dims}",
    )
    ctx.check(
        "warning flap pivots on its hinge",
        vertical_dims is not None
        and swung_dims is not None
        and swung_dims[0] > vertical_dims[0] + 0.10
        and swung_dims[2] < vertical_dims[2] - 0.05,
        details=f"vertical={vertical_dims}, swung={swung_dims}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
