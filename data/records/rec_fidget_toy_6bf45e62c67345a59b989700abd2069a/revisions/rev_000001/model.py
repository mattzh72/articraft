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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="click_slider_fidget")

    housing_length = 0.092
    housing_width = 0.038
    housing_height = 0.011
    body_radius = 0.0055
    floor_thickness = 0.0032
    wall_height = 0.0060
    top_thickness = housing_height - floor_thickness - wall_height
    cavity_width = 0.021
    slider_travel = 0.022

    shell_color = model.material("shell_color", rgba=(0.14, 0.15, 0.17, 1.0))
    frame_color = model.material("frame_color", rgba=(0.20, 0.21, 0.24, 1.0))
    slider_color = model.material("slider_color", rgba=(0.93, 0.43, 0.16, 1.0))
    detent_color = model.material("detent_color", rgba=(0.62, 0.65, 0.69, 1.0))
    grip_color = model.material("grip_color", rgba=(0.98, 0.58, 0.22, 1.0))

    outer_profile = rounded_rect_profile(housing_length, housing_width, body_radius)
    slot_profile = rounded_rect_profile(0.056, 0.012, 0.003)

    base_mesh = mesh_from_geometry(
        ExtrudeGeometry(outer_profile, floor_thickness, center=True),
        "housing_base",
    )
    top_frame_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            outer_profile,
            [slot_profile],
            top_thickness,
            center=True,
        ),
        "housing_top_frame",
    )
    thumb_pad_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.020, 0.015, 0.003),
            0.0030,
            center=True,
        ),
        "thumb_pad",
    )

    housing = model.part("housing")
    housing.visual(
        base_mesh,
        origin=Origin(xyz=(0.0, 0.0, floor_thickness * 0.5)),
        material=shell_color,
        name="body_base",
    )
    side_rail_length = 0.080
    side_wall_thickness = (housing_width - cavity_width) * 0.5
    rail_center_y = cavity_width * 0.5 + side_wall_thickness * 0.5
    for side_name, side_y in (("left", rail_center_y), ("right", -rail_center_y)):
        housing.visual(
            Box((side_rail_length, side_wall_thickness, wall_height)),
            origin=Origin(
                xyz=(0.0, side_y, floor_thickness + wall_height * 0.5),
            ),
            material=frame_color,
            name=f"{side_name}_rail",
        )

    for end_name, end_x in (("left", -0.0335), ("right", 0.0335)):
        housing.visual(
            Box((0.009, 0.020, 0.0020)),
            origin=Origin(xyz=(end_x, 0.0, 0.0042)),
            material=frame_color,
            name=f"{end_name}_detent_seat",
        )
        for boss_name, boss_y in (("upper", 0.0085), ("lower", -0.0085)):
            housing.visual(
                Box((0.004, 0.0020, 0.0036)),
                origin=Origin(xyz=(end_x, boss_y, 0.0060)),
                material=frame_color,
                name=f"{end_name}_detent_{boss_name}_boss",
            )

    housing.visual(
        top_frame_mesh,
        origin=Origin(xyz=(0.0, 0.0, floor_thickness + wall_height + top_thickness * 0.5)),
        material=shell_color,
        name="top_frame",
    )
    housing.inertial = Inertial.from_geometry(
        Box((housing_length, housing_width, housing_height)),
        mass=0.14,
        origin=Origin(xyz=(0.0, 0.0, housing_height * 0.5)),
    )

    slider = model.part("thumb_slider")
    slider.visual(
        Box((0.026, 0.016, 0.0036)),
        origin=Origin(xyz=(0.0, 0.0, 0.00535)),
        material=slider_color,
        name="slider_sled",
    )
    slider.visual(
        Box((0.008, 0.0062, 0.00415)),
        origin=Origin(xyz=(0.0, 0.0, 0.009225)),
        material=slider_color,
        name="slider_stem",
    )
    slider.visual(
        thumb_pad_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=slider_color,
        name="thumb_pad",
    )
    for ridge_x, ridge_name in ((-0.0044, "left"), (0.0, "center"), (0.0044, "right")):
        slider.visual(
            Box((0.0022, 0.0110, 0.0008)),
            origin=Origin(xyz=(ridge_x, 0.0, 0.0144)),
            material=grip_color,
            name=f"thumb_ridge_{ridge_name}",
        )
    slider.inertial = Inertial.from_geometry(
        Box((0.026, 0.020, 0.0130)),
        mass=0.028,
        origin=Origin(xyz=(0.0, 0.0, 0.0072)),
    )

    model.articulation(
        "housing_to_slider",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=slider,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.18,
            lower=-slider_travel,
            upper=slider_travel,
        ),
    )

    left_detent = model.part("left_detent")
    left_detent.visual(
        Cylinder(radius=0.00135, length=0.012),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=detent_color,
        name="pivot_barrel",
    )
    left_detent.visual(
        Box((0.0085, 0.0062, 0.0016)),
        origin=Origin(xyz=(0.00425, 0.0, 0.0008)),
        material=detent_color,
        name="detent_arm",
    )
    left_detent.visual(
        Box((0.0048, 0.0048, 0.0022)),
        origin=Origin(xyz=(0.0093, 0.0, 0.0019)),
        material=detent_color,
        name="detent_tooth",
    )
    left_detent.inertial = Inertial.from_geometry(
        Box((0.014, 0.012, 0.0045)),
        mass=0.004,
        origin=Origin(xyz=(0.005, 0.0, 0.0012)),
    )

    model.articulation(
        "housing_to_left_detent",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=left_detent,
        origin=Origin(xyz=(-0.0335, 0.0, 0.0056)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.35,
            velocity=6.0,
            lower=0.0,
            upper=0.42,
        ),
    )

    right_detent = model.part("right_detent")
    right_detent.visual(
        Cylinder(radius=0.00135, length=0.012),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=detent_color,
        name="pivot_barrel",
    )
    right_detent.visual(
        Box((0.0085, 0.0062, 0.0016)),
        origin=Origin(xyz=(-0.00425, 0.0, 0.0008)),
        material=detent_color,
        name="detent_arm",
    )
    right_detent.visual(
        Box((0.0048, 0.0048, 0.0022)),
        origin=Origin(xyz=(-0.0093, 0.0, 0.0019)),
        material=detent_color,
        name="detent_tooth",
    )
    right_detent.inertial = Inertial.from_geometry(
        Box((0.014, 0.012, 0.0045)),
        mass=0.004,
        origin=Origin(xyz=(-0.005, 0.0, 0.0012)),
    )

    model.articulation(
        "housing_to_right_detent",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=right_detent,
        origin=Origin(xyz=(0.0335, 0.0, 0.0056)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.35,
            velocity=6.0,
            lower=0.0,
            upper=0.42,
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
    slider = object_model.get_part("thumb_slider")
    left_detent = object_model.get_part("left_detent")
    right_detent = object_model.get_part("right_detent")

    slide_joint = object_model.get_articulation("housing_to_slider")
    left_joint = object_model.get_articulation("housing_to_left_detent")
    right_joint = object_model.get_articulation("housing_to_right_detent")

    ctx.expect_gap(
        slider,
        housing,
        axis="z",
        positive_elem="thumb_pad",
        negative_elem="top_frame",
        min_gap=0.0,
        max_gap=0.0002,
        name="thumb pad rides directly on the housing top",
    )
    ctx.expect_gap(
        housing,
        slider,
        axis="z",
        positive_elem="top_frame",
        negative_elem="slider_sled",
        min_gap=0.0015,
        max_gap=0.0030,
        name="slider sled stays below the top frame",
    )
    ctx.expect_gap(
        slider,
        housing,
        axis="z",
        positive_elem="slider_sled",
        negative_elem="body_base",
        min_gap=0.0001,
        max_gap=0.0010,
        name="slider sled clears the housing floor",
    )

    lower = slide_joint.motion_limits.lower if slide_joint.motion_limits is not None else None
    upper = slide_joint.motion_limits.upper if slide_joint.motion_limits is not None else None
    left_pose = None
    right_pose = None
    if lower is not None:
        with ctx.pose({slide_joint: lower}):
            ctx.expect_within(
                slider,
                housing,
                axes="yz",
                inner_elem="slider_sled",
                margin=0.0,
                name="slider stays inside the housing section at the left end",
            )
            ctx.expect_overlap(
                slider,
                housing,
                axes="x",
                elem_a="slider_sled",
                min_overlap=0.018,
                name="slider remains retained inside the housing at the left end",
            )
            left_pose = ctx.part_world_position(slider)
    if upper is not None:
        with ctx.pose({slide_joint: upper}):
            ctx.expect_within(
                slider,
                housing,
                axes="yz",
                inner_elem="slider_sled",
                margin=0.0,
                name="slider stays inside the housing section at the right end",
            )
            ctx.expect_overlap(
                slider,
                housing,
                axes="x",
                elem_a="slider_sled",
                min_overlap=0.018,
                name="slider remains retained inside the housing at the right end",
            )
            right_pose = ctx.part_world_position(slider)

    ctx.check(
        "slider travels along the housing long axis",
        left_pose is not None
        and right_pose is not None
        and right_pose[0] > left_pose[0] + 0.040
        and abs(right_pose[1] - left_pose[1]) < 1e-6
        and abs(right_pose[2] - left_pose[2]) < 1e-6,
        details=f"left_pose={left_pose}, right_pose={right_pose}",
    )

    ctx.check(
        "left detent is a short revolute end detent",
        left_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(left_joint.axis) == (0.0, -1.0, 0.0)
        and left_joint.motion_limits is not None
        and left_joint.motion_limits.lower == 0.0
        and left_joint.motion_limits.upper is not None
        and left_joint.motion_limits.upper <= 0.5,
        details=f"axis={left_joint.axis}, limits={left_joint.motion_limits}",
    )
    ctx.check(
        "right detent is a short revolute end detent",
        right_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(right_joint.axis) == (0.0, 1.0, 0.0)
        and right_joint.motion_limits is not None
        and right_joint.motion_limits.lower == 0.0
        and right_joint.motion_limits.upper is not None
        and right_joint.motion_limits.upper <= 0.5,
        details=f"axis={right_joint.axis}, limits={right_joint.motion_limits}",
    )

    left_detent_pos = ctx.part_world_position(left_detent)
    right_detent_pos = ctx.part_world_position(right_detent)
    ctx.check(
        "detents sit at opposite housing ends",
        left_detent_pos is not None
        and right_detent_pos is not None
        and left_detent_pos[0] < -0.025
        and right_detent_pos[0] > 0.025,
        details=f"left_detent_pos={left_detent_pos}, right_detent_pos={right_detent_pos}",
    )

    left_closed_tooth = ctx.part_element_world_aabb(left_detent, elem="detent_tooth")
    left_open_tooth = None
    if left_joint.motion_limits is not None and left_joint.motion_limits.upper is not None:
        with ctx.pose({left_joint: left_joint.motion_limits.upper}):
            left_open_tooth = ctx.part_element_world_aabb(left_detent, elem="detent_tooth")
    ctx.check(
        "left detent tooth lifts out of the track when opened",
        left_closed_tooth is not None
        and left_open_tooth is not None
        and left_open_tooth[1][2] > left_closed_tooth[1][2] + 0.003,
        details=f"left_closed_tooth={left_closed_tooth}, left_open_tooth={left_open_tooth}",
    )

    right_closed_tooth = ctx.part_element_world_aabb(right_detent, elem="detent_tooth")
    right_open_tooth = None
    if right_joint.motion_limits is not None and right_joint.motion_limits.upper is not None:
        with ctx.pose({right_joint: right_joint.motion_limits.upper}):
            right_open_tooth = ctx.part_element_world_aabb(right_detent, elem="detent_tooth")
    ctx.check(
        "right detent tooth lifts out of the track when opened",
        right_closed_tooth is not None
        and right_open_tooth is not None
        and right_open_tooth[1][2] > right_closed_tooth[1][2] + 0.003,
        details=f"right_closed_tooth={right_closed_tooth}, right_open_tooth={right_open_tooth}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
