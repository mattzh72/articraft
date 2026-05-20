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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _circle_profile(
    radius: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 28,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos((2.0 * math.pi * index) / segments),
            cy + radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="marine_glove_compartment")

    panel_white = model.material("panel_white", rgba=(0.90, 0.92, 0.90, 1.0))
    bezel_grey = model.material("bezel_grey", rgba=(0.72, 0.74, 0.75, 1.0))
    cavity_black = model.material("cavity_black", rgba=(0.10, 0.11, 0.12, 1.0))
    latch_black = model.material("latch_black", rgba=(0.14, 0.14, 0.15, 1.0))
    hardware_dark = model.material("hardware_dark", rgba=(0.22, 0.24, 0.25, 1.0))
    stay_metal = model.material("stay_metal", rgba=(0.68, 0.70, 0.72, 1.0))

    panel_width = 0.520
    panel_height = 0.300
    panel_thickness = 0.008
    opening_width = 0.350
    opening_height = 0.152
    corner_radius = 0.022

    box_depth = 0.182
    shell_wall = 0.006
    shell_outer_width = opening_width
    shell_outer_height = opening_height
    shell_inner_width = shell_outer_width - 2.0 * shell_wall
    shell_inner_height = shell_outer_height - 2.0 * shell_wall

    lid_width = 0.364
    lid_height = 0.164
    lid_thickness = 0.016
    lid_center_y = 0.014
    lid_center_z = -0.077
    latch_center_z = -0.088
    hinge_y = -0.004
    hinge_z = 0.084

    lower_stay_length = 0.044
    upper_stay_length = 0.034
    lower_stay_pitch = 0.37
    upper_stay_pitch = 2.74
    box_stay_y = -0.090
    box_stay_z = -0.010
    stay_x = shell_outer_width * 0.5 - 0.011
    lid_stay_x = stay_x - 0.013
    lid_stay_y = -0.012
    lid_stay_z = -0.096

    housing = model.part("housing")

    panel_profile = rounded_rect_profile(panel_width, panel_height, 0.028)
    opening_profile = rounded_rect_profile(opening_width, opening_height, corner_radius)
    panel_geom = ExtrudeWithHolesGeometry(
        panel_profile,
        [opening_profile],
        height=panel_thickness,
        center=True,
    ).rotate_x(math.pi / 2.0)
    housing.visual(
        mesh_from_geometry(panel_geom, "marine_glove_box_panel"),
        material=bezel_grey,
        name="panel_bezel",
    )

    top_wall_length = box_depth - 0.030
    housing.visual(
        Box((shell_outer_width, top_wall_length, shell_wall)),
        origin=Origin(
            xyz=(
                0.0,
                -box_depth + top_wall_length * 0.5,
                shell_outer_height * 0.5 - shell_wall * 0.5,
            )
        ),
        material=cavity_black,
        name="box_top_wall",
    )
    housing.visual(
        Box((shell_outer_width, box_depth - panel_thickness, shell_wall)),
        origin=Origin(
            xyz=(
                0.0,
                -(box_depth + panel_thickness) * 0.5 + panel_thickness * 0.5,
                -shell_outer_height * 0.5 + shell_wall * 0.5,
            )
        ),
        material=cavity_black,
        name="box_bottom_wall",
    )
    housing.visual(
        Box((shell_wall, box_depth - panel_thickness, shell_inner_height)),
        origin=Origin(
            xyz=(
                -shell_outer_width * 0.5 + shell_wall * 0.5,
                -(box_depth + panel_thickness) * 0.5 + panel_thickness * 0.5,
                0.0,
            )
        ),
        material=cavity_black,
        name="box_left_wall",
    )
    housing.visual(
        Box((shell_wall, box_depth - panel_thickness, shell_inner_height)),
        origin=Origin(
            xyz=(
                shell_outer_width * 0.5 - shell_wall * 0.5,
                -(box_depth + panel_thickness) * 0.5 + panel_thickness * 0.5,
                0.0,
            )
        ),
        material=cavity_black,
        name="box_right_wall",
    )
    housing.visual(
        Box((shell_inner_width, shell_wall, shell_inner_height)),
        origin=Origin(
            xyz=(
                0.0,
                -box_depth + shell_wall * 0.5,
                0.0,
            )
        ),
        material=cavity_black,
        name="box_back_wall",
    )
    housing.visual(
        Box((shell_outer_width + 0.010, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, -0.022, hinge_z + 0.012)),
        material=hardware_dark,
        name="hinge_header",
    )
    housing.visual(
        Box((shell_outer_width + 0.010, 0.018, 0.013)),
        origin=Origin(xyz=(0.0, -0.022, hinge_z - 0.002)),
        material=hardware_dark,
        name="hinge_mount_web",
    )
    housing.visual(
        Box((0.034, 0.014, 0.024)),
        origin=Origin(xyz=(-0.108, -0.022, hinge_z + 0.010)),
        material=hardware_dark,
        name="left_hinge_bracket",
    )
    housing.visual(
        Box((0.034, 0.014, 0.024)),
        origin=Origin(xyz=(0.108, -0.022, hinge_z + 0.010)),
        material=hardware_dark,
        name="right_hinge_bracket",
    )
    housing.visual(
        Box((0.014, 0.012, 0.022)),
        origin=Origin(xyz=(-stay_x, box_stay_y - 0.012, box_stay_z)),
        material=hardware_dark,
        name="left_box_stay_boss",
    )
    housing.visual(
        Box((0.014, 0.012, 0.022)),
        origin=Origin(xyz=(stay_x, box_stay_y - 0.012, box_stay_z)),
        material=hardware_dark,
        name="right_box_stay_boss",
    )
    housing.inertial = Inertial.from_geometry(
        Box((panel_width, box_depth, panel_height)),
        mass=3.6,
        origin=Origin(xyz=(0.0, -box_depth * 0.5, 0.0)),
    )

    lid = model.part("lid")

    latch_hole = _circle_profile(0.0155, center=(0.0, -0.011))
    lid_panel_geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(lid_width, lid_height, 0.024),
        [latch_hole],
        height=lid_thickness,
        center=True,
    ).rotate_x(math.pi / 2.0)
    lid.visual(
        mesh_from_geometry(lid_panel_geom, "marine_glove_box_lid"),
        origin=Origin(xyz=(0.0, lid_center_y, lid_center_z)),
        material=panel_white,
        name="lid_outer_panel",
    )
    lid.visual(
        Box((lid_width - 0.060, 0.020, 0.012)),
        origin=Origin(xyz=(0.0, -0.004, -0.020)),
        material=panel_white,
        name="lid_top_inner_rail",
    )
    lid.visual(
        Box((0.018, 0.028, lid_height - 0.056)),
        origin=Origin(xyz=(-lid_width * 0.5 + 0.019, -0.008, -0.088)),
        material=panel_white,
        name="lid_left_return",
    )
    lid.visual(
        Box((0.018, 0.028, lid_height - 0.056)),
        origin=Origin(xyz=(lid_width * 0.5 - 0.019, -0.008, -0.088)),
        material=panel_white,
        name="lid_right_return",
    )
    lid.visual(
        Box((lid_width - 0.084, 0.028, 0.018)),
        origin=Origin(xyz=(0.0, -0.008, -lid_height + 0.026)),
        material=panel_white,
        name="lid_bottom_return",
    )
    lid.visual(
        Box((0.014, 0.022, 0.050)),
        origin=Origin(xyz=(-0.034, -0.005, -0.092)),
        material=panel_white,
        name="left_latch_reinforcement",
    )
    lid.visual(
        Box((0.014, 0.022, 0.050)),
        origin=Origin(xyz=(0.034, -0.005, -0.092)),
        material=panel_white,
        name="right_latch_reinforcement",
    )
    lid.visual(
        Box((0.028, 0.038, 0.004)),
        origin=Origin(xyz=(0.0, -0.013, latch_center_z + 0.013)),
        material=panel_white,
        name="latch_upper_guide",
    )
    lid.visual(
        Box((0.028, 0.038, 0.004)),
        origin=Origin(xyz=(0.0, -0.013, latch_center_z - 0.013)),
        material=panel_white,
        name="latch_lower_guide",
    )
    lid.visual(
        Cylinder(radius=0.006, length=0.006),
        origin=Origin(
            xyz=(-lid_stay_x - 0.008, lid_stay_y - 0.012, lid_stay_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=hardware_dark,
        name="left_lid_stay_boss",
    )
    lid.visual(
        Cylinder(radius=0.006, length=0.006),
        origin=Origin(
            xyz=(lid_stay_x + 0.008, lid_stay_y - 0.012, lid_stay_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=hardware_dark,
        name="right_lid_stay_boss",
    )
    lid.inertial = Inertial.from_geometry(
        Box((lid_width, 0.036, lid_height)),
        mass=1.2,
        origin=Origin(xyz=(0.0, -0.010, -0.082)),
    )

    latch = model.part("latch")
    latch.visual(
        Cylinder(radius=0.0135, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=latch_black,
        name="latch_button",
    )
    latch.visual(
        Cylinder(radius=0.0095, length=0.032),
        origin=Origin(xyz=(0.0, -0.019, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=latch_black,
        name="latch_slider_stem",
    )
    latch.visual(
        Box((0.024, 0.016, 0.022)),
        origin=Origin(xyz=(0.0, -0.040, 0.0)),
        material=hardware_dark,
        name="latch_rear_body",
    )
    latch.inertial = Inertial.from_geometry(
        Box((0.030, 0.052, 0.026)),
        mass=0.10,
        origin=Origin(xyz=(0.0, -0.024, 0.0)),
    )

    def _build_stay_part(name: str, stay_length: float):
        bar_start = 0.011
        bar_end_inset = 0.005
        bar_length = stay_length - bar_start - bar_end_inset
        stay = model.part(name)
        stay.visual(
            Cylinder(radius=0.005, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=stay_metal,
            name="pivot_barrel",
        )
        stay.visual(
            Box((0.005, 0.012, 0.010)),
            origin=Origin(xyz=(0.0, 0.006, 0.0)),
            material=stay_metal,
            name="pivot_neck",
        )
        stay.visual(
            Box((0.005, bar_length, 0.012)),
            origin=Origin(xyz=(0.0, bar_start + bar_length * 0.5, 0.0)),
            material=stay_metal,
            name="stay_bar",
        )
        stay.visual(
            Cylinder(radius=0.005, length=0.010),
            origin=Origin(
                xyz=(0.0, stay_length, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=stay_metal,
            name="tip_barrel",
        )
        stay.inertial = Inertial.from_geometry(
            Box((0.012, stay_length + 0.006, 0.016)),
            mass=0.05,
            origin=Origin(xyz=(0.0, stay_length * 0.5, 0.0)),
        )
        return stay

    left_lower_stay = _build_stay_part("left_lower_stay", lower_stay_length)
    right_lower_stay = _build_stay_part("right_lower_stay", lower_stay_length)
    left_upper_stay = _build_stay_part("left_upper_stay", upper_stay_length)
    right_upper_stay = _build_stay_part("right_upper_stay", upper_stay_length)

    model.articulation(
        "housing_to_lid",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )
    model.articulation(
        "lid_to_latch",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=latch,
        origin=Origin(xyz=(0.0, 0.023, latch_center_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.05,
            lower=0.0,
            upper=0.006,
        ),
    )
    model.articulation(
        "housing_to_left_lower_stay",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=left_lower_stay,
        origin=Origin(
            xyz=(-stay_x, box_stay_y, box_stay_z),
            rpy=(lower_stay_pitch, 0.0, 0.0),
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=1.0,
            lower=-0.15,
            upper=1.10,
        ),
    )
    model.articulation(
        "housing_to_right_lower_stay",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=right_lower_stay,
        origin=Origin(
            xyz=(stay_x, box_stay_y, box_stay_z),
            rpy=(lower_stay_pitch, 0.0, 0.0),
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=1.0,
            lower=-0.15,
            upper=1.10,
        ),
    )
    model.articulation(
        "lid_to_left_upper_stay",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=left_upper_stay,
        origin=Origin(
            xyz=(-lid_stay_x, lid_stay_y, lid_stay_z),
            rpy=(upper_stay_pitch, 0.0, 0.0),
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=1.0,
            lower=-1.10,
            upper=0.20,
        ),
    )
    model.articulation(
        "lid_to_right_upper_stay",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=right_upper_stay,
        origin=Origin(
            xyz=(lid_stay_x, lid_stay_y, lid_stay_z),
            rpy=(upper_stay_pitch, 0.0, 0.0),
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=1.0,
            lower=-1.10,
            upper=0.20,
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
    lid = object_model.get_part("lid")
    latch = object_model.get_part("latch")
    left_lower_stay = object_model.get_part("left_lower_stay")
    right_lower_stay = object_model.get_part("right_lower_stay")
    left_upper_stay = object_model.get_part("left_upper_stay")
    right_upper_stay = object_model.get_part("right_upper_stay")

    lid_hinge = object_model.get_articulation("housing_to_lid")
    latch_slide = object_model.get_articulation("lid_to_latch")
    left_lower_joint = object_model.get_articulation("housing_to_left_lower_stay")
    right_lower_joint = object_model.get_articulation("housing_to_right_lower_stay")
    left_upper_joint = object_model.get_articulation("lid_to_left_upper_stay")
    right_upper_joint = object_model.get_articulation("lid_to_right_upper_stay")

    ctx.expect_gap(
        lid,
        housing,
        axis="y",
        positive_elem="lid_outer_panel",
        negative_elem="panel_bezel",
        max_gap=0.002,
        max_penetration=0.002,
        name="lid sits flush against the bezel",
    )
    ctx.expect_overlap(
        lid,
        housing,
        axes="xz",
        elem_a="lid_outer_panel",
        elem_b="panel_bezel",
        min_overlap=0.150,
        name="lid covers the recessed opening footprint",
    )

    def _center_from_aabb(aabb):
        if aabb is None:
            return None
        (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
        return (
            (min_x + max_x) * 0.5,
            (min_y + max_y) * 0.5,
            (min_z + max_z) * 0.5,
        )

    closed_lid = _center_from_aabb(ctx.part_element_world_aabb(lid, elem="lid_outer_panel"))
    rest_latch = ctx.part_world_position(latch)
    open_pose = {
        lid_hinge: math.radians(72.0),
        left_lower_joint: 0.55,
        right_lower_joint: 0.55,
        left_upper_joint: -0.82,
        right_upper_joint: -0.82,
    }
    with ctx.pose(open_pose):
        open_lid = _center_from_aabb(ctx.part_element_world_aabb(lid, elem="lid_outer_panel"))
        left_lower_tip_open = _center_from_aabb(
            ctx.part_element_world_aabb(left_lower_stay, elem="tip_barrel")
        )
        right_lower_tip_open = _center_from_aabb(
            ctx.part_element_world_aabb(right_lower_stay, elem="tip_barrel")
        )
        left_upper_tip_open = _center_from_aabb(
            ctx.part_element_world_aabb(left_upper_stay, elem="tip_barrel")
        )
        right_upper_tip_open = _center_from_aabb(
            ctx.part_element_world_aabb(right_upper_stay, elem="tip_barrel")
        )

    left_lower_tip_closed = _center_from_aabb(
        ctx.part_element_world_aabb(left_lower_stay, elem="tip_barrel")
    )
    right_lower_tip_closed = _center_from_aabb(
        ctx.part_element_world_aabb(right_lower_stay, elem="tip_barrel")
    )
    left_upper_tip_closed = _center_from_aabb(
        ctx.part_element_world_aabb(left_upper_stay, elem="tip_barrel")
    )
    right_upper_tip_closed = _center_from_aabb(
        ctx.part_element_world_aabb(right_upper_stay, elem="tip_barrel")
    )

    with ctx.pose({latch_slide: 0.006}):
        pressed_latch = ctx.part_world_position(latch)

    ctx.check(
        "lid opens outward and upward",
        closed_lid is not None
        and open_lid is not None
        and open_lid[1] > closed_lid[1] + 0.050
        and open_lid[2] > closed_lid[2] + 0.020,
        details=f"closed={closed_lid}, open={open_lid}",
    )
    ctx.check(
        "push latch slides inward",
        rest_latch is not None
        and pressed_latch is not None
        and pressed_latch[1] < rest_latch[1] - 0.004,
        details=f"rest={rest_latch}, pressed={pressed_latch}",
    )
    ctx.check(
        "all stay joints rotate about the horizontal hinge axis",
        left_lower_joint.axis == (1.0, 0.0, 0.0)
        and right_lower_joint.axis == (1.0, 0.0, 0.0)
        and left_upper_joint.axis == (1.0, 0.0, 0.0)
        and right_upper_joint.axis == (1.0, 0.0, 0.0),
        details=(
            f"axes="
            f"{left_lower_joint.axis}, {right_lower_joint.axis}, "
            f"{left_upper_joint.axis}, {right_upper_joint.axis}"
        ),
    )
    ctx.check(
        "lower stays lift as the lid opens",
        left_lower_tip_closed is not None
        and right_lower_tip_closed is not None
        and left_lower_tip_open is not None
        and right_lower_tip_open is not None
        and left_lower_tip_open[2] > left_lower_tip_closed[2] + 0.015
        and right_lower_tip_open[2] > right_lower_tip_closed[2] + 0.015,
        details=(
            f"left_closed={left_lower_tip_closed}, left_open={left_lower_tip_open}, "
            f"right_closed={right_lower_tip_closed}, right_open={right_lower_tip_open}"
        ),
    )
    ctx.check(
        "upper stays rise on their lid-side pivots",
        left_upper_tip_closed is not None
        and right_upper_tip_closed is not None
        and left_upper_tip_open is not None
        and right_upper_tip_open is not None
        and left_upper_tip_open[2] > left_upper_tip_closed[2] + 0.030
        and right_upper_tip_open[2] > right_upper_tip_closed[2] + 0.030,
        details=(
            f"left_closed={left_upper_tip_closed}, left_open={left_upper_tip_open}, "
            f"right_closed={right_upper_tip_closed}, right_open={right_upper_tip_open}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
