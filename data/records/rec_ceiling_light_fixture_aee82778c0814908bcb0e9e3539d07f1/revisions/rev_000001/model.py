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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_batwing_strip_light")

    housing_paint = model.material("housing_paint", rgba=(0.42, 0.45, 0.48, 1.0))
    reflector_enamel = model.material("reflector_enamel", rgba=(0.92, 0.93, 0.90, 1.0))
    lamp_glass = model.material("lamp_glass", rgba=(0.96, 0.97, 0.93, 0.92))
    socket_dark = model.material("socket_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.66, 0.68, 0.70, 1.0))

    fixture_length = 1.26
    housing_width = 0.112
    housing_depth = 0.052
    top_thickness = 0.0025
    wall_thickness = 0.0022
    end_thickness = 0.0025
    wall_height = housing_depth - top_thickness

    rail_length = 1.18
    rail_width = 0.036
    rail_depth = 0.016
    socket_length = 0.025
    socket_width = 0.028
    socket_height = 0.018
    lamp_radius = 0.013
    lamp_length = 1.18

    panel_length = 1.22
    panel_depth = 0.140
    panel_thickness = 0.0014
    outer_lip_height = 0.014
    outer_lip_thickness = 0.004
    hinge_leaf_depth = 0.010
    hinge_leaf_height = 0.008
    hinge_barrel_radius = 0.0055
    hinge_axis_y = 0.062475568
    hinge_axis_z = -0.022
    rest_angle = 0.75

    housing = model.part("housing")
    housing.visual(
        Box((fixture_length, housing_width, top_thickness)),
        origin=Origin(xyz=(0.0, 0.0, -top_thickness * 0.5)),
        material=housing_paint,
        name="top_pan",
    )
    housing.visual(
        Box((fixture_length - 0.004, wall_thickness, wall_height)),
        origin=Origin(
            xyz=(
                0.0,
                -housing_width * 0.5 + wall_thickness * 0.5,
                -(top_thickness + wall_height * 0.5),
            )
        ),
        material=housing_paint,
        name="left_sidewall",
    )
    housing.visual(
        Box((fixture_length - 0.004, wall_thickness, wall_height)),
        origin=Origin(
            xyz=(
                0.0,
                housing_width * 0.5 - wall_thickness * 0.5,
                -(top_thickness + wall_height * 0.5),
            )
        ),
        material=housing_paint,
        name="right_sidewall",
    )
    housing.visual(
        Box((end_thickness, housing_width - 2.0 * wall_thickness, wall_height)),
        origin=Origin(
            xyz=(
                -fixture_length * 0.5 + end_thickness * 0.5,
                0.0,
                -(top_thickness + wall_height * 0.5),
            )
        ),
        material=housing_paint,
        name="left_endcap",
    )
    housing.visual(
        Box((end_thickness, housing_width - 2.0 * wall_thickness, wall_height)),
        origin=Origin(
            xyz=(
                fixture_length * 0.5 - end_thickness * 0.5,
                0.0,
                -(top_thickness + wall_height * 0.5),
            )
        ),
        material=housing_paint,
        name="right_endcap",
    )
    housing.visual(
        Box((rail_length, rail_width, rail_depth)),
        origin=Origin(xyz=(0.0, 0.0, -0.0095)),
        material=trim_gray,
        name="lamp_rail",
    )
    housing.visual(
        Box((rail_length, 0.018, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.019)),
        material=trim_gray,
        name="lamp_clip_channel",
    )
    housing.visual(
        Box((socket_length, socket_width, socket_height)),
        origin=Origin(xyz=(-0.5975, 0.0, -0.031)),
        material=socket_dark,
        name="left_socket",
    )
    housing.visual(
        Box((socket_length, socket_width, socket_height)),
        origin=Origin(xyz=(0.5975, 0.0, -0.031)),
        material=socket_dark,
        name="right_socket",
    )
    housing.visual(
        Cylinder(radius=lamp_radius, length=lamp_length),
        origin=Origin(xyz=(0.0, 0.0, -0.033), rpy=(0.0, pi / 2.0, 0.0)),
        material=lamp_glass,
        name="lamp_tube",
    )
    housing.inertial = Inertial.from_geometry(
        Box((fixture_length, 0.34, 0.16)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
    )

    left_panel = model.part("left_reflector_panel")
    left_panel.visual(
        Cylinder(radius=hinge_barrel_radius, length=panel_length),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=trim_gray,
        name="hinge_barrel",
    )
    left_panel.visual(
        Box((panel_length, hinge_leaf_depth, hinge_leaf_height)),
        origin=Origin(xyz=(0.0, -hinge_leaf_depth * 0.5, -hinge_barrel_radius)),
        material=trim_gray,
        name="hinge_leaf",
    )
    left_panel.visual(
        Box((panel_length, panel_depth, panel_thickness)),
        origin=Origin(
            xyz=(0.0, -panel_depth * 0.5, -(hinge_barrel_radius + panel_thickness * 0.5 - 0.0002))
        ),
        material=reflector_enamel,
        name="left_reflector_sheet",
    )
    left_panel.visual(
        Box((panel_length, outer_lip_thickness, outer_lip_height)),
        origin=Origin(
            xyz=(
                0.0,
                -(panel_depth - outer_lip_thickness * 0.5),
                -(hinge_barrel_radius + outer_lip_height * 0.5 - 0.0002),
            )
        ),
        material=reflector_enamel,
        name="left_outer_lip",
    )
    left_panel.inertial = Inertial.from_geometry(
        Box((panel_length, panel_depth, 0.028)),
        mass=1.2,
        origin=Origin(xyz=(0.0, -panel_depth * 0.5, -0.014)),
    )

    right_panel = model.part("right_reflector_panel")
    right_panel.visual(
        Cylinder(radius=hinge_barrel_radius, length=panel_length),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=trim_gray,
        name="hinge_barrel",
    )
    right_panel.visual(
        Box((panel_length, hinge_leaf_depth, hinge_leaf_height)),
        origin=Origin(xyz=(0.0, hinge_leaf_depth * 0.5, -hinge_barrel_radius)),
        material=trim_gray,
        name="hinge_leaf",
    )
    right_panel.visual(
        Box((panel_length, panel_depth, panel_thickness)),
        origin=Origin(
            xyz=(0.0, panel_depth * 0.5, -(hinge_barrel_radius + panel_thickness * 0.5 - 0.0002))
        ),
        material=reflector_enamel,
        name="right_reflector_sheet",
    )
    right_panel.visual(
        Box((panel_length, outer_lip_thickness, outer_lip_height)),
        origin=Origin(
            xyz=(
                0.0,
                panel_depth - outer_lip_thickness * 0.5,
                -(hinge_barrel_radius + outer_lip_height * 0.5 - 0.0002),
            )
        ),
        material=reflector_enamel,
        name="right_outer_lip",
    )
    right_panel.inertial = Inertial.from_geometry(
        Box((panel_length, panel_depth, 0.028)),
        mass=1.2,
        origin=Origin(xyz=(0.0, panel_depth * 0.5, -0.014)),
    )

    model.articulation(
        "housing_to_left_reflector",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=left_panel,
        origin=Origin(xyz=(0.0, -hinge_axis_y, hinge_axis_z), rpy=(rest_angle, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.4, lower=0.0, upper=0.60),
    )
    model.articulation(
        "housing_to_right_reflector",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=right_panel,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z), rpy=(-rest_angle, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.4, lower=0.0, upper=0.60),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    left_panel = object_model.get_part("left_reflector_panel")
    right_panel = object_model.get_part("right_reflector_panel")
    left_joint = object_model.get_articulation("housing_to_left_reflector")
    right_joint = object_model.get_articulation("housing_to_right_reflector")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    with ctx.pose({left_joint: 0.0, right_joint: 0.0}):
        ctx.expect_overlap(
            left_panel,
            housing,
            axes="x",
            min_overlap=1.10,
            name="left reflector runs nearly full fixture length",
        )
        ctx.expect_overlap(
            right_panel,
            housing,
            axes="x",
            min_overlap=1.10,
            name="right reflector runs nearly full fixture length",
        )
        ctx.expect_contact(
            left_panel,
            housing,
            contact_tol=1e-6,
            name="left reflector is mounted on the housing hinge line",
        )
        ctx.expect_contact(
            right_panel,
            housing,
            contact_tol=1e-6,
            name="right reflector is mounted on the housing hinge line",
        )
        ctx.expect_gap(
            housing,
            left_panel,
            axis="y",
            positive_elem="left_sidewall",
            negative_elem="left_reflector_sheet",
            min_gap=0.001,
            max_gap=0.004,
            name="left reflector sheet tucks just below the housing sidewall",
        )
        ctx.expect_gap(
            right_panel,
            housing,
            axis="y",
            positive_elem="right_reflector_sheet",
            negative_elem="right_sidewall",
            min_gap=0.001,
            max_gap=0.004,
            name="right reflector sheet tucks just below the housing sidewall",
        )

    left_upper = left_joint.motion_limits.upper if left_joint.motion_limits is not None else None
    right_upper = right_joint.motion_limits.upper if right_joint.motion_limits is not None else None

    with ctx.pose({left_joint: 0.0, right_joint: 0.0}):
        left_closed = _aabb_center(ctx.part_element_world_aabb(left_panel, elem="left_reflector_sheet"))
        right_closed = _aabb_center(ctx.part_element_world_aabb(right_panel, elem="right_reflector_sheet"))

    with ctx.pose({left_joint: left_upper, right_joint: right_upper}):
        ctx.expect_gap(
            housing,
            left_panel,
            axis="y",
            positive_elem="left_sidewall",
            negative_elem="left_reflector_sheet",
            min_gap=0.005,
            name="left reflector swings wider when opened",
        )
        ctx.expect_gap(
            right_panel,
            housing,
            axis="y",
            positive_elem="right_reflector_sheet",
            negative_elem="right_sidewall",
            min_gap=0.005,
            name="right reflector swings wider when opened",
        )
        left_open = _aabb_center(ctx.part_element_world_aabb(left_panel, elem="left_reflector_sheet"))
        right_open = _aabb_center(ctx.part_element_world_aabb(right_panel, elem="right_reflector_sheet"))

    ctx.check(
        "left reflector opens outward and upward",
        left_closed is not None
        and left_open is not None
        and left_open[1] < left_closed[1] - 0.018
        and left_open[2] > left_closed[2] + 0.025,
        details=f"closed={left_closed}, open={left_open}",
    )
    ctx.check(
        "right reflector opens outward and upward",
        right_closed is not None
        and right_open is not None
        and right_open[1] > right_closed[1] + 0.018
        and right_open[2] > right_closed[2] + 0.025,
        details=f"closed={right_closed}, open={right_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
