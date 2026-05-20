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
    mesh_from_geometry,
    tube_from_spline_points,
    wire_from_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _rect_loop_xy(width: float, depth: float, tube_radius: float, corner_radius: float):
    half_w = width * 0.5
    half_d = depth * 0.5
    return wire_from_points(
        [
            (-half_w, -half_d, 0.0),
            (-half_w, half_d, 0.0),
            (half_w, half_d, 0.0),
            (half_w, -half_d, 0.0),
        ],
        radius=tube_radius,
        closed_path=True,
        corner_mode="fillet",
        corner_radius=corner_radius,
        corner_segments=10,
        radial_segments=18,
    )


def _tube_between(start, end, radius: float):
    return tube_from_spline_points(
        [start, end],
        radius=radius,
        samples_per_segment=2,
        radial_segments=18,
        cap_ends=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="winged_drying_rack")

    painted_steel = model.material("painted_steel", rgba=(0.90, 0.92, 0.94, 1.0))
    cap_gray = model.material("cap_gray", rgba=(0.73, 0.76, 0.79, 1.0))
    hinge_dark = model.material("hinge_dark", rgba=(0.23, 0.25, 0.28, 1.0))
    foot_gray = model.material("foot_gray", rgba=(0.32, 0.35, 0.38, 1.0))

    top_width = 0.76
    top_depth = 0.58
    top_tube = 0.011

    central_frame = model.part("central_frame")
    central_frame.inertial = Inertial.from_geometry(
        Box((0.86, 0.76, 0.92)),
        mass=5.8,
        origin=Origin(xyz=(0.0, -0.02, -0.46)),
    )
    central_frame.visual(
        _save_mesh(
            "central_top_loop",
            _rect_loop_xy(top_width, top_depth, top_tube, corner_radius=0.048),
        ),
        material=painted_steel,
        name="central_top_loop",
    )
    for index, rail_y in enumerate((-0.20, -0.12, -0.04, 0.04, 0.12, 0.20)):
        central_frame.visual(
            Cylinder(radius=0.0048, length=0.746),
            origin=Origin(xyz=(0.0, rail_y, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=painted_steel,
            name=f"central_rail_{index}",
        )
    central_frame.visual(
        Box((0.058, top_depth * 0.94, 0.03)),
        origin=Origin(xyz=(-top_width * 0.5, 0.0, 0.0)),
        material=cap_gray,
        name="left_top_cap",
    )
    central_frame.visual(
        Box((0.058, top_depth * 0.94, 0.03)),
        origin=Origin(xyz=(top_width * 0.5, 0.0, 0.0)),
        material=cap_gray,
        name="right_top_cap",
    )
    central_frame.visual(
        Cylinder(radius=0.012, length=0.16),
        origin=Origin(xyz=(0.0, 0.278, -0.004), rpy=(0.0, pi / 2.0, 0.0)),
        material=hinge_dark,
        name="support_hinge_sleeve",
    )
    central_frame.visual(
        Cylinder(radius=0.009, length=0.08),
        origin=Origin(xyz=(-0.12, 0.284, -0.026), rpy=(0.0, pi / 2.0, 0.0)),
        material=hinge_dark,
        name="support_left_hinge_barrel",
    )
    central_frame.visual(
        Cylinder(radius=0.009, length=0.08),
        origin=Origin(xyz=(0.12, 0.284, -0.026), rpy=(0.0, pi / 2.0, 0.0)),
        material=hinge_dark,
        name="support_right_hinge_barrel",
    )
    central_frame.visual(
        Box((0.042, 0.022, 0.032)),
        origin=Origin(xyz=(-0.12, 0.290, -0.018)),
        material=hinge_dark,
        name="support_left_hinge_web",
    )
    central_frame.visual(
        Box((0.042, 0.022, 0.032)),
        origin=Origin(xyz=(0.12, 0.290, -0.018)),
        material=hinge_dark,
        name="support_right_hinge_web",
    )
    rear_leg_left = _save_mesh(
        "rear_leg_left",
        _tube_between((0.31, -0.278, -0.004), (0.31, -0.34, -0.88), 0.0105),
    )
    rear_leg_right = _save_mesh(
        "rear_leg_right",
        _tube_between((-0.31, -0.278, -0.004), (-0.31, -0.34, -0.88), 0.0105),
    )
    central_frame.visual(rear_leg_left, material=painted_steel, name="rear_leg_left")
    central_frame.visual(rear_leg_right, material=painted_steel, name="rear_leg_right")
    central_frame.visual(
        Cylinder(radius=0.011, length=0.68),
        origin=Origin(xyz=(0.0, -0.34, -0.88), rpy=(0.0, pi / 2.0, 0.0)),
        material=painted_steel,
        name="rear_foot_bar",
    )
    central_frame.visual(
        Cylinder(radius=0.009, length=0.64),
        origin=Origin(xyz=(0.0, -0.300, -0.325), rpy=(0.0, pi / 2.0, 0.0)),
        material=painted_steel,
        name="rear_spreader",
    )
    central_frame.visual(
        Box((0.07, 0.028, 0.02)),
        origin=Origin(xyz=(0.31, -0.34, -0.89)),
        material=foot_gray,
        name="rear_left_foot",
    )
    central_frame.visual(
        Box((0.07, 0.028, 0.02)),
        origin=Origin(xyz=(-0.31, -0.34, -0.89)),
        material=foot_gray,
        name="rear_right_foot",
    )

    wing_width = 0.29
    wing_depth = 0.54
    wing_tube = 0.0095
    wing_hinge_offset = 0.0385
    wing_rail_span = 0.296
    wing_rail_ys = (-0.17, -0.06, 0.06, 0.17)

    left_wing = model.part("left_wing")
    left_wing.inertial = Inertial.from_geometry(
        Box((0.34, 0.58, 0.05)),
        mass=1.25,
        origin=Origin(xyz=(-0.19, 0.0, 0.0)),
    )
    left_wing.visual(
        _save_mesh(
            "left_wing_loop",
            _rect_loop_xy(wing_width, wing_depth, wing_tube, corner_radius=0.036),
        ),
        origin=Origin(xyz=(-(wing_hinge_offset + wing_width * 0.5), 0.0, 0.0)),
        material=painted_steel,
        name="left_wing_loop",
    )
    for index, rail_y in enumerate(wing_rail_ys):
        left_wing.visual(
            Cylinder(radius=0.0042, length=wing_rail_span),
            origin=Origin(
                xyz=(-(wing_hinge_offset + wing_width * 0.5), rail_y, 0.0),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=painted_steel,
            name=f"left_wing_rail_{index}",
        )
    left_wing.visual(
        Box((0.05, wing_depth * 0.90, 0.024)),
        origin=Origin(xyz=(-(wing_hinge_offset + wing_width), 0.0, 0.0)),
        material=cap_gray,
        name="left_outer_cap",
    )

    right_wing = model.part("right_wing")
    right_wing.inertial = Inertial.from_geometry(
        Box((0.34, 0.58, 0.05)),
        mass=1.25,
        origin=Origin(xyz=(0.19, 0.0, 0.0)),
    )
    right_wing.visual(
        _save_mesh(
            "right_wing_loop",
            _rect_loop_xy(wing_width, wing_depth, wing_tube, corner_radius=0.036),
        ),
        origin=Origin(xyz=(wing_hinge_offset + wing_width * 0.5, 0.0, 0.0)),
        material=painted_steel,
        name="right_wing_loop",
    )
    for index, rail_y in enumerate(wing_rail_ys):
        right_wing.visual(
            Cylinder(radius=0.0042, length=wing_rail_span),
            origin=Origin(
                xyz=(wing_hinge_offset + wing_width * 0.5, rail_y, 0.0),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=painted_steel,
            name=f"right_wing_rail_{index}",
        )
    right_wing.visual(
        Box((0.05, wing_depth * 0.90, 0.024)),
        origin=Origin(xyz=(wing_hinge_offset + wing_width, 0.0, 0.0)),
        material=cap_gray,
        name="right_outer_cap",
    )

    support_frame = model.part("lower_support")
    support_frame.inertial = Inertial.from_geometry(
        Box((0.76, 0.36, 0.92)),
        mass=2.35,
        origin=Origin(xyz=(0.0, 0.16, -0.46)),
    )
    support_top_y = 0.012
    support_top_z = -0.028
    support_foot_y = 0.30
    support_foot_z = -0.86
    support_frame.visual(
        Box((0.05, 0.016, 0.026)),
        origin=Origin(xyz=(-0.12, -0.007, -0.030)),
        material=hinge_dark,
        name="support_left_hinge_tab",
    )
    support_frame.visual(
        Box((0.05, 0.016, 0.026)),
        origin=Origin(xyz=(0.12, -0.007, -0.030)),
        material=hinge_dark,
        name="support_right_hinge_tab",
    )
    support_frame.visual(
        _save_mesh(
            "support_upper_brace_left",
            _tube_between((-0.12, -0.010, -0.032), (-0.19, 0.060, -0.180), 0.0095),
        ),
        material=painted_steel,
        name="support_upper_brace_left",
    )
    support_frame.visual(
        _save_mesh(
            "support_upper_brace_right",
            _tube_between((0.12, -0.010, -0.032), (0.19, 0.060, -0.180), 0.0095),
        ),
        material=painted_steel,
        name="support_upper_brace_right",
    )
    support_frame.visual(
        Cylinder(radius=0.009, length=0.44),
        origin=Origin(xyz=(0.0, 0.060, -0.180), rpy=(0.0, pi / 2.0, 0.0)),
        material=painted_steel,
        name="support_top_bar",
    )
    support_frame.visual(
        _save_mesh(
            "support_leg_left",
            _tube_between((-0.19, 0.060, -0.180), (-0.31, support_foot_y, support_foot_z), 0.0105),
        ),
        material=painted_steel,
        name="support_leg_left",
    )
    support_frame.visual(
        _save_mesh(
            "support_leg_right",
            _tube_between((0.19, 0.060, -0.180), (0.31, support_foot_y, support_foot_z), 0.0105),
        ),
        material=painted_steel,
        name="support_leg_right",
    )
    support_frame.visual(
        Cylinder(radius=0.011, length=0.68),
        origin=Origin(xyz=(0.0, support_foot_y, support_foot_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=painted_steel,
        name="support_foot_bar",
    )
    for index, (rail_y, rail_z) in enumerate(((0.11, -0.317), (0.18, -0.520), (0.25, -0.722))):
        support_frame.visual(
            Cylinder(radius=0.0048, length=0.64),
            origin=Origin(xyz=(0.0, rail_y, rail_z), rpy=(0.0, pi / 2.0, 0.0)),
            material=painted_steel,
            name=f"support_rail_{index}",
        )
    support_frame.visual(
        Box((0.07, 0.028, 0.02)),
        origin=Origin(xyz=(0.31, support_foot_y, support_foot_z - 0.01)),
        material=foot_gray,
        name="support_left_foot",
    )
    support_frame.visual(
        Box((0.07, 0.028, 0.02)),
        origin=Origin(xyz=(-0.31, support_foot_y, support_foot_z - 0.01)),
        material=foot_gray,
        name="support_right_foot",
    )

    model.articulation(
        "left_wing_hinge",
        ArticulationType.REVOLUTE,
        parent=central_frame,
        child=left_wing,
        origin=Origin(xyz=(-top_width * 0.5, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.18),
    )
    model.articulation(
        "right_wing_hinge",
        ArticulationType.REVOLUTE,
        parent=central_frame,
        child=right_wing,
        origin=Origin(xyz=(top_width * 0.5, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.18),
    )
    model.articulation(
        "lower_support_hinge",
        ArticulationType.REVOLUTE,
        parent=central_frame,
        child=support_frame,
        origin=Origin(xyz=(0.0, 0.278, -0.004)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.6, lower=0.0, upper=1.08),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    central_frame = object_model.get_part("central_frame")
    left_wing = object_model.get_part("left_wing")
    right_wing = object_model.get_part("right_wing")
    lower_support = object_model.get_part("lower_support")
    left_hinge = object_model.get_articulation("left_wing_hinge")
    right_hinge = object_model.get_articulation("right_wing_hinge")
    support_hinge = object_model.get_articulation("lower_support_hinge")

    ctx.expect_gap(
        central_frame,
        left_wing,
        axis="x",
        positive_elem="left_top_cap",
        negative_elem="left_wing_loop",
        max_gap=0.001,
        max_penetration=0.0,
        name="left wing hinge line stays neatly tucked under the cap",
    )
    ctx.expect_gap(
        right_wing,
        central_frame,
        axis="x",
        positive_elem="right_wing_loop",
        negative_elem="right_top_cap",
        max_gap=0.001,
        max_penetration=0.0,
        name="right wing hinge line stays neatly tucked under the cap",
    )
    ctx.expect_gap(
        lower_support,
        central_frame,
        axis="y",
        positive_elem="support_foot_bar",
        negative_elem="rear_foot_bar",
        min_gap=0.55,
        name="deployed support frame creates a broad front to rear stance",
    )
    ctx.expect_overlap(
        lower_support,
        central_frame,
        axes="x",
        elem_a="support_foot_bar",
        elem_b="rear_foot_bar",
        min_overlap=0.60,
        name="front and rear feet span the same working width",
    )

    right_rest = ctx.part_element_world_aabb(right_wing, elem="right_outer_cap")
    left_rest = ctx.part_element_world_aabb(left_wing, elem="left_outer_cap")
    with ctx.pose({left_hinge: 1.0, right_hinge: 1.0}):
        right_open = ctx.part_element_world_aabb(right_wing, elem="right_outer_cap")
        left_open = ctx.part_element_world_aabb(left_wing, elem="left_outer_cap")
    wings_lift = (
        right_rest is not None
        and right_open is not None
        and left_rest is not None
        and left_open is not None
        and right_open[1][2] > right_rest[1][2] + 0.20
        and left_open[1][2] > left_rest[1][2] + 0.20
    )
    ctx.check(
        "wing frames raise their outer edges for tall garments",
        wings_lift,
        details=(
            f"right_rest={right_rest}, right_open={right_open}, "
            f"left_rest={left_rest}, left_open={left_open}"
        ),
    )

    support_rest = ctx.part_element_world_aabb(lower_support, elem="support_foot_bar")
    with ctx.pose({support_hinge: 0.98}):
        support_folded = ctx.part_element_world_aabb(lower_support, elem="support_foot_bar")
        ctx.expect_gap(
            central_frame,
            lower_support,
            axis="z",
            positive_elem="central_top_loop",
            negative_elem="support_top_bar",
            min_gap=0.005,
            name="folded support frame still clears the rack deck",
        )
    support_folds = (
        support_rest is not None
        and support_folded is not None
        and support_folded[0][1] < support_rest[0][1] - 0.45
        and support_folded[1][2] > support_rest[1][2] + 0.12
    )
    ctx.check(
        "lower support frame folds inward under the main rack",
        support_folds,
        details=f"rest={support_rest}, folded={support_folded}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
