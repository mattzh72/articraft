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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="watch_winder_presentation_box")

    wood = model.material("wood", rgba=(0.30, 0.18, 0.10, 1.0))
    lining = model.material("lining", rgba=(0.17, 0.14, 0.12, 1.0))
    metal = model.material("metal", rgba=(0.74, 0.76, 0.78, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.25, 0.28, 0.32, 0.50))

    outer_w = 0.190
    outer_d = 0.170
    body_h = 0.110
    wall_t = 0.018
    floor_t = 0.018
    lid_t = 0.022
    hinge_r = 0.007
    hinge_axis_y = -(outer_d / 2.0 + 0.003)
    hinge_axis_z = body_h - hinge_r

    body = model.part("body")
    body.visual(
        Box((outer_w, outer_d, floor_t)),
        origin=Origin(xyz=(0.0, 0.0, floor_t / 2.0)),
        material=wood,
        name="floor",
    )
    body.visual(
        Box((wall_t, outer_d, body_h)),
        origin=Origin(xyz=(-(outer_w / 2.0) + wall_t / 2.0, 0.0, body_h / 2.0)),
        material=wood,
        name="left_wall",
    )
    body.visual(
        Box((wall_t, outer_d, body_h)),
        origin=Origin(xyz=((outer_w / 2.0) - wall_t / 2.0, 0.0, body_h / 2.0)),
        material=wood,
        name="right_wall",
    )
    inner_w = outer_w - 2.0 * wall_t
    body.visual(
        Box((inner_w, wall_t, body_h)),
        origin=Origin(xyz=(0.0, (outer_d / 2.0) - wall_t / 2.0, body_h / 2.0)),
        material=wood,
        name="front_wall",
    )
    body.visual(
        Box((inner_w, wall_t, body_h)),
        origin=Origin(xyz=(0.0, -(outer_d / 2.0) + wall_t / 2.0, body_h / 2.0)),
        material=wood,
        name="rear_wall",
    )
    body.visual(
        Box((inner_w - 0.010, outer_d - 2.0 * wall_t, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, floor_t + 0.002)),
        material=lining,
        name="base_liner",
    )
    body.visual(
        Box((0.094, 0.018, 0.060)),
        origin=Origin(
            xyz=(0.0, -(outer_d / 2.0) + wall_t + 0.009, 0.058),
        ),
        material=wood,
        name="motor_housing",
    )
    body.visual(
        Cylinder(radius=0.020, length=0.008),
        origin=Origin(
            xyz=(0.0, -(outer_d / 2.0) + wall_t + 0.022, 0.058),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=metal,
        name="spindle_bezel",
    )
    body.visual(
        Cylinder(radius=hinge_r, length=0.040),
        origin=Origin(
            xyz=(-0.052, hinge_axis_y, hinge_axis_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=metal,
        name="left_hinge_barrel",
    )
    body.visual(
        Cylinder(radius=hinge_r, length=0.040),
        origin=Origin(
            xyz=(0.052, hinge_axis_y, hinge_axis_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=metal,
        name="right_hinge_barrel",
    )
    body.inertial = Inertial.from_geometry(
        Box((outer_w, outer_d, body_h)),
        mass=3.0,
        origin=Origin(xyz=(0.0, 0.0, body_h / 2.0)),
    )

    lid = model.part("lid")
    lid_frame_w = 0.040
    lid_front_rear = 0.032
    lid_center_y = 0.004 + outer_d / 2.0
    lid_z = 0.007 + lid_t / 2.0
    side_span_y = outer_d
    front_rear_span_x = outer_w - 2.0 * lid_frame_w

    lid.visual(
        Box((lid_frame_w, side_span_y, lid_t)),
        origin=Origin(xyz=(-(outer_w / 2.0) + lid_frame_w / 2.0, lid_center_y, lid_z)),
        material=wood,
        name="left_frame",
    )
    lid.visual(
        Box((lid_frame_w, side_span_y, lid_t)),
        origin=Origin(xyz=((outer_w / 2.0) - lid_frame_w / 2.0, lid_center_y, lid_z)),
        material=wood,
        name="right_frame",
    )
    lid.visual(
        Box((front_rear_span_x, lid_front_rear, lid_t)),
        origin=Origin(xyz=(0.0, 0.004 + lid_front_rear / 2.0, lid_z)),
        material=wood,
        name="rear_frame",
    )
    lid.visual(
        Box((front_rear_span_x, lid_front_rear, lid_t)),
        origin=Origin(xyz=(0.0, 0.004 + outer_d - lid_front_rear / 2.0, lid_z)),
        material=wood,
        name="front_frame",
    )
    lid.visual(
        Box((front_rear_span_x, outer_d - 2.0 * lid_front_rear, 0.006)),
        origin=Origin(xyz=(0.0, lid_center_y, 0.011)),
        material=smoked_glass,
        name="window_panel",
    )
    lid.visual(
        Cylinder(radius=hinge_r * 0.95, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="center_hinge_barrel",
    )
    lid.visual(
        Box((0.050, 0.024, 0.018)),
        origin=Origin(xyz=(0.0, 0.008, 0.014)),
        material=metal,
        name="hinge_leaf",
    )
    lid.inertial = Inertial.from_geometry(
        Box((outer_w, outer_d, lid_t)),
        mass=1.2,
        origin=Origin(xyz=(0.0, lid_center_y, lid_z)),
    )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.006, length=0.006),
        origin=Origin(xyz=(0.0, 0.003, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="spindle_stub",
    )
    cradle.visual(
        Cylinder(radius=0.026, length=0.010),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="back_plate",
    )
    cradle.visual(
        Box((0.074, 0.040, 0.082)),
        origin=Origin(xyz=(0.0, 0.031, 0.0)),
        material=lining,
        name="pillow_block",
    )
    cradle.visual(
        Box((0.030, 0.020, 0.022)),
        origin=Origin(xyz=(0.0, 0.046, 0.040)),
        material=lining,
        name="retainer_tab",
    )
    cradle.inertial = Inertial.from_geometry(
        Box((0.074, 0.058, 0.084)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.030, 0.0)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )
    model.articulation(
        "body_to_cradle",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cradle,
        origin=Origin(xyz=(0.0, -(outer_d / 2.0) + wall_t + 0.026, 0.058)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=6.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    cradle = object_model.get_part("cradle")
    lid_hinge = object_model.get_articulation("body_to_lid")
    cradle_spin = object_model.get_articulation("body_to_cradle")

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="front_frame",
            max_gap=0.004,
            max_penetration=0.0,
            name="closed lid sits tightly on the box body",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.120,
            name="closed lid covers the body opening footprint",
        )

    def aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lo + hi) / 2.0 for lo, hi in zip(lower, upper))

    closed_pos = aabb_center(ctx.part_element_world_aabb(lid, elem="front_frame"))
    with ctx.pose({lid_hinge: math.radians(90.0)}):
        open_pos = aabb_center(ctx.part_element_world_aabb(lid, elem="front_frame"))

    ctx.check(
        "lid opens upward from the rear hinge",
        closed_pos is not None
        and open_pos is not None
        and open_pos[2] > closed_pos[2] + 0.06
        and open_pos[1] < closed_pos[1] - 0.06,
        details=f"closed={closed_pos}, open={open_pos}",
    )

    ctx.expect_gap(
        cradle,
        body,
        axis="y",
        positive_elem="spindle_stub",
        negative_elem="spindle_bezel",
        max_gap=0.001,
        max_penetration=0.0,
        name="cradle spindle seats against the rear spindle bezel",
    )

    with ctx.pose({cradle_spin: 0.0}):
        ctx.expect_within(
            cradle,
            body,
            axes="xz",
            inner_elem="pillow_block",
            margin=0.0,
            name="cradle stays within the box opening at rest",
        )

    tab_rest = aabb_center(ctx.part_element_world_aabb(cradle, elem="retainer_tab"))
    with ctx.pose({cradle_spin: math.pi / 2.0}):
        ctx.expect_within(
            cradle,
            body,
            axes="xz",
            inner_elem="pillow_block",
            margin=0.0,
            name="cradle stays within the box opening while rotated",
        )
        tab_quarter = aabb_center(ctx.part_element_world_aabb(cradle, elem="retainer_tab"))

    ctx.check(
        "cradle rotates about the rear spindle",
        tab_rest is not None
        and tab_quarter is not None
        and tab_quarter[0] > tab_rest[0] + 0.025
        and tab_quarter[2] < tab_rest[2] - 0.025,
        details=f"rest={tab_rest}, quarter_turn={tab_quarter}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
