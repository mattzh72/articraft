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


def _x_cylinder_origin(x_start: float, length: float, *, y: float = 0.0, z: float = 0.0) -> Origin:
    return Origin(xyz=(x_start + length / 2.0, y, z), rpy=(0.0, pi / 2.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dj_fx_rack_unit")

    chassis_color = model.material("chassis_color", rgba=(0.13, 0.14, 0.15, 1.0))
    fascia_color = model.material("fascia_color", rgba=(0.18, 0.19, 0.21, 1.0))
    metal_dark = model.material("metal_dark", rgba=(0.28, 0.30, 0.33, 1.0))
    metal_mid = model.material("metal_mid", rgba=(0.55, 0.57, 0.60, 1.0))
    knob_cap = model.material("knob_cap", rgba=(0.70, 0.72, 0.74, 1.0))
    smoked_cover = model.material("smoked_cover", rgba=(0.22, 0.26, 0.30, 0.34))

    total_width = 0.483
    body_width = 0.438
    ear_width = (total_width - body_width) / 2.0
    height = 0.133
    depth = 0.245
    wall = 0.003
    front_plate_t = 0.004
    rear_plate_t = 0.003
    front_outer_x = depth / 2.0

    housing = model.part("housing")
    housing.visual(
        Box((depth, body_width, wall)),
        origin=Origin(xyz=(0.0, 0.0, -height / 2.0 + wall / 2.0)),
        material=chassis_color,
        name="bottom_shell",
    )
    housing.visual(
        Box((depth, body_width, wall)),
        origin=Origin(xyz=(0.0, 0.0, height / 2.0 - wall / 2.0)),
        material=chassis_color,
        name="top_shell",
    )
    housing.visual(
        Box((depth, wall, height - 2.0 * wall)),
        origin=Origin(xyz=(0.0, body_width / 2.0 - wall / 2.0, 0.0)),
        material=chassis_color,
        name="left_shell",
    )
    housing.visual(
        Box((depth, wall, height - 2.0 * wall)),
        origin=Origin(xyz=(0.0, -body_width / 2.0 + wall / 2.0, 0.0)),
        material=chassis_color,
        name="right_shell",
    )
    housing.visual(
        Box((rear_plate_t, body_width - 2.0 * wall, height - 2.0 * wall)),
        origin=Origin(xyz=(-depth / 2.0 + rear_plate_t / 2.0, 0.0, 0.0)),
        material=chassis_color,
        name="rear_panel",
    )
    housing.visual(
        Box((front_plate_t, total_width, height)),
        origin=Origin(xyz=(front_outer_x - front_plate_t / 2.0, 0.0, 0.0)),
        material=fascia_color,
        name="front_panel",
    )
    housing.visual(
        Box((0.030, ear_width, height - 0.008)),
        origin=Origin(
            xyz=(front_outer_x - 0.015, body_width / 2.0 + ear_width / 2.0, 0.0)
        ),
        material=chassis_color,
        name="left_ear_return",
    )
    housing.visual(
        Box((0.030, ear_width, height - 0.008)),
        origin=Origin(
            xyz=(front_outer_x - 0.015, -body_width / 2.0 - ear_width / 2.0, 0.0)
        ),
        material=chassis_color,
        name="right_ear_return",
    )
    housing.visual(
        Box((0.0015, 0.100, 0.021)),
        origin=Origin(xyz=(front_outer_x - 0.00075, 0.0, 0.053)),
        material=metal_dark,
        name="display_window",
    )

    jog_wheel = model.part("jog_wheel")
    jog_wheel.visual(
        Cylinder(radius=0.052, length=0.012),
        origin=_x_cylinder_origin(0.0, 0.012),
        material=metal_dark,
        name="wheel_ring",
    )
    jog_wheel.visual(
        Cylinder(radius=0.041, length=0.009),
        origin=_x_cylinder_origin(0.002, 0.009),
        material=metal_mid,
        name="wheel_platter",
    )
    jog_wheel.visual(
        Cylinder(radius=0.018, length=0.015),
        origin=_x_cylinder_origin(0.001, 0.015),
        material=knob_cap,
        name="wheel_hub",
    )

    knob_positions = (-0.145, -0.065, 0.065, 0.145)
    knob_parts = []
    for index, y_pos in enumerate(knob_positions, start=1):
        knob = model.part(f"knob_{index}")
        knob.visual(
            Cylinder(radius=0.006, length=0.010),
            origin=_x_cylinder_origin(0.0, 0.010),
            material=metal_dark,
            name="shaft",
        )
        knob.visual(
            Cylinder(radius=0.015, length=0.018),
            origin=_x_cylinder_origin(0.004, 0.018),
            material=chassis_color,
            name="body",
        )
        knob.visual(
            Cylinder(radius=0.012, length=0.006),
            origin=_x_cylinder_origin(0.016, 0.006),
            material=knob_cap,
            name="cap",
        )
        knob_parts.append((knob, y_pos))

    cover = model.part("front_cover")
    cover_width = 0.428
    cover_height = 0.121
    cover_depth = 0.048
    frame_t = 0.010
    side_t = 0.003
    window_t = 0.0025
    cover.visual(
        Box((window_t, cover_width - 2.0 * frame_t, cover_height - 2.0 * frame_t)),
        origin=Origin(xyz=(cover_depth - window_t / 2.0, 0.0, cover_height / 2.0)),
        material=smoked_cover,
        name="cover_window",
    )
    cover.visual(
        Box((window_t, cover_width, frame_t)),
        origin=Origin(xyz=(cover_depth - window_t / 2.0, 0.0, frame_t / 2.0)),
        material=metal_dark,
        name="bottom_frame",
    )
    cover.visual(
        Box((window_t, cover_width, frame_t)),
        origin=Origin(
            xyz=(cover_depth - window_t / 2.0, 0.0, cover_height - frame_t / 2.0)
        ),
        material=metal_dark,
        name="top_frame",
    )
    cover.visual(
        Box((window_t, frame_t, cover_height - 2.0 * frame_t)),
        origin=Origin(
            xyz=(
                cover_depth - window_t / 2.0,
                cover_width / 2.0 - frame_t / 2.0,
                cover_height / 2.0,
            )
        ),
        material=metal_dark,
        name="left_front_frame",
    )
    cover.visual(
        Box((window_t, frame_t, cover_height - 2.0 * frame_t)),
        origin=Origin(
            xyz=(
                cover_depth - window_t / 2.0,
                -cover_width / 2.0 + frame_t / 2.0,
                cover_height / 2.0,
            )
        ),
        material=metal_dark,
        name="right_front_frame",
    )
    cover.visual(
        Box((cover_depth, side_t, cover_height)),
        origin=Origin(
            xyz=(cover_depth / 2.0, cover_width / 2.0 - side_t / 2.0, cover_height / 2.0)
        ),
        material=metal_dark,
        name="left_side_wall",
    )
    cover.visual(
        Box((cover_depth, side_t, cover_height)),
        origin=Origin(
            xyz=(cover_depth / 2.0, -cover_width / 2.0 + side_t / 2.0, cover_height / 2.0)
        ),
        material=metal_dark,
        name="right_side_wall",
    )
    cover.visual(
        Box((cover_depth, cover_width - 2.0 * side_t, side_t)),
        origin=Origin(xyz=(cover_depth / 2.0, 0.0, side_t / 2.0)),
        material=metal_dark,
        name="bottom_tray_wall",
    )
    cover.visual(
        Box((cover_depth, cover_width - 2.0 * side_t, side_t)),
        origin=Origin(xyz=(cover_depth / 2.0, 0.0, cover_height - side_t / 2.0)),
        material=metal_dark,
        name="top_tray_wall",
    )

    model.articulation(
        "housing_to_jog_wheel",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=jog_wheel,
        origin=Origin(xyz=(front_outer_x, 0.0, -0.004)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=8.0,
            lower=-pi,
            upper=pi,
        ),
    )

    for index, (knob, y_pos) in enumerate(knob_parts, start=1):
        model.articulation(
            f"housing_to_knob_{index}",
            ArticulationType.REVOLUTE,
            parent=housing,
            child=knob,
            origin=Origin(xyz=(front_outer_x, y_pos, 0.039)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.0,
                velocity=4.0,
                lower=-2.35,
                upper=2.35,
            ),
        )

    model.articulation(
        "housing_to_front_cover",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=cover,
        origin=Origin(xyz=(front_outer_x, 0.0, -0.0605)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    jog_wheel = object_model.get_part("jog_wheel")
    cover = object_model.get_part("front_cover")
    cover_hinge = object_model.get_articulation("housing_to_front_cover")
    wheel_joint = object_model.get_articulation("housing_to_jog_wheel")
    knob_joints = [object_model.get_articulation(f"housing_to_knob_{i}") for i in range(1, 5)]
    knob_parts = [object_model.get_part(f"knob_{i}") for i in range(1, 5)]

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "control axes spin about the front-panel normal",
        wheel_joint.axis == (1.0, 0.0, 0.0) and all(joint.axis == (1.0, 0.0, 0.0) for joint in knob_joints),
        details=f"wheel_axis={wheel_joint.axis}, knob_axes={[joint.axis for joint in knob_joints]}",
    )
    ctx.check(
        "cover hinge rotates about the lower horizontal edge",
        cover_hinge.axis == (0.0, 1.0, 0.0),
        details=f"cover_axis={cover_hinge.axis}",
    )

    ctx.expect_gap(
        jog_wheel,
        housing,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        name="jog wheel seats on the front panel",
    )
    for index, knob in enumerate(knob_parts, start=1):
        ctx.expect_gap(
            knob,
            housing,
            axis="x",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"knob {index} seats on the front panel",
        )

    with ctx.pose({cover_hinge: 0.0}):
        ctx.expect_contact(
            cover,
            housing,
            elem_a="bottom_tray_wall",
            elem_b="front_panel",
            name="closed cover meets the front panel at the lower edge",
        )
        ctx.expect_overlap(
            cover,
            housing,
            axes="yz",
            min_overlap=0.10,
            name="cover spans the active front-panel area",
        )
        ctx.expect_within(
            jog_wheel,
            cover,
            axes="yz",
            margin=0.0,
            name="jog wheel sits inside the cover footprint",
        )
        for index, knob in enumerate(knob_parts, start=1):
            ctx.expect_within(
                knob,
                cover,
                axes="yz",
                margin=0.0,
                name=f"knob {index} sits inside the cover footprint",
            )

    closed_window_aabb = ctx.part_element_world_aabb(cover, elem="cover_window")
    with ctx.pose({cover_hinge: 1.20}):
        open_window_aabb = ctx.part_element_world_aabb(cover, elem="cover_window")

    if closed_window_aabb is None or open_window_aabb is None:
        ctx.fail(
            "cover window pose samples resolve",
            details=f"closed={closed_window_aabb}, open={open_window_aabb}",
        )
    else:
        closed_min, closed_max = closed_window_aabb
        open_min, open_max = open_window_aabb
        ctx.check(
            "cover swings downward and outward when opened",
            open_max[0] > closed_max[0] + 0.050
            and open_max[2] < closed_min[2] - 0.010
            and open_min[2] < closed_min[2] - 0.035,
            details=(
                f"closed_min={closed_min}, closed_max={closed_max}, "
                f"open_min={open_min}, open_max={open_max}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
