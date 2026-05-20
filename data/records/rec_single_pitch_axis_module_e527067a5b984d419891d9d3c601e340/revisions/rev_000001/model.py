from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

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
    mesh_from_cadquery,
)


def make_base(base_w: float, base_d: float, base_t: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(base_w, base_d, base_t)
        .edges("|Z")
        .fillet(min(0.006, base_t * 0.3))
    )


def make_tower(
    tower_t: float,
    tower_d: float,
    tower_h: float,
    shaft_local_z: float,
    tower_boss_r: float,
    shaft_r: float,
) -> cq.Workplane:
    column = (
        cq.Workplane("XY")
        .box(tower_t, tower_d, tower_h)
        .translate((0.0, 0.0, tower_h / 2.0))
        .edges("|Z")
        .fillet(0.0035)
        .edges(">Z")
        .fillet(0.010)
    )

    rear_gusset = (
        cq.Workplane("YZ")
        .polyline(
            [
                (-tower_d / 2.0, 0.0),
                (-tower_d / 2.0, shaft_local_z - 0.045),
                (-0.010, 0.0),
            ]
        )
        .close()
        .extrude(tower_t * 0.24, both=True)
    )

    boss = (
        cq.Workplane("YZ")
        .center(0.0, shaft_local_z)
        .circle(tower_boss_r)
        .extrude((tower_t / 2.0) + 0.004, both=True)
    )

    return column.union(rear_gusset).union(boss)


def make_cradle(
    cradle_outer_w: float,
    side_plate_t: float,
    tray_d: float,
    tray_h: float,
    tray_top_z: float,
    wall_t: float,
    cradle_boss_r: float,
    hole_r: float,
) -> cq.Workplane:
    tray_w = cradle_outer_w - (2.0 * side_plate_t) + 0.004
    tray_center_z = tray_top_z - (tray_h / 2.0)
    plate_drop = tray_h - tray_top_z

    tray = (
        cq.Workplane("XY")
        .box(tray_w, tray_d, tray_h)
        .translate((0.0, 0.0, tray_center_z))
        .faces(">Z")
        .shell(-wall_t)
    )

    cheek = (
        cq.Workplane("YZ")
        .center(0.0, -plate_drop / 2.0)
        .rect(tray_d, plate_drop)
        .extrude(side_plate_t / 2.0, both=True)
        .union(
            cq.Workplane("YZ")
            .circle(cradle_boss_r)
            .extrude(side_plate_t / 2.0, both=True)
        )
        .cut(
            cq.Workplane("YZ")
            .circle(hole_r)
            .extrude(side_plate_t, both=True)
        )
    )

    left_cheek = cheek.translate(
        (-(cradle_outer_w / 2.0) + (side_plate_t / 2.0), 0.0, 0.0)
    )
    right_cheek = cheek.translate(
        ((cradle_outer_w / 2.0) - (side_plate_t / 2.0), 0.0, 0.0)
    )

    return tray.union(left_cheek).union(right_cheek)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilt_head")

    frame_dark = model.material("frame_dark", rgba=(0.20, 0.22, 0.25, 1.0))
    shaft_black = model.material("shaft_black", rgba=(0.08, 0.08, 0.09, 1.0))
    cradle_gray = model.material("cradle_gray", rgba=(0.73, 0.75, 0.77, 1.0))

    base_w = 0.24
    base_d = 0.16
    base_t = 0.020

    tower_t = 0.022
    tower_d = 0.060
    tower_h = 0.180
    tower_embed = 0.001

    inner_gap = 0.190
    tower_center_x = (inner_gap / 2.0) + (tower_t / 2.0)

    shaft_d = 0.018
    shaft_r = shaft_d / 2.0
    shaft_z = 0.175
    shaft_len = 2.0 * (tower_center_x + (tower_t / 2.0) + 0.004)
    tower_boss_r = 0.022
    shaft_local_z = shaft_z - (base_t - tower_embed)

    cradle_outer_w = inner_gap - 0.022
    side_plate_t = 0.010
    tray_d = 0.120
    tray_h = 0.050
    tray_top_z = -0.030
    wall_t = 0.004
    cradle_boss_r = 0.018
    hole_r = shaft_r + 0.00025

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(
            make_base(base_w, base_d, base_t),
            "base_block",
            tolerance=0.0003,
            angular_tolerance=0.05,
        ),
        origin=Origin(xyz=(0.0, 0.0, base_t / 2.0)),
        material=frame_dark,
        name="base",
    )
    frame.visual(
        mesh_from_cadquery(
            make_tower(
                tower_t=tower_t,
                tower_d=tower_d,
                tower_h=tower_h,
                shaft_local_z=shaft_local_z,
                tower_boss_r=tower_boss_r,
                shaft_r=shaft_r,
            ),
            "left_tower",
            tolerance=0.0003,
            angular_tolerance=0.05,
        ),
        origin=Origin(xyz=(-tower_center_x, 0.0, base_t - tower_embed)),
        material=frame_dark,
        name="left_tower",
    )
    frame.visual(
        mesh_from_cadquery(
            make_tower(
                tower_t=tower_t,
                tower_d=tower_d,
                tower_h=tower_h,
                shaft_local_z=shaft_local_z,
                tower_boss_r=tower_boss_r,
                shaft_r=shaft_r,
            ),
            "right_tower",
            tolerance=0.0003,
            angular_tolerance=0.05,
        ),
        origin=Origin(xyz=(tower_center_x, 0.0, base_t - tower_embed)),
        material=frame_dark,
        name="right_tower",
    )
    frame.visual(
        Cylinder(radius=shaft_r, length=shaft_len),
        origin=Origin(xyz=(0.0, 0.0, shaft_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shaft_black,
        name="cross_shaft",
    )
    frame.inertial = Inertial.from_geometry(
        Box((base_w, base_d, tower_h + base_t)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, (tower_h + base_t) / 2.0)),
    )

    cradle = model.part("cradle")
    cradle.visual(
        mesh_from_cadquery(
            make_cradle(
                cradle_outer_w=cradle_outer_w,
                side_plate_t=side_plate_t,
                tray_d=tray_d,
                tray_h=tray_h,
                tray_top_z=tray_top_z,
                wall_t=wall_t,
                cradle_boss_r=cradle_boss_r,
                hole_r=hole_r,
            ),
            "payload_cradle",
            tolerance=0.00015,
            angular_tolerance=0.03,
        ),
        material=cradle_gray,
        name="payload_cradle",
    )
    cradle_height = tray_h - tray_top_z
    cradle.inertial = Inertial.from_geometry(
        Box((cradle_outer_w, tray_d, cradle_height)),
        mass=1.0,
        origin=Origin(xyz=(0.0, 0.0, -cradle_height / 2.0)),
    )

    model.articulation(
        "frame_to_cradle_tilt",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, shaft_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=-1.05,
            upper=1.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts(contact_tol=0.0006)
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

    part_names = {part.name for part in object_model.parts}
    joint_names = {joint.name for joint in object_model.articulations}

    ctx.check("frame_present", "frame" in part_names, "expected part 'frame'")
    ctx.check("cradle_present", "cradle" in part_names, "expected part 'cradle'")
    ctx.check(
        "tilt_joint_present",
        "frame_to_cradle_tilt" in joint_names,
        "expected articulation 'frame_to_cradle_tilt'",
    )

    if (
        "frame" in part_names
        and "cradle" in part_names
        and "frame_to_cradle_tilt" in joint_names
    ):
        frame = object_model.get_part("frame")
        cradle = object_model.get_part("cradle")
        tilt = object_model.get_articulation("frame_to_cradle_tilt")
        limits = tilt.motion_limits

        ctx.check(
            "tilt_joint_is_revolute",
            tilt.articulation_type == ArticulationType.REVOLUTE,
            f"unexpected articulation type: {tilt.articulation_type!r}",
        )
        ctx.check(
            "tilt_axis_is_horizontal_x",
            tuple(round(v, 6) for v in tilt.axis) == (1.0, 0.0, 0.0),
            f"unexpected tilt axis: {tilt.axis!r}",
        )
        ctx.check(
            "tilt_limits_allow_bidirectional_pitch",
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower < 0.0 < limits.upper,
            f"unexpected motion limits: {limits!r}",
        )

        ctx.expect_origin_distance(
            cradle,
            frame,
            axes="xy",
            max_dist=0.001,
            name="cradle_centered_between_towers",
        )
        ctx.expect_origin_gap(
            cradle,
            frame,
            axis="z",
            min_gap=0.170,
            max_gap=0.180,
            name="pivot_height_above_base",
        )
        ctx.expect_gap(
            cradle,
            frame,
            axis="x",
            negative_elem="left_tower",
            min_gap=0.006,
            max_gap=0.016,
            name="left_tower_clearance",
        )
        ctx.expect_gap(
            frame,
            cradle,
            axis="x",
            positive_elem="right_tower",
            min_gap=0.006,
            max_gap=0.016,
            name="right_tower_clearance",
        )
        ctx.expect_gap(
            cradle,
            frame,
            axis="z",
            negative_elem="base",
            min_gap=0.050,
            name="tray_clears_base_at_rest",
        )
        ctx.expect_overlap(
            cradle,
            frame,
            axes="x",
            min_overlap=0.160,
            elem_b="cross_shaft",
            name="cradle_is_carried_by_cross_shaft_span",
        )
        ctx.expect_overlap(
            cradle,
            frame,
            axes="yz",
            min_overlap=0.015,
            elem_b="cross_shaft",
            name="cross_shaft_runs_through_cradle_pivot_zone",
        )

        with ctx.pose({tilt: -0.90}):
            ctx.fail_if_parts_overlap_in_current_pose(
                name="no_part_overlap_at_forward_tilt"
            )
            ctx.expect_gap(
                cradle,
                frame,
                axis="z",
                negative_elem="base",
                min_gap=0.040,
                name="tray_clears_base_at_forward_tilt",
            )

        with ctx.pose({tilt: 0.90}):
            ctx.fail_if_parts_overlap_in_current_pose(
                name="no_part_overlap_at_backward_tilt"
            )
            ctx.expect_gap(
                cradle,
                frame,
                axis="z",
                negative_elem="base",
                min_gap=0.040,
                name="tray_clears_base_at_backward_tilt",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
