from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    LatheGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _circle_profile(radius: float, *, center=(0.0, 0.0), segments: int = 72) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * i / segments),
            cy + radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _hollow_cylinder_mesh(
    outer_radius: float,
    inner_radius: float,
    height: float,
    *,
    segments: int = 72,
) -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, 0.0), (outer_radius, height)],
        [(inner_radius, 0.0), (inner_radius, height)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
        lip_samples=3,
    )


def _blade_mesh() -> MeshGeometry:
    blade_thickness = 0.003
    # A two-armed swept cutting profile.  The roots deliberately overlap the
    # hub radius so the piece reads as one stamped metal blade.
    blade_profile = [
        (0.012, -0.010),
        (0.046, -0.018),
        (0.078, -0.011),
        (0.074, 0.005),
        (0.034, 0.014),
        (0.014, 0.009),
    ]

    mesh = MeshGeometry()
    for angle in (0.0, math.pi):
        part = ExtrudeGeometry(blade_profile, blade_thickness, cap=True, center=True)
        part.rotate_z(angle)
        part.translate(0.0, 0.0, 0.004)
        mesh.merge(part)

    # Small raised triangular vanes give the cutter a pitched, sharpened look
    # without placing broad blade surfaces near the bowl wall.
    for angle in (math.radians(18.0), math.pi + math.radians(18.0)):
        vane = ExtrudeGeometry(
            [(0.028, -0.004), (0.064, -0.006), (0.066, 0.004), (0.030, 0.007)],
            0.002,
            cap=True,
            center=True,
        )
        vane.rotate_z(angle)
        vane.translate(0.0, 0.0, 0.0052)
        mesh.merge(vane)

    return mesh


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mini_food_processor")

    white = model.material("warm_white_plastic", rgba=(0.92, 0.89, 0.83, 1.0))
    dark = model.material("dark_graphite", rgba=(0.05, 0.055, 0.06, 1.0))
    rubber = model.material("soft_black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    clear = model.material("clear_smoked_polycarbonate", rgba=(0.72, 0.90, 1.0, 0.36))
    steel = model.material("brushed_stainless_steel", rgba=(0.78, 0.78, 0.74, 1.0))
    amber = model.material("amber_switch_mark", rgba=(1.0, 0.55, 0.12, 1.0))

    # Root base: compact countertop motor housing with an inset front control
    # panel and a short annular twist-lock track for the bowl.
    base = model.part("base")
    base_shell = cq.Workplane("XY").box(0.24, 0.20, 0.085).edges("|Z").fillet(0.018)
    base.visual(
        mesh_from_cadquery(base_shell, "base_shell", tolerance=0.0008, angular_tolerance=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.0425)),
        material=white,
        name="base_shell",
    )
    base.visual(
        Box((0.165, 0.006, 0.055)),
        origin=Origin(xyz=(0.0, -0.103, 0.046)),
        material=dark,
        name="front_panel",
    )
    base.visual(
        mesh_from_geometry(_hollow_cylinder_mesh(0.083, 0.063, 0.010), "lock_track"),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=dark,
        name="lock_track",
    )
    for i, (x, y) in enumerate(((-0.080, -0.065), (0.080, -0.065), (-0.080, 0.065), (0.080, 0.065))):
        base.visual(
            Cylinder(radius=0.014, length=0.006),
            origin=Origin(xyz=(x, y, 0.003)),
            material=rubber,
            name=f"foot_{i}",
        )

    bowl = model.part("bowl")
    bowl_shell = LatheGeometry.from_shell_profiles(
        [
            (0.052, 0.000),
            (0.068, 0.018),
            (0.086, 0.080),
            (0.080, 0.142),
        ],
        [
            (0.035, 0.014),
            (0.056, 0.030),
            (0.073, 0.080),
            (0.069, 0.132),
        ],
        segments=96,
        start_cap="flat",
        end_cap="round",
        lip_samples=8,
    )
    bowl.visual(
        mesh_from_geometry(bowl_shell, "bowl_shell"),
        material=clear,
        name="bowl_shell",
    )

    # A transparent lid with a real off-center opening, carrying the narrow
    # hollow feed tube.  The bowl cavity remains visible through the clear shell.
    feed_xy = (0.034, -0.030)
    lid_plate = ExtrudeWithHolesGeometry(
        _circle_profile(0.083, segments=96),
        [_circle_profile(0.020, center=feed_xy, segments=48)],
        0.008,
        cap=True,
        center=True,
    )
    bowl.visual(
        mesh_from_geometry(lid_plate, "lid_plate"),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material=clear,
        name="lid_plate",
    )
    bowl.visual(
        mesh_from_geometry(_hollow_cylinder_mesh(0.026, 0.017, 0.108, segments=72), "feed_tube_shell"),
        origin=Origin(xyz=(feed_xy[0], feed_xy[1], 0.149)),
        material=clear,
        name="feed_tube_shell",
    )
    bowl.visual(
        mesh_from_geometry(_hollow_cylinder_mesh(0.031, 0.019, 0.010, segments=72), "feed_tube_lip"),
        origin=Origin(xyz=(feed_xy[0], feed_xy[1], 0.247)),
        material=clear,
        name="feed_tube_lip",
    )
    # Stationary center spindle molded into the bowl bottom.
    bowl.visual(
        Cylinder(radius=0.039, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=clear,
        name="bowl_floor",
    )
    bowl.visual(
        Cylinder(radius=0.008, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=dark,
        name="center_spindle",
    )
    bowl.visual(
        Cylinder(radius=0.018, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=dark,
        name="spindle_shoulder",
    )
    # Small molded handle tab to make the bowl orientation and twist-lock nature
    # legible while staying part of the rotating bowl assembly.
    bowl.visual(
        Box((0.034, 0.020, 0.045)),
        origin=Origin(xyz=(0.090, 0.0, 0.080)),
        material=clear,
        name="side_handle",
    )

    blade = model.part("blade")
    hub = LatheGeometry.from_shell_profiles(
        [(0.020, -0.006), (0.020, 0.018)],
        [(0.011, -0.006), (0.011, 0.018)],
        segments=64,
        start_cap="flat",
        end_cap="flat",
        lip_samples=3,
    )
    blade.visual(
        mesh_from_geometry(hub, "blade_hub"),
        material=steel,
        name="blade_hub",
    )
    blade.visual(
        mesh_from_geometry(_blade_mesh(), "cutting_blade"),
        material=steel,
        name="cutting_blade",
    )

    pusher = model.part("pusher")
    pusher.visual(
        Cylinder(radius=0.014, length=0.105),
        origin=Origin(xyz=(0.0, 0.0, -0.0525)),
        material=white,
        name="pusher_stem",
    )
    pusher.visual(
        Cylinder(radius=0.029, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=white,
        name="pusher_cap",
    )
    pusher.visual(
        Cylinder(radius=0.020, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=rubber,
        name="pusher_grip",
    )

    selector_knob = model.part("selector_knob")
    knob_geom = KnobGeometry(
        0.046,
        0.026,
        body_style="skirted",
        top_diameter=0.036,
        grip=KnobGrip(style="fluted", count=18, depth=0.0012),
        indicator=KnobIndicator(style="line", mode="raised", depth=0.0009, angle_deg=90.0),
        center=False,
    )
    selector_knob.visual(
        mesh_from_geometry(knob_geom, "selector_knob"),
        material=dark,
        name="knob_cap",
    )

    for idx, z in enumerate((0.034, 0.065)):
        rocker = model.part(f"rocker_{idx}")
        rocker.visual(
            Cylinder(radius=0.006, length=0.010),
            origin=Origin(xyz=(0.0, -0.005, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark,
            name="pivot_stem",
        )
        rocker.visual(
            Box((0.045, 0.010, 0.022)),
            origin=Origin(xyz=(0.0, -0.013, 0.0)),
            material=dark,
            name="rocker_face",
        )
        rocker.visual(
            Box((0.030, 0.002, 0.003)),
            origin=Origin(xyz=(0.0, -0.019, 0.005)),
            material=amber if idx == 0 else white,
            name="rocker_mark",
        )

    model.articulation(
        "base_to_bowl",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2),
    )
    model.articulation(
        "bowl_to_blade",
        ArticulationType.CONTINUOUS,
        parent=bowl,
        child=blade,
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=40.0),
    )
    model.articulation(
        "bowl_to_pusher",
        ArticulationType.PRISMATIC,
        parent=bowl,
        child=pusher,
        origin=Origin(xyz=(feed_xy[0], feed_xy[1], 0.257)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.065, effort=12.0, velocity=0.35),
    )
    model.articulation(
        "base_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=selector_knob,
        origin=Origin(xyz=(-0.052, -0.106, 0.052), rpy=(math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=6.0),
    )
    for idx, z in enumerate((0.034, 0.065)):
        model.articulation(
            f"base_to_rocker_{idx}",
            ArticulationType.REVOLUTE,
            parent=base,
            child=f"rocker_{idx}",
            origin=Origin(xyz=(0.054, -0.106, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(lower=-0.24, upper=0.24, effort=3.0, velocity=2.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    bowl = object_model.get_part("bowl")
    blade = object_model.get_part("blade")
    pusher = object_model.get_part("pusher")
    knob = object_model.get_part("selector_knob")
    rocker_0 = object_model.get_part("rocker_0")
    rocker_1 = object_model.get_part("rocker_1")
    pusher_slide = object_model.get_articulation("bowl_to_pusher")
    knob_turn = object_model.get_articulation("base_to_selector_knob")
    rocker_joint_0 = object_model.get_articulation("base_to_rocker_0")
    rocker_joint_1 = object_model.get_articulation("base_to_rocker_1")

    ctx.expect_contact(
        bowl,
        base,
        elem_a="bowl_shell",
        elem_b="base_shell",
        contact_tol=0.001,
        name="bowl sits on base at twist lock",
    )
    ctx.expect_within(
        blade,
        bowl,
        axes="xy",
        inner_elem="cutting_blade",
        outer_elem="bowl_shell",
        margin=0.001,
        name="blade fits within bowl cavity footprint",
    )
    ctx.expect_within(
        pusher,
        bowl,
        axes="xy",
        inner_elem="pusher_stem",
        outer_elem="feed_tube_shell",
        margin=0.001,
        name="pusher stem is centered inside feed tube",
    )
    ctx.expect_contact(
        pusher,
        bowl,
        elem_a="pusher_cap",
        elem_b="feed_tube_lip",
        contact_tol=0.002,
        name="pusher cap rests on feed tube lip",
    )

    rest_pos = ctx.part_world_position(pusher)
    with ctx.pose({pusher_slide: 0.060}):
        raised_pos = ctx.part_world_position(pusher)
        ctx.expect_within(
            pusher,
            bowl,
            axes="xy",
            inner_elem="pusher_stem",
            outer_elem="feed_tube_shell",
            margin=0.001,
            name="raised pusher remains guided by tube",
        )
        ctx.expect_overlap(
            pusher,
            bowl,
            axes="z",
            elem_a="pusher_stem",
            elem_b="feed_tube_shell",
            min_overlap=0.025,
            name="raised pusher remains inserted in feed tube",
        )
    ctx.check(
        "pusher slides upward in feed tube",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.050,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    with ctx.pose({knob_turn: math.pi / 2.0}):
        ctx.expect_contact(
            knob,
            base,
            elem_a="knob_cap",
            elem_b="front_panel",
            contact_tol=0.003,
            name="selector knob remains mounted while rotating",
        )
    with ctx.pose({rocker_joint_0: 0.18, rocker_joint_1: -0.18}):
        ctx.expect_contact(
            rocker_0,
            base,
            elem_a="pivot_stem",
            elem_b="front_panel",
            contact_tol=0.004,
            name="upper rocker stays on its pivot",
        )
        ctx.expect_contact(
            rocker_1,
            base,
            elem_a="pivot_stem",
            elem_b="front_panel",
            contact_tol=0.004,
            name="lower rocker stays on its pivot",
        )

    return ctx.report()


object_model = build_object_model()
