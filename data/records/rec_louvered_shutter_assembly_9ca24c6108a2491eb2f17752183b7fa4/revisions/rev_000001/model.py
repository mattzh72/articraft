from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PANEL_HEIGHT = 1.20
PANEL_WIDTH = 0.585
PANEL_DEPTH = 0.035
HINGE_OFFSET_Y = 0.050
PANEL_BOTTOM_Z = 0.120

STILE_W = 0.055
MEETING_STILE_W = 0.055
RAIL_H = 0.075
LOUVER_COUNT = 8
LOUVER_CHORD = 0.090
LOUVER_THICKNESS = 0.012
LOUVER_PIVOT_RADIUS = 0.006
LOUVER_INITIAL_ANGLE = math.radians(25.0)
LOUVER_INNER_WIDTH = PANEL_WIDTH - STILE_W - MEETING_STILE_W - 0.030


def _add_box(part, size, center, material, name):
    part.visual(Box(size), origin=Origin(xyz=center), material=material, name=name)


def _add_hinge_hardware(part, *, direction: float, material, moving: bool, hinge_x: float = 0.0) -> None:
    """Add alternating visible hinge leaves and knuckles around the local hinge axis."""
    segments = ((0.20, 0.135), (0.525, 0.150), (0.850, 0.150)) if not moving else ((0.355, 0.155), (0.695, 0.155), (1.020, 0.135))
    for idx, (z_center, height) in enumerate(segments):
        if moving:
            knuckle_origin = Origin(xyz=(0.0, 0.0, z_center))
        else:
            knuckle_origin = Origin(xyz=(hinge_x, -HINGE_OFFSET_Y, PANEL_BOTTOM_Z + z_center))
        part.visual(Cylinder(radius=0.011, length=height), origin=knuckle_origin, material=material, name=f"hinge_knuckle_{idx}")
        if moving:
            # A moving leaf bridges the barrel to the shutter's outer stile.
            _add_box(
                part,
                (0.050, 0.040, height * 0.76),
                (direction * 0.030, 0.016, z_center),
                material,
                f"hinge_leaf_{idx}",
            )
        else:
            # Fixed leaves sit on the front face of the surrounding opening frame.
            _add_box(
                part,
                (0.035, 0.040, height * 0.76),
                (hinge_x + direction * 0.020, -0.024, PANEL_BOTTOM_Z + z_center),
                material,
                f"fixed_leaf_{idx}_{'a' if direction < 0 else 'b'}",
            )


def _add_panel_frame(part, *, direction: float, wood, trim, metal) -> None:
    """Build a connected shutter panel frame in a hinge-axis local frame."""
    panel_center_x = direction * (PANEL_WIDTH * 0.5)
    y = HINGE_OFFSET_Y

    _add_box(
        part,
        (STILE_W, PANEL_DEPTH, PANEL_HEIGHT),
        (direction * (STILE_W * 0.5), y, PANEL_HEIGHT * 0.5),
        wood,
        "hinge_stile",
    )
    _add_box(
        part,
        (MEETING_STILE_W, PANEL_DEPTH, PANEL_HEIGHT),
        (direction * (PANEL_WIDTH - MEETING_STILE_W * 0.5), y, PANEL_HEIGHT * 0.5),
        wood,
        "meeting_stile",
    )
    _add_box(
        part,
        (PANEL_WIDTH, PANEL_DEPTH, RAIL_H),
        (panel_center_x, y, RAIL_H * 0.5),
        wood,
        "bottom_rail",
    )
    _add_box(
        part,
        (PANEL_WIDTH, PANEL_DEPTH, RAIL_H),
        (panel_center_x, y, PANEL_HEIGHT - RAIL_H * 0.5),
        wood,
        "top_rail",
    )

    # Thin raised inner stop strips frame the louver bay without filling it.
    inner_min = STILE_W
    inner_max = PANEL_WIDTH - MEETING_STILE_W
    inner_w = inner_max - inner_min
    inner_center = direction * ((inner_min + inner_max) * 0.5)
    stop_y = y - PANEL_DEPTH * 0.5 - 0.004
    _add_box(
        part,
        (inner_w, 0.007, 0.018),
        (inner_center, stop_y, RAIL_H + 0.009),
        trim,
        "lower_louver_stop",
    )
    _add_box(
        part,
        (inner_w, 0.007, 0.018),
        (inner_center, stop_y, PANEL_HEIGHT - RAIL_H - 0.009),
        trim,
        "upper_louver_stop",
    )
    _add_box(
        part,
        (0.018, 0.007, PANEL_HEIGHT - 2.0 * RAIL_H),
        (direction * (inner_min + 0.009), stop_y, PANEL_HEIGHT * 0.5),
        trim,
        "outer_louver_stop",
    )
    _add_box(
        part,
        (0.018, 0.007, PANEL_HEIGHT - 2.0 * RAIL_H),
        (direction * (inner_max - 0.009), stop_y, PANEL_HEIGHT * 0.5),
        trim,
        "inner_louver_stop",
    )

    # A small bevel-like strip on the meeting edge makes the center stiles read as a pair.
    _add_box(
        part,
        (0.014, 0.010, PANEL_HEIGHT - 0.060),
        (direction * (PANEL_WIDTH - 0.008), y - PANEL_DEPTH * 0.5 - 0.005, PANEL_HEIGHT * 0.5),
        trim,
        "meeting_lip",
    )
    _add_hinge_hardware(part, direction=direction, material=metal, moving=True)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="louvered_shutter_assembly")
    painted_wood = model.material("warm_white_painted_wood", rgba=(0.92, 0.90, 0.84, 1.0))
    trim_white = model.material("slightly_shadowed_trim", rgba=(0.82, 0.80, 0.74, 1.0))
    dark_hardware = model.material("dark_bronze_hardware", rgba=(0.10, 0.08, 0.055, 1.0))
    shadow = model.material("dark_opening_shadow", rgba=(0.055, 0.050, 0.045, 1.0))

    louver_blade_mesh = mesh_from_cadquery(
        cq.Workplane("XY")
        .box(LOUVER_INNER_WIDTH, LOUVER_CHORD, LOUVER_THICKNESS)
        .edges("|X")
        .fillet(LOUVER_THICKNESS * 0.42),
        "rounded_louver_blade",
        tolerance=0.0008,
        angular_tolerance=0.08,
    )

    opening_frame = model.part("opening_frame")
    # Surrounding fixed opening frame; it is made from connected jambs and rails,
    # leaving a real hollow opening behind the two shutter panels.
    _add_box(opening_frame, (0.070, 0.080, 1.440), (-0.635, 0.0, 0.720), painted_wood, "side_jamb_0")
    _add_box(opening_frame, (0.070, 0.080, 1.440), (0.635, 0.0, 0.720), painted_wood, "side_jamb_1")
    _add_box(opening_frame, (1.340, 0.080, 0.070), (0.0, 0.0, 0.035), painted_wood, "bottom_jamb")
    _add_box(opening_frame, (1.340, 0.080, 0.070), (0.0, 0.0, 1.405), painted_wood, "top_jamb")
    # A thin dark back liner, set well behind the louvers, makes the hollow window
    # opening legible without capping the front aperture.
    _add_box(opening_frame, (1.170, 0.010, 1.240), (0.0, 0.075, 0.720), shadow, "recess_shadow")
    _add_box(opening_frame, (0.018, 0.045, 1.260), (-0.594, 0.057, 0.720), shadow, "side_shadow_return_0")
    _add_box(opening_frame, (0.018, 0.045, 1.260), (0.594, 0.057, 0.720), shadow, "side_shadow_return_1")
    _add_box(opening_frame, (1.190, 0.045, 0.018), (0.0, 0.057, 0.084), shadow, "bottom_shadow_return")
    _add_box(opening_frame, (1.190, 0.045, 0.018), (0.0, 0.057, 1.356), shadow, "top_shadow_return")

    _add_hinge_hardware(opening_frame, direction=-1.0, material=dark_hardware, moving=False, hinge_x=-0.600)
    _add_hinge_hardware(opening_frame, direction=1.0, material=dark_hardware, moving=False, hinge_x=0.600)

    panels = []
    for panel_index, direction, hinge_x, axis_z in (
        (0, 1.0, -0.600, 1.0),
        (1, -1.0, 0.600, -1.0),
    ):
        panel = model.part(f"panel_{panel_index}")
        _add_panel_frame(panel, direction=direction, wood=painted_wood, trim=trim_white, metal=dark_hardware)
        panels.append((panel, direction))

        model.articulation(
            f"frame_to_panel_{panel_index}",
            ArticulationType.REVOLUTE,
            parent=opening_frame,
            child=panel,
            origin=Origin(xyz=(hinge_x, -HINGE_OFFSET_Y, PANEL_BOTTOM_Z)),
            axis=(0.0, 0.0, axis_z),
            motion_limits=MotionLimits(lower=0.0, upper=math.radians(105.0), effort=18.0, velocity=1.5),
        )

    inner_min = STILE_W
    inner_max = PANEL_WIDTH - MEETING_STILE_W
    louver_span = inner_max - inner_min
    louver_center_abs = (inner_min + inner_max) * 0.5
    z_min = RAIL_H + 0.070
    z_spacing = (PANEL_HEIGHT - 2.0 * (RAIL_H + 0.070)) / (LOUVER_COUNT - 1)

    for panel_index, (panel, direction) in enumerate(panels):
        for louver_index in range(LOUVER_COUNT):
            z = z_min + louver_index * z_spacing
            louver = model.part(f"louver_{panel_index}_{louver_index}")
            louver.visual(
                louver_blade_mesh,
                origin=Origin(rpy=(LOUVER_INITIAL_ANGLE, 0.0, 0.0)),
                material=painted_wood,
                name="blade",
            )
            louver.visual(
                Cylinder(radius=LOUVER_PIVOT_RADIUS, length=louver_span),
                origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
                material=dark_hardware,
                name="pivot_pin",
            )
            model.articulation(
                f"panel_{panel_index}_to_louver_{louver_index}",
                ArticulationType.REVOLUTE,
                parent=panel,
                child=louver,
                origin=Origin(xyz=(direction * louver_center_abs, HINGE_OFFSET_Y, z)),
                axis=(1.0, 0.0, 0.0),
                motion_limits=MotionLimits(lower=math.radians(-55.0), upper=math.radians(55.0), effort=1.0, velocity=2.0),
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("opening_frame")
    panel_0 = object_model.get_part("panel_0")
    panel_1 = object_model.get_part("panel_1")
    panel_joint_0 = object_model.get_articulation("frame_to_panel_0")
    panel_joint_1 = object_model.get_articulation("frame_to_panel_1")

    ctx.check("two_wide_panels_present", panel_0 is not None and panel_1 is not None, "Expected two shutter panels.")
    louver_parts = [p for p in object_model.parts if p.name.startswith("louver_")]
    louver_joints = [j for j in object_model.articulations if "_to_louver_" in j.name]
    ctx.check("all_louvers_are_parts", len(louver_parts) == 2 * LOUVER_COUNT, f"louvers={len(louver_parts)}")
    ctx.check("all_louvers_are_revolute", len(louver_joints) == 2 * LOUVER_COUNT, f"louver_joints={len(louver_joints)}")

    if frame is not None and panel_0 is not None and panel_1 is not None:
        ctx.expect_contact(
            panel_0,
            frame,
            elem_a="hinge_stile",
            elem_b="side_jamb_0",
            contact_tol=0.001,
            name="panel_0_hinged_at_outer_jamb",
        )
        ctx.expect_contact(
            panel_1,
            frame,
            elem_a="hinge_stile",
            elem_b="side_jamb_1",
            contact_tol=0.001,
            name="panel_1_hinged_at_outer_jamb",
        )
        ctx.expect_gap(
            panel_1,
            panel_0,
            axis="x",
            positive_elem="meeting_stile",
            negative_elem="meeting_stile",
            min_gap=0.020,
            max_gap=0.040,
            name="center_meeting_stiles_have_reveal_gap",
        )

    # A representative louver is carried between the two stiles by its pivot pin.
    sample_louver = object_model.get_part("louver_0_3")
    if panel_0 is not None and sample_louver is not None:
        ctx.expect_within(
            sample_louver,
            panel_0,
            axes="x",
            inner_elem="pivot_pin",
            outer_elem="hinge_stile",
            margin=PANEL_WIDTH,
            name="representative_louver_has_panel_pivot",
        )
        ctx.expect_contact(
            sample_louver,
            panel_0,
            elem_a="pivot_pin",
            elem_b="hinge_stile",
            contact_tol=0.0015,
            name="louver_pin_contacts_outer_stile",
        )

    if panel_joint_0 is not None and panel_joint_1 is not None and panel_0 is not None and panel_1 is not None:
        closed_0 = ctx.part_world_aabb(panel_0)
        closed_1 = ctx.part_world_aabb(panel_1)
        with ctx.pose({panel_joint_0: math.radians(80.0), panel_joint_1: math.radians(80.0)}):
            open_0 = ctx.part_world_aabb(panel_0)
            open_1 = ctx.part_world_aabb(panel_1)
        if closed_0 is not None and open_0 is not None and closed_1 is not None and open_1 is not None:
            ctx.check(
                "panels_swing_outward",
                open_0[1][1] > closed_0[1][1] + 0.25 and open_1[1][1] > closed_1[1][1] + 0.25,
                details=f"closed0={closed_0}, open0={open_0}, closed1={closed_1}, open1={open_1}",
            )

    louver_joint = object_model.get_articulation("panel_0_to_louver_3")
    if sample_louver is not None and louver_joint is not None:
        rest = ctx.part_world_aabb(sample_louver)
        with ctx.pose({louver_joint: math.radians(45.0)}):
            tilted = ctx.part_world_aabb(sample_louver)
        if rest is not None and tilted is not None:
            rest_depth = rest[1][1] - rest[0][1]
            tilted_depth = tilted[1][1] - tilted[0][1]
            rest_height = rest[1][2] - rest[0][2]
            tilted_height = tilted[1][2] - tilted[0][2]
            ctx.check(
                "louver_rotates_about_long_axis",
                abs(tilted_depth - rest_depth) > 0.006 and abs(tilted_height - rest_height) > 0.030,
                details=f"rest_depth={rest_depth}, tilted_depth={tilted_depth}, rest_height={rest_height}, tilted_height={tilted_height}",
            )

    return ctx.report()


object_model = build_object_model()
