from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)

BODY_W = 0.62
BODY_D = 0.34
BODY_H = 0.82
WALL_T = 0.022
FRONT_T = 0.018
TOP_T = 0.022
PLINTH_H = 0.040
CONTROL_H = 0.160
LOWER_H = BODY_H - PLINTH_H - CONTROL_H
LOWER_CENTER_Z = PLINTH_H + (LOWER_H * 0.5)
CONTROL_CENTER_Z = BODY_H - (CONTROL_H * 0.5)
OPENING_R = 0.190
DOOR_OUTER_R = 0.232
WINDOW_R = 0.156
DOOR_T = 0.044
DOOR_CENTER_X = DOOR_OUTER_R + 0.038
HINGE_X = -DOOR_CENTER_X
HINGE_Y = BODY_D * 0.5 + 0.006
HINGE_BARREL_R = 0.006
DRUM_R = 0.180
DRUM_DEPTH = 0.220
DRUM_CENTER_Y = 0.008
DOOR_RING_CENTER_Y = 0.024
DOOR_TRIM_CENTER_Y = 0.008
DOOR_GLASS_CENTER_Y = 0.010
HANDLE_PIVOT_LOCAL = (DOOR_CENTER_X + 0.218, 0.060, 0.020)
DIAL_CENTER = (0.185, BODY_D * 0.5, CONTROL_CENTER_Z + 0.005)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _circle_profile(radius: float, *, segments: int = 64) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _annulus_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    thickness: float,
    segments: int = 72,
):
    geometry = ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius, segments=segments),
        [_circle_profile(inner_radius, segments=segments)],
        height=thickness,
        center=True,
    )
    geometry.rotate_x(-math.pi / 2.0)
    return _save_mesh(name, geometry)


def _pipe_shell_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    edge_inset: float,
):
    geometry = LatheGeometry.from_shell_profiles(
        [
            (outer_radius, -length * 0.5),
            (outer_radius, length * 0.5),
        ],
        [
            (inner_radius, -length * 0.5 + edge_inset),
            (inner_radius, length * 0.5 - edge_inset),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    geometry.rotate_x(-math.pi / 2.0)
    return _save_mesh(name, geometry)


def _aabb_center(aabb):
    return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stacked_washer_unit", assets=ASSETS)

    body_white = model.material("body_white", rgba=(0.95, 0.96, 0.97, 1.0))
    trim_silver = model.material("trim_silver", rgba=(0.76, 0.78, 0.81, 1.0))
    drum_steel = model.material("drum_steel", rgba=(0.67, 0.70, 0.73, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.20, 0.21, 0.22, 1.0))
    glass = model.material("glass", rgba=(0.70, 0.80, 0.88, 0.38))
    panel_black = model.material("panel_black", rgba=(0.10, 0.11, 0.12, 1.0))

    inner_w = BODY_W - (2.0 * WALL_T)
    opening_diameter = OPENING_R * 2.0
    lower_sill_h = 0.110
    lower_header_h = LOWER_H - opening_diameter - lower_sill_h
    side_frame_w = (inner_w - opening_diameter) * 0.5
    front_y = BODY_D * 0.5 - FRONT_T * 0.5

    bezel_ring_mesh = _annulus_mesh(
        "washer_bezel_ring.obj",
        outer_radius=OPENING_R + 0.018,
        inner_radius=OPENING_R - 0.004,
        thickness=0.010,
    )
    opening_liner_mesh = _annulus_mesh(
        "washer_opening_liner.obj",
        outer_radius=OPENING_R + 0.002,
        inner_radius=OPENING_R - 0.020,
        thickness=0.026,
    )
    drum_shell_mesh = _pipe_shell_mesh(
        "washer_drum_shell.obj",
        outer_radius=DRUM_R,
        inner_radius=DRUM_R - 0.010,
        length=DRUM_DEPTH,
        edge_inset=0.008,
    )
    door_ring_mesh = _annulus_mesh(
        "washer_door_ring.obj",
        outer_radius=DOOR_OUTER_R,
        inner_radius=OPENING_R - 0.008,
        thickness=DOOR_T,
    )
    door_trim_mesh = _annulus_mesh(
        "washer_door_trim.obj",
        outer_radius=DOOR_OUTER_R - 0.018,
        inner_radius=WINDOW_R,
        thickness=0.010,
    )

    body = model.part("body")
    body.visual(
        Box((BODY_W, BODY_D, PLINTH_H)),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_H * 0.5)),
        material=body_white,
        name="plinth",
    )
    body.visual(
        Box((WALL_T, BODY_D, BODY_H - PLINTH_H)),
        origin=Origin(xyz=(-BODY_W * 0.5 + WALL_T * 0.5, 0.0, PLINTH_H + (BODY_H - PLINTH_H) * 0.5)),
        material=body_white,
        name="left_side",
    )
    body.visual(
        Box((WALL_T, BODY_D, BODY_H - PLINTH_H)),
        origin=Origin(xyz=(BODY_W * 0.5 - WALL_T * 0.5, 0.0, PLINTH_H + (BODY_H - PLINTH_H) * 0.5)),
        material=body_white,
        name="right_side",
    )
    body.visual(
        Box((inner_w, BODY_D, TOP_T)),
        origin=Origin(xyz=(0.0, 0.0, BODY_H - TOP_T * 0.5)),
        material=body_white,
        name="top_panel",
    )
    body.visual(
        Box((inner_w, WALL_T, BODY_H - PLINTH_H - TOP_T)),
        origin=Origin(xyz=(0.0, -BODY_D * 0.5 + WALL_T * 0.5, PLINTH_H + (BODY_H - PLINTH_H - TOP_T) * 0.5)),
        material=body_white,
        name="back_panel",
    )
    body.visual(
        Box((inner_w, FRONT_T, CONTROL_H)),
        origin=Origin(xyz=(0.0, front_y, CONTROL_CENTER_Z)),
        material=body_white,
        name="control_panel",
    )
    body.visual(
        Box((side_frame_w, FRONT_T, LOWER_H)),
        origin=Origin(xyz=(-(opening_diameter + side_frame_w) * 0.5, front_y, LOWER_CENTER_Z)),
        material=body_white,
        name="left_front_frame",
    )
    body.visual(
        Box((side_frame_w, FRONT_T, LOWER_H)),
        origin=Origin(xyz=((opening_diameter + side_frame_w) * 0.5, front_y, LOWER_CENTER_Z)),
        material=body_white,
        name="right_front_frame",
    )
    body.visual(
        Box((opening_diameter, FRONT_T, lower_sill_h)),
        origin=Origin(xyz=(0.0, front_y, PLINTH_H + lower_sill_h * 0.5)),
        material=body_white,
        name="front_sill",
    )
    body.visual(
        Box((opening_diameter, FRONT_T, lower_header_h)),
        origin=Origin(
            xyz=(0.0, front_y, PLINTH_H + lower_sill_h + opening_diameter + lower_header_h * 0.5)
        ),
        material=body_white,
        name="front_header",
    )
    body.visual(
        bezel_ring_mesh,
        origin=Origin(xyz=(0.0, BODY_D * 0.5 - 0.0005, LOWER_CENTER_Z)),
        material=trim_silver,
        name="bezel_ring",
    )
    body.visual(
        opening_liner_mesh,
        origin=Origin(xyz=(0.0, BODY_D * 0.5 - 0.024, LOWER_CENTER_Z)),
        material=dark_grey,
        name="opening_liner",
    )
    body.visual(
        Box((0.136, 0.010, 0.050)),
        origin=Origin(xyz=(-0.115, BODY_D * 0.5 + 0.005, CONTROL_CENTER_Z + 0.005)),
        material=panel_black,
        name="display_window",
    )
    for index, x_pos in enumerate((-0.015, 0.015, 0.045)):
        body.visual(
            Box((0.016, 0.008, 0.016)),
            origin=Origin(xyz=(x_pos, BODY_D * 0.5 + 0.004, CONTROL_CENTER_Z - 0.030)),
            material=trim_silver,
            name=f"button_{index}",
        )
    body.visual(
        Box((0.008, 0.012, 0.300)),
        origin=Origin(xyz=(HINGE_X - 0.010, HINGE_Y, LOWER_CENTER_Z)),
        material=trim_silver,
        name="hinge_mount",
    )
    body.visual(
        Cylinder(radius=0.024, length=0.060),
        origin=Origin(xyz=(0.0, -0.135, LOWER_CENTER_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_grey,
        name="axle_mount",
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.028, length=0.016),
        origin=Origin(xyz=(0.0, 0.008, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=trim_silver,
        name="dial_knob",
    )
    dial.visual(
        Box((0.006, 0.006, 0.020)),
        origin=Origin(xyz=(0.0, 0.017, 0.015)),
        material=panel_black,
        name="dial_pointer",
    )

    drum = model.part("drum")
    drum.visual(drum_shell_mesh, material=drum_steel, name="drum_shell")
    drum.visual(
        Cylinder(radius=DRUM_R - 0.009, length=0.008),
        origin=Origin(xyz=(0.0, -(DRUM_DEPTH * 0.5) + 0.004, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=drum_steel,
        name="drum_back_plate",
    )
    drum.visual(
        Cylinder(radius=0.020, length=0.042),
        origin=Origin(xyz=(0.0, -(DRUM_DEPTH * 0.5) - 0.019, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_grey,
        name="drum_hub",
    )
    drum.visual(
        Box((0.042, DRUM_DEPTH * 0.70, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, DRUM_R - 0.024)),
        material=drum_steel,
        name="drum_lifter",
    )

    door = model.part("door")
    door.visual(
        door_ring_mesh,
        origin=Origin(xyz=(DOOR_CENTER_X, DOOR_RING_CENTER_Y, 0.0)),
        material=body_white,
        name="door_ring",
    )
    door.visual(
        door_trim_mesh,
        origin=Origin(xyz=(DOOR_CENTER_X, DOOR_TRIM_CENTER_Y, 0.0)),
        material=trim_silver,
        name="door_trim",
    )
    door.visual(
        Cylinder(radius=WINDOW_R + 0.004, length=0.010),
        origin=Origin(xyz=(DOOR_CENTER_X, DOOR_GLASS_CENTER_Y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=glass,
        name="door_glass",
    )
    door.visual(
        Cylinder(radius=HINGE_BARREL_R, length=0.290),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=trim_silver,
        name="hinge_barrel",
    )
    door.visual(
        Box((0.040, 0.010, 0.290)),
        origin=Origin(xyz=(0.044, 0.008, 0.0)),
        material=trim_silver,
        name="hinge_spine",
    )
    door.visual(
        Box((0.040, 0.010, 0.018)),
        origin=Origin(xyz=(0.022, 0.004, 0.116)),
        material=trim_silver,
        name="hinge_tab_upper",
    )
    door.visual(
        Box((0.040, 0.010, 0.018)),
        origin=Origin(xyz=(0.022, 0.004, -0.116)),
        material=trim_silver,
        name="hinge_tab_lower",
    )
    door.visual(
        Box((0.048, 0.012, 0.020)),
        origin=Origin(xyz=(DOOR_CENTER_X + 0.186, 0.038, 0.020)),
        material=trim_silver,
        name="handle_bridge",
    )
    door.visual(
        Box((0.030, 0.016, 0.020)),
        origin=Origin(xyz=(HANDLE_PIVOT_LOCAL[0], HANDLE_PIVOT_LOCAL[1] - 0.006, HANDLE_PIVOT_LOCAL[2])),
        material=trim_silver,
        name="handle_seat",
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.011, length=0.028),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_silver,
        name="handle_base",
    )
    handle.visual(
        Box((0.018, 0.012, 0.056)),
        origin=Origin(xyz=(0.0, 0.000, -0.037)),
        material=trim_silver,
        name="handle_grip",
    )

    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=DIAL_CENTER),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0),
    )
    model.articulation(
        "body_to_drum",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=drum,
        origin=Origin(xyz=(0.0, DRUM_CENTER_Y, LOWER_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=15.0),
    )
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(HINGE_X, HINGE_Y, LOWER_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=math.radians(110.0)),
    )
    model.articulation(
        "door_to_handle",
        ArticulationType.REVOLUTE,
        parent=door,
        child=handle,
        origin=Origin(xyz=HANDLE_PIVOT_LOCAL),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0, lower=0.0, upper=1.15),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)

    body = object_model.get_part("body")
    dial = object_model.get_part("dial")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    handle = object_model.get_part("handle")

    dial_spin = object_model.get_articulation("body_to_dial")
    drum_axle = object_model.get_articulation("body_to_drum")
    door_hinge = object_model.get_articulation("body_to_door")
    handle_hinge = object_model.get_articulation("door_to_handle")

    control_panel = body.get_visual("control_panel")
    bezel_ring = body.get_visual("bezel_ring")
    opening_liner = body.get_visual("opening_liner")
    hinge_mount = body.get_visual("hinge_mount")
    axle_mount = body.get_visual("axle_mount")
    dial_knob = dial.get_visual("dial_knob")
    drum_shell = drum.get_visual("drum_shell")
    drum_hub = drum.get_visual("drum_hub")
    door_ring = door.get_visual("door_ring")
    door_trim = door.get_visual("door_trim")
    hinge_barrel = door.get_visual("hinge_barrel")
    handle_seat = door.get_visual("handle_seat")
    handle_base = handle.get_visual("handle_base")
    handle_grip = handle.get_visual("handle_grip")

    ctx.allow_overlap(
        body,
        drum,
        elem_a=axle_mount,
        elem_b=drum_hub,
        reason="The drum hub is intentionally captured inside the rear bearing sleeve.",
    )
    ctx.allow_overlap(
        body,
        door,
        elem_a=hinge_mount,
        elem_b=hinge_barrel,
        reason="The porthole hinge barrel is intentionally seated inside the body-side hinge bracket.",
    )
    ctx.allow_overlap(
        body,
        door,
        elem_a=hinge_mount,
        elem_b="hinge_tab_upper",
        reason="The simplified hinge ear is modeled as a tight captured leaf around the body-side hinge bracket.",
    )
    ctx.allow_overlap(
        body,
        door,
        elem_a=hinge_mount,
        elem_b="hinge_tab_lower",
        reason="The simplified hinge ear is modeled as a tight captured leaf around the body-side hinge bracket.",
    )
    ctx.allow_overlap(
        door,
        handle,
        elem_a=handle_seat,
        elem_b=handle_base,
        reason="The flip-up latch uses a captured pivot pin inside the handle seat.",
    )
    ctx.allow_overlap(
        door,
        handle,
        elem_a=handle_seat,
        elem_b=handle_grip,
        reason="The folded latch grip nests into a shallow recessed cup; the simplified block grip intentionally shares a small swept volume with that recess during early lift travel.",
    )

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts(max_pose_samples=24)
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=48)

    ctx.expect_contact(dial, body, elem_a=dial_knob, elem_b=control_panel)
    ctx.expect_contact(drum, body, elem_a=drum_hub, elem_b=axle_mount)
    ctx.expect_contact(door, body, elem_a=hinge_barrel, elem_b=hinge_mount)
    ctx.expect_contact(handle, door, elem_a=handle_base, elem_b=handle_seat)
    ctx.expect_gap(
        body,
        drum,
        axis="y",
        min_gap=0.015,
        max_gap=0.055,
        positive_elem=opening_liner,
        negative_elem=drum_shell,
    )
    ctx.expect_within(drum, body, axes="xz", inner_elem=drum_shell)
    ctx.expect_gap(
        door,
        body,
        axis="y",
        min_gap=0.001,
        max_gap=0.012,
        positive_elem=door_trim,
        negative_elem=bezel_ring,
    )
    ctx.expect_overlap(door, body, axes="xz", min_overlap=0.30, elem_a=door_ring, elem_b=opening_liner)

    rest_door_ring_aabb = ctx.part_element_world_aabb(door, elem="door_ring")
    rest_handle_grip_aabb = ctx.part_element_world_aabb(handle, elem="handle_grip")
    rest_dial_pointer_aabb = ctx.part_element_world_aabb(dial, elem="dial_pointer")

    ctx.check("door_ring_aabb_available", rest_door_ring_aabb is not None, "door_ring world AABB missing")
    ctx.check("handle_grip_aabb_available", rest_handle_grip_aabb is not None, "handle_grip world AABB missing")
    ctx.check("dial_pointer_aabb_available", rest_dial_pointer_aabb is not None, "dial_pointer world AABB missing")

    rest_door_ring_center = _aabb_center(rest_door_ring_aabb) if rest_door_ring_aabb is not None else (0.0, 0.0, 0.0)
    rest_handle_grip_center = _aabb_center(rest_handle_grip_aabb) if rest_handle_grip_aabb is not None else (0.0, 0.0, 0.0)
    rest_dial_pointer_center = (
        _aabb_center(rest_dial_pointer_aabb) if rest_dial_pointer_aabb is not None else (0.0, 0.0, 0.0)
    )

    with ctx.pose({drum_axle: math.pi * 0.75}):
        ctx.expect_contact(drum, body, elem_a=drum_hub, elem_b=axle_mount)
        ctx.expect_within(drum, body, axes="xz", inner_elem=drum_shell)
        ctx.expect_gap(
            body,
            drum,
            axis="y",
            min_gap=0.015,
            max_gap=0.055,
            positive_elem=opening_liner,
            negative_elem=drum_shell,
        )

    door_limits = door_hinge.motion_limits
    if door_limits is not None and door_limits.lower is not None and door_limits.upper is not None:
        with ctx.pose({door_hinge: door_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="body_to_door_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="body_to_door_lower_no_floating")
        with ctx.pose({door_hinge: door_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="body_to_door_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="body_to_door_upper_no_floating")
            ctx.expect_contact(door, body, elem_a=hinge_barrel, elem_b=hinge_mount)
            ctx.expect_gap(
                door,
                body,
                axis="y",
                min_gap=0.18,
                positive_elem=handle_seat,
                negative_elem=opening_liner,
            )
            open_door_ring_aabb = ctx.part_element_world_aabb(door, elem="door_ring")
            ctx.check(
                "door_ring_open_aabb_available",
                open_door_ring_aabb is not None,
                "open door_ring world AABB missing",
            )
            if open_door_ring_aabb is not None:
                open_door_ring_center = _aabb_center(open_door_ring_aabb)
                ctx.check(
                    "door_swings_out_left",
                    (
                        open_door_ring_center[0] < rest_door_ring_center[0] - 0.10
                        and open_door_ring_center[1] > rest_door_ring_center[1] + 0.14
                    ),
                    (
                        f"closed center={rest_door_ring_center}, "
                        f"open center={open_door_ring_center}"
                    ),
                )

    handle_limits = handle_hinge.motion_limits
    if handle_limits is not None and handle_limits.lower is not None and handle_limits.upper is not None:
        with ctx.pose({handle_hinge: handle_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="door_to_handle_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="door_to_handle_lower_no_floating")
        with ctx.pose({handle_hinge: handle_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="door_to_handle_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="door_to_handle_upper_no_floating")
            ctx.expect_contact(handle, door, elem_a=handle_base, elem_b=handle_seat)
            ctx.expect_gap(
                handle,
                door,
                axis="y",
                min_gap=0.019,
                positive_elem=handle_grip,
                negative_elem=door_ring,
            )
            open_handle_grip_aabb = ctx.part_element_world_aabb(handle, elem="handle_grip")
            ctx.check(
                "handle_grip_open_aabb_available",
                open_handle_grip_aabb is not None,
                "open handle_grip world AABB missing",
            )
            if open_handle_grip_aabb is not None:
                open_handle_grip_center = _aabb_center(open_handle_grip_aabb)
                ctx.check(
                    "handle_flips_up",
                    (
                        open_handle_grip_center[1] > rest_handle_grip_center[1] + 0.018
                        and open_handle_grip_center[2] > rest_handle_grip_center[2] + 0.020
                    ),
                    (
                        f"closed center={rest_handle_grip_center}, "
                        f"open center={open_handle_grip_center}"
                    ),
                )

    with ctx.pose({door_hinge: math.radians(95.0), handle_hinge: 1.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="door_and_handle_open_no_overlap")
        ctx.fail_if_isolated_parts(name="door_and_handle_open_no_floating")

    with ctx.pose({dial_spin: math.pi * 0.5}):
        ctx.expect_contact(dial, body, elem_a=dial_knob, elem_b=control_panel)
        dial_pointer_quarter_turn = ctx.part_element_world_aabb(dial, elem="dial_pointer")
        ctx.check(
            "dial_pointer_rotated_aabb_available",
            dial_pointer_quarter_turn is not None,
            "rotated dial_pointer world AABB missing",
        )
        if dial_pointer_quarter_turn is not None:
            dial_pointer_rotated_center = _aabb_center(dial_pointer_quarter_turn)
            ctx.check(
                "dial_rotates_about_y_axis",
                dial_pointer_rotated_center[0] > rest_dial_pointer_center[0] + 0.010,
                (
                    f"rest center={rest_dial_pointer_center}, "
                    f"turned center={dial_pointer_rotated_center}"
                ),
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
