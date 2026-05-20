from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

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
    TorusGeometry,
    mesh_from_geometry,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _spoke_origin(radius_mid: float, angle: float, x: float) -> Origin:
    return Origin(
        xyz=(x, radius_mid * cos(angle), radius_mid * sin(angle)),
        rpy=(angle, 0.0, 0.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="recessed_wall_safe")

    body_paint = model.material("body_paint", rgba=(0.17, 0.18, 0.19, 1.0))
    frame_paint = model.material("frame_paint", rgba=(0.24, 0.25, 0.26, 1.0))
    door_paint = model.material("door_paint", rgba=(0.11, 0.12, 0.13, 1.0))
    dial_metal = model.material("dial_metal", rgba=(0.74, 0.76, 0.78, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.30, 0.31, 0.33, 1.0))
    handle_metal = model.material("handle_metal", rgba=(0.67, 0.69, 0.72, 1.0))

    frame_outer_w = 0.70
    frame_outer_h = 0.50
    opening_w = 0.58
    opening_h = 0.38
    body_outer_w = 0.62
    body_outer_h = 0.42
    body_depth = 0.34
    body_wall = 0.02
    back_t = 0.012
    frame_t = 0.01
    door_panel_w = 0.628
    door_h = 0.44
    door_t = 0.044
    door_gap = 0.002
    door_axis_clear = 0.012
    hinge_axis_x = frame_t + door_gap + door_t * 0.5
    hinge_axis_y = door_axis_clear + door_panel_w * 0.5
    door_center_y = -(door_axis_clear + door_panel_w * 0.5)

    safe_body = model.part("safe_body")
    safe_body.inertial = Inertial.from_geometry(
        Box((0.40, frame_outer_w, frame_outer_h)),
        mass=42.0,
        origin=Origin(xyz=(-0.16, 0.0, 0.0)),
    )

    safe_body.visual(
        Box((back_t, body_outer_w, body_outer_h)),
        origin=Origin(xyz=(-body_depth + back_t * 0.5, 0.0, 0.0)),
        material=body_paint,
        name="back_panel",
    )
    safe_body.visual(
        Box((body_depth - back_t, body_wall, body_outer_h)),
        origin=Origin(xyz=(-(body_depth - back_t) * 0.5, -(body_outer_w * 0.5 - body_wall * 0.5), 0.0)),
        material=body_paint,
        name="left_wall",
    )
    safe_body.visual(
        Box((body_depth - back_t, body_wall, body_outer_h)),
        origin=Origin(xyz=(-(body_depth - back_t) * 0.5, body_outer_w * 0.5 - body_wall * 0.5, 0.0)),
        material=body_paint,
        name="right_wall",
    )
    safe_body.visual(
        Box((body_depth - back_t, body_outer_w - 2.0 * body_wall, body_wall)),
        origin=Origin(xyz=(-(body_depth - back_t) * 0.5, 0.0, body_outer_h * 0.5 - body_wall * 0.5)),
        material=body_paint,
        name="top_wall",
    )
    safe_body.visual(
        Box((body_depth - back_t, body_outer_w - 2.0 * body_wall, body_wall)),
        origin=Origin(xyz=(-(body_depth - back_t) * 0.5, 0.0, -(body_outer_h * 0.5 - body_wall * 0.5))),
        material=body_paint,
        name="bottom_wall",
    )

    rail_w = (frame_outer_w - opening_w) * 0.5
    rail_h = (frame_outer_h - opening_h) * 0.5
    safe_body.visual(
        Box((frame_t, rail_w, frame_outer_h)),
        origin=Origin(xyz=(frame_t * 0.5, -(frame_outer_w * 0.5 - rail_w * 0.5), 0.0)),
        material=frame_paint,
        name="frame_left_rail",
    )
    safe_body.visual(
        Box((frame_t, rail_w, frame_outer_h)),
        origin=Origin(xyz=(frame_t * 0.5, frame_outer_w * 0.5 - rail_w * 0.5, 0.0)),
        material=frame_paint,
        name="frame_right_rail",
    )
    safe_body.visual(
        Box((frame_t, opening_w, rail_h)),
        origin=Origin(xyz=(frame_t * 0.5, 0.0, frame_outer_h * 0.5 - rail_h * 0.5)),
        material=frame_paint,
        name="frame_top_rail",
    )
    safe_body.visual(
        Box((frame_t, opening_w, rail_h)),
        origin=Origin(xyz=(frame_t * 0.5, 0.0, -(frame_outer_h * 0.5 - rail_h * 0.5))),
        material=frame_paint,
        name="frame_bottom_rail",
    )

    frame_knuckle_r = 0.011
    frame_knuckle_len = 0.085
    for idx, z in enumerate((0.0825, -0.0825), start=1):
        safe_body.visual(
            Box((0.024, 0.034, frame_knuckle_len)),
            origin=Origin(xyz=(0.022, hinge_axis_y + 0.012, z)),
            material=frame_paint,
            name=f"frame_hinge_tab_{idx}",
        )
        safe_body.visual(
            Cylinder(radius=frame_knuckle_r, length=frame_knuckle_len),
            origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, z)),
            material=dark_metal,
            name=f"frame_hinge_knuckle_{idx}",
        )

    door = model.part("door")
    door.inertial = Inertial.from_geometry(
        Box((door_t, door_panel_w, door_h)),
        mass=18.0,
        origin=Origin(xyz=(0.0, door_center_y, 0.0)),
    )
    door.visual(
        Box((door_t, door_panel_w, door_h)),
        origin=Origin(xyz=(0.0, door_center_y, 0.0)),
        material=door_paint,
        name="door_panel",
    )

    for idx, (z, knuckle_len) in enumerate(((0.1625, 0.075), (0.0, 0.08), (-0.1625, 0.075)), start=1):
        door.visual(
            Box((0.020, 0.022, knuckle_len)),
            origin=Origin(xyz=(0.0, -0.006, z)),
            material=door_paint,
            name=f"door_hinge_tab_{idx}",
        )
        door.visual(
            Cylinder(radius=frame_knuckle_r, length=knuckle_len),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=dark_metal,
            name=f"door_hinge_knuckle_{idx}",
        )

    dial = model.part("dial")
    dial.inertial = Inertial.from_geometry(
        Cylinder(radius=0.045, length=0.028),
        mass=0.35,
        origin=Origin(xyz=(0.014, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
    )
    dial.visual(
        Cylinder(radius=0.008, length=0.010),
        origin=Origin(xyz=(0.005, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="dial_shaft",
    )
    dial.visual(
        Cylinder(radius=0.045, length=0.006),
        origin=Origin(xyz=(0.003, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="dial_bezel",
    )
    dial.visual(
        Cylinder(radius=0.038, length=0.022),
        origin=Origin(xyz=(0.017, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dial_metal,
        name="dial_face",
    )
    dial.visual(
        Cylinder(radius=0.016, length=0.010),
        origin=Origin(xyz=(0.030, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="dial_cap",
    )

    handle = model.part("handle")
    handle.inertial = Inertial.from_geometry(
        Box((0.04, 0.16, 0.16)),
        mass=0.8,
        origin=Origin(xyz=(0.018, 0.0, 0.0)),
    )
    handle.visual(
        Cylinder(radius=0.008, length=0.012),
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="handle_shaft",
    )
    handle.visual(
        Cylinder(radius=0.022, length=0.024),
        origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=handle_metal,
        name="handle_hub",
    )

    ring_mesh = _mesh("safe_handle_ring", TorusGeometry(radius=0.062, tube=0.007, radial_segments=18, tubular_segments=36))
    handle.visual(
        ring_mesh,
        origin=Origin(xyz=(0.019, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=handle_metal,
        name="handle_ring",
    )

    spoke_len = 0.040
    spoke_mid = 0.039
    for idx, angle in enumerate((pi / 2.0, pi / 2.0 + 2.0 * pi / 3.0, pi / 2.0 + 4.0 * pi / 3.0), start=1):
        handle.visual(
            Box((0.010, spoke_len, 0.012)),
            origin=_spoke_origin(spoke_mid, angle, 0.019),
            material=handle_metal,
            name=f"handle_spoke_{idx}",
        )
        handle.visual(
            Cylinder(radius=0.008, length=0.022),
            origin=Origin(
                xyz=(0.029, 0.062 * cos(angle), 0.062 * sin(angle)),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=handle_metal,
            name=f"handle_grip_{idx}",
        )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=safe_body,
        child=door,
        origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.2, lower=0.0, upper=2.15),
    )
    model.articulation(
        "door_to_dial",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=dial,
        origin=Origin(xyz=(door_t * 0.5, door_center_y, 0.105)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=8.0),
    )
    model.articulation(
        "door_to_handle",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=handle,
        origin=Origin(xyz=(door_t * 0.5, door_center_y, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=4.0),
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

    safe_body = object_model.get_part("safe_body")
    door = object_model.get_part("door")
    dial = object_model.get_part("dial")
    handle = object_model.get_part("handle")

    door_hinge = object_model.get_articulation("body_to_door")
    dial_spin = object_model.get_articulation("door_to_dial")
    handle_spin = object_model.get_articulation("door_to_handle")

    ctx.check(
        "door hinge is vertical and front hardware spins about the door normal",
        door_hinge.axis == (0.0, 0.0, 1.0) and dial_spin.axis == (1.0, 0.0, 0.0) and handle_spin.axis == (1.0, 0.0, 0.0),
        details=f"door_axis={door_hinge.axis}, dial_axis={dial_spin.axis}, handle_axis={handle_spin.axis}",
    )

    ctx.expect_gap(
        door,
        safe_body,
        axis="x",
        positive_elem="door_panel",
        negative_elem="frame_top_rail",
        min_gap=0.0015,
        max_gap=0.0035,
        name="door sits just proud of the front frame",
    )
    ctx.expect_overlap(
        door,
        safe_body,
        axes="yz",
        elem_a="door_panel",
        elem_b="frame_top_rail",
        min_overlap=0.025,
        name="door covers the top frame rail when closed",
    )
    ctx.expect_contact(
        dial,
        door,
        elem_a="dial_shaft",
        elem_b="door_panel",
        name="dial shaft seats on the door face",
    )
    ctx.expect_contact(
        handle,
        door,
        elem_a="handle_shaft",
        elem_b="door_panel",
        name="handle shaft seats on the door face",
    )
    ctx.expect_origin_gap(
        dial,
        handle,
        axis="z",
        min_gap=0.09,
        max_gap=0.12,
        name="combination dial sits above the center handle",
    )

    closed_panel = ctx.part_element_world_aabb(door, elem="door_panel")
    handle_pos = ctx.part_world_position(handle)
    if closed_panel is not None and handle_pos is not None:
        panel_center_y = 0.5 * (closed_panel[0][1] + closed_panel[1][1])
        panel_center_z = 0.5 * (closed_panel[0][2] + closed_panel[1][2])
        ctx.check(
            "handle sits at the door center",
            abs(handle_pos[1] - panel_center_y) <= 0.002 and abs(handle_pos[2] - panel_center_z) <= 0.002,
            details=f"handle={handle_pos}, panel_center=({panel_center_y}, {panel_center_z})",
        )

    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.35}):
        open_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door opens outward on the right-side hinge",
        closed_aabb is not None and open_aabb is not None and open_aabb[1][0] > closed_aabb[1][0] + 0.18,
        details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
