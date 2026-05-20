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
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    SphereGeometry,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _build_spoke_handle_mesh():
    ring = TorusGeometry(
        radius=0.056,
        tube=0.0048,
        radial_segments=16,
        tubular_segments=44,
    ).rotate_x(pi / 2.0).translate(0.0, 0.017, 0.0)
    ring.merge(
        CylinderGeometry(radius=0.024, height=0.006, radial_segments=20)
        .rotate_x(pi / 2.0)
        .translate(0.0, 0.003, 0.0)
    )
    ring.merge(
        CylinderGeometry(radius=0.018, height=0.030, radial_segments=20)
        .rotate_x(pi / 2.0)
        .translate(0.0, 0.015, 0.0)
    )
    ring.merge(
        CylinderGeometry(radius=0.012, height=0.010, radial_segments=20)
        .rotate_x(pi / 2.0)
        .translate(0.0, 0.031, 0.0)
    )

    for angle in (0.0, 2.0 * pi / 3.0, 4.0 * pi / 3.0):
        spoke = (
            CylinderGeometry(radius=0.0048, height=0.066, radial_segments=16)
            .rotate_y(pi / 2.0)
            .translate(0.034, 0.017, 0.0)
            .rotate_y(angle)
        )
        knob = SphereGeometry(radius=0.0095).translate(0.056, 0.017, 0.0).rotate_y(angle)
        ring.merge(spoke)
        ring.merge(knob)

    return ring


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="recessed_wall_safe")

    body_paint = model.material("body_paint", rgba=(0.18, 0.18, 0.20, 1.0))
    door_paint = model.material("door_paint", rgba=(0.11, 0.11, 0.12, 1.0))
    dial_metal = model.material("dial_metal", rgba=(0.73, 0.74, 0.77, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.24, 0.25, 0.27, 1.0))
    bright_steel = model.material("bright_steel", rgba=(0.82, 0.83, 0.85, 1.0))

    outer_w = 0.46
    outer_h = 0.46
    body_depth = 0.34
    wall_t = 0.016
    frame_t = 0.020
    border = 0.055
    opening_w = outer_w - 2.0 * border
    opening_h = outer_h - 2.0 * border

    door_clearance = 0.003
    door_w = opening_w - 2.0 * door_clearance
    door_h = opening_h - 2.0 * door_clearance
    door_t = 0.038
    hinge_axis_offset = 0.010
    door_center_x = hinge_axis_offset + door_w / 2.0

    safe_body = model.part("safe_body")
    safe_body.visual(
        Box((outer_w, wall_t, outer_h)),
        origin=Origin(xyz=(0.0, -body_depth + wall_t / 2.0, 0.0)),
        material=body_paint,
        name="back_shell",
    )
    safe_body.visual(
        Box((wall_t, body_depth, outer_h)),
        origin=Origin(xyz=(-outer_w / 2.0 + wall_t / 2.0, -body_depth / 2.0, 0.0)),
        material=body_paint,
        name="left_shell",
    )
    safe_body.visual(
        Box((wall_t, body_depth, outer_h)),
        origin=Origin(xyz=(outer_w / 2.0 - wall_t / 2.0, -body_depth / 2.0, 0.0)),
        material=body_paint,
        name="right_shell",
    )
    safe_body.visual(
        Box((outer_w - 2.0 * wall_t, body_depth, wall_t)),
        origin=Origin(xyz=(0.0, -body_depth / 2.0, outer_h / 2.0 - wall_t / 2.0)),
        material=body_paint,
        name="top_shell",
    )
    safe_body.visual(
        Box((outer_w - 2.0 * wall_t, body_depth, wall_t)),
        origin=Origin(xyz=(0.0, -body_depth / 2.0, -outer_h / 2.0 + wall_t / 2.0)),
        material=body_paint,
        name="bottom_shell",
    )
    safe_body.visual(
        Box((outer_w, frame_t, border)),
        origin=Origin(xyz=(0.0, -frame_t / 2.0, outer_h / 2.0 - border / 2.0)),
        material=body_paint,
        name="frame_top",
    )
    safe_body.visual(
        Box((outer_w, frame_t, border)),
        origin=Origin(xyz=(0.0, -frame_t / 2.0, -outer_h / 2.0 + border / 2.0)),
        material=body_paint,
        name="frame_bottom",
    )
    safe_body.visual(
        Box((border, frame_t, opening_h)),
        origin=Origin(xyz=(-outer_w / 2.0 + border / 2.0, -frame_t / 2.0, 0.0)),
        material=body_paint,
        name="frame_left",
    )
    safe_body.visual(
        Box((border, frame_t, opening_h)),
        origin=Origin(xyz=(outer_w / 2.0 - border / 2.0, -frame_t / 2.0, 0.0)),
        material=body_paint,
        name="frame_right",
    )
    safe_body.visual(
        Box((0.016, 0.012, 0.190)),
        origin=Origin(xyz=(-door_center_x - 0.004, -0.006, 0.0)),
        material=dark_metal,
        name="frame_hinge_leaf",
    )
    safe_body.inertial = Inertial.from_geometry(
        Box((outer_w, body_depth, outer_h)),
        mass=24.0,
        origin=Origin(xyz=(0.0, -body_depth / 2.0, 0.0)),
    )

    door = model.part("door")
    door.visual(
        Box((door_w, door_t, door_h)),
        origin=Origin(xyz=(door_center_x, door_t / 2.0, 0.0)),
        material=door_paint,
        name="door_plate",
    )
    door.visual(
        Box((door_w - 0.065, door_t * 0.62, door_h - 0.065)),
        origin=Origin(xyz=(door_center_x, door_t * 0.32, 0.0)),
        material=body_paint,
        name="door_inner_slab",
    )
    door.visual(
        Box((0.016, 0.012, 0.190)),
        origin=Origin(xyz=(0.008, 0.006, 0.0)),
        material=dark_metal,
        name="door_hinge_leaf",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_w, door_t, door_h)),
        mass=16.0,
        origin=Origin(xyz=(door_center_x, door_t / 2.0, 0.0)),
    )

    dial = model.part("combination_dial")
    dial.visual(
        Cylinder(radius=0.031, length=0.006),
        origin=Origin(xyz=(0.0, 0.003, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="dial_backing",
    )
    dial.visual(
        Cylinder(radius=0.048, length=0.016),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dial_metal,
        name="dial_body",
    )
    dial.visual(
        Cylinder(radius=0.055, length=0.006),
        origin=Origin(xyz=(0.0, 0.017, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=bright_steel,
        name="dial_grip",
    )
    dial.visual(
        Cylinder(radius=0.017, length=0.008),
        origin=Origin(xyz=(0.0, 0.020, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="dial_cap",
    )
    dial.inertial = Inertial.from_geometry(
        Cylinder(radius=0.055, length=0.024),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.012, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
    )

    handle = model.part("spoke_handle")
    handle.visual(
        _save_mesh("safe_spoke_handle", _build_spoke_handle_mesh()),
        material=bright_steel,
        name="handle_wheel",
    )
    handle.visual(
        Cylinder(radius=0.024, length=0.006),
        origin=Origin(xyz=(0.0, 0.003, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="handle_backplate",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.135, 0.045, 0.135)),
        mass=0.7,
        origin=Origin(xyz=(0.0, 0.018, 0.0)),
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=safe_body,
        child=door,
        origin=Origin(xyz=(-door_center_x, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.2,
            lower=0.0,
            upper=1.7,
        ),
    )
    model.articulation(
        "door_to_dial",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=dial,
        origin=Origin(xyz=(door_center_x, door_t, 0.060)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )
    model.articulation(
        "door_to_handle",
        ArticulationType.REVOLUTE,
        parent=door,
        child=handle,
        origin=Origin(xyz=(door_center_x, door_t, -0.072)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.5,
            lower=-1.1,
            upper=1.1,
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

    safe_body = object_model.get_part("safe_body")
    door = object_model.get_part("door")
    dial = object_model.get_part("combination_dial")
    handle = object_model.get_part("spoke_handle")

    door_hinge = object_model.get_articulation("body_to_door")
    dial_spin = object_model.get_articulation("door_to_dial")
    handle_turn = object_model.get_articulation("door_to_handle")

    ctx.check(
        "door uses a vertical side hinge",
        door_hinge.articulation_type == ArticulationType.REVOLUTE and door_hinge.axis == (0.0, 0.0, 1.0),
        details=f"type={door_hinge.articulation_type}, axis={door_hinge.axis}",
    )
    ctx.check(
        "dial spins continuously about the door-normal shaft",
        dial_spin.articulation_type == ArticulationType.CONTINUOUS
        and dial_spin.axis == (0.0, 1.0, 0.0)
        and dial_spin.motion_limits is not None
        and dial_spin.motion_limits.lower is None
        and dial_spin.motion_limits.upper is None,
        details=f"type={dial_spin.articulation_type}, axis={dial_spin.axis}, limits={dial_spin.motion_limits}",
    )
    ctx.check(
        "handle rotates on a coaxial door shaft",
        handle_turn.articulation_type == ArticulationType.REVOLUTE
        and handle_turn.axis == (0.0, 1.0, 0.0),
        details=f"type={handle_turn.articulation_type}, axis={handle_turn.axis}",
    )

    with ctx.pose({door_hinge: 0.0, handle_turn: 0.0}):
        ctx.expect_gap(
            door,
            safe_body,
            axis="y",
            positive_elem="door_plate",
            negative_elem="frame_left",
            min_gap=0.0,
            max_gap=0.0005,
            name="door back face sits on the frame plane when closed",
        )
        ctx.expect_contact(
            dial,
            door,
            elem_a="dial_backing",
            elem_b="door_plate",
            name="dial is mounted to the door face",
        )
        ctx.expect_contact(
            handle,
            door,
            elem_a="handle_backplate",
            elem_b="door_plate",
            name="handle is mounted to the door face",
        )
        ctx.expect_origin_distance(
            dial,
            handle,
            axes="x",
            max_dist=0.001,
            name="dial and handle share the same vertical centerline",
        )
        ctx.expect_origin_gap(
            dial,
            handle,
            axis="z",
            min_gap=0.11,
            max_gap=0.15,
            name="dial sits above the spoke handle",
        )

    closed_aabb = None
    open_aabb = None
    with ctx.pose({door_hinge: 0.0}):
        closed_aabb = ctx.part_element_world_aabb(door, elem="door_plate")
    with ctx.pose({door_hinge: 1.3}):
        ctx.expect_contact(
            door,
            safe_body,
            elem_a="door_hinge_leaf",
            elem_b="frame_hinge_leaf",
            name="hinge leaves stay aligned when the door is open",
        )
        ctx.expect_gap(
            door,
            safe_body,
            axis="y",
            positive_elem="door_plate",
            negative_elem="frame_left",
            min_gap=0.008,
            name="open door plate clears the frame",
        )
        open_aabb = ctx.part_element_world_aabb(door, elem="door_plate")

    ctx.check(
        "positive hinge motion swings the door outward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][1] > closed_aabb[1][1] + 0.12,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
