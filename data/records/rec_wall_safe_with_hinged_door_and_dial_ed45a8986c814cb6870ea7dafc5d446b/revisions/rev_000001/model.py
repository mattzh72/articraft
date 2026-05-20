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
    model = ArticulatedObject(name="flush_wall_safe")

    body_color = model.material("body_paint", rgba=(0.18, 0.18, 0.20, 1.0))
    door_color = model.material("door_paint", rgba=(0.22, 0.22, 0.24, 1.0))
    hardware_color = model.material("hardware", rgba=(0.70, 0.72, 0.75, 1.0))
    shelf_color = model.material("shelf_gray", rgba=(0.63, 0.64, 0.66, 1.0))

    outer_w = 0.44
    outer_h = 0.44
    body_d = 0.18
    wall_t = 0.025
    frame_t = 0.010
    frame_border = 0.035
    door_gap = 0.003
    door_t = 0.030

    opening_w = outer_w - 2.0 * frame_border
    opening_h = outer_h - 2.0 * frame_border
    door_w = opening_w - 2.0 * door_gap
    door_h = opening_h - 2.0 * door_gap
    cavity_w = outer_w - 2.0 * wall_t
    cavity_h = outer_h - 2.0 * wall_t
    cavity_back_y = -body_d + wall_t

    body = model.part("body")
    body.visual(
        Box((wall_t, body_d, outer_h)),
        origin=Origin(xyz=(-outer_w / 2.0 + wall_t / 2.0, -body_d / 2.0, 0.0)),
        material=body_color,
        name="body_left_wall",
    )
    body.visual(
        Box((wall_t, body_d, outer_h)),
        origin=Origin(xyz=(outer_w / 2.0 - wall_t / 2.0, -body_d / 2.0, 0.0)),
        material=body_color,
        name="body_right_wall",
    )
    body.visual(
        Box((cavity_w, body_d, wall_t)),
        origin=Origin(xyz=(0.0, -body_d / 2.0, outer_h / 2.0 - wall_t / 2.0)),
        material=body_color,
        name="body_top_wall",
    )
    body.visual(
        Box((cavity_w, body_d, wall_t)),
        origin=Origin(xyz=(0.0, -body_d / 2.0, -outer_h / 2.0 + wall_t / 2.0)),
        material=body_color,
        name="body_bottom_wall",
    )
    body.visual(
        Box((cavity_w, wall_t, cavity_h)),
        origin=Origin(xyz=(0.0, -body_d + wall_t / 2.0, 0.0)),
        material=body_color,
        name="body_back",
    )
    body.visual(
        Box((outer_w, frame_t, frame_border)),
        origin=Origin(xyz=(0.0, frame_t / 2.0, outer_h / 2.0 - frame_border / 2.0)),
        material=body_color,
        name="frame_top",
    )
    body.visual(
        Box((outer_w, frame_t, frame_border)),
        origin=Origin(xyz=(0.0, frame_t / 2.0, -outer_h / 2.0 + frame_border / 2.0)),
        material=body_color,
        name="frame_bottom",
    )
    body.visual(
        Box((frame_border, frame_t, opening_h)),
        origin=Origin(xyz=(-outer_w / 2.0 + frame_border / 2.0, frame_t / 2.0, 0.0)),
        material=body_color,
        name="frame_left",
    )
    body.visual(
        Box((frame_border, frame_t, opening_h)),
        origin=Origin(xyz=(outer_w / 2.0 - frame_border / 2.0, frame_t / 2.0, 0.0)),
        material=body_color,
        name="frame_right",
    )
    body.inertial = Inertial.from_geometry(
        Box((outer_w, body_d + frame_t, outer_h)),
        mass=34.0,
        origin=Origin(xyz=(0.0, -body_d / 2.0 + frame_t / 2.0, 0.0)),
    )

    door = model.part("door")
    door.visual(
        Box((door_w, door_t, door_h)),
        origin=Origin(xyz=(door_w / 2.0, -door_t / 2.0, 0.0)),
        material=door_color,
        name="door_slab",
    )
    door.visual(
        Box((door_w - 0.060, 0.016, door_h - 0.060)),
        origin=Origin(xyz=(door_w / 2.0 + 0.006, -door_t + 0.008, 0.0)),
        material=door_color,
        name="door_inner_plate",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_w, door_t, door_h)),
        mass=12.0,
        origin=Origin(xyz=(door_w / 2.0, -door_t / 2.0, 0.0)),
    )

    shelf = model.part("shelf")
    shelf_depth = 0.105
    shelf_t = 0.008
    shelf.visual(
        Box((cavity_w, shelf_depth, shelf_t)),
        origin=Origin(),
        material=shelf_color,
        name="shelf_panel",
    )
    shelf.inertial = Inertial.from_geometry(
        Box((cavity_w, shelf_depth, shelf_t)),
        mass=1.5,
        origin=Origin(),
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.045, length=0.012),
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=hardware_color,
        name="dial_base",
    )
    dial.visual(
        Cylinder(radius=0.036, length=0.010),
        origin=Origin(xyz=(0.0, 0.014, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=hardware_color,
        name="dial_ring",
    )
    dial.visual(
        Cylinder(radius=0.013, length=0.020),
        origin=Origin(xyz=(0.0, 0.020, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=hardware_color,
        name="dial_knob",
    )
    dial.inertial = Inertial.from_geometry(
        Cylinder(radius=0.045, length=0.030),
        mass=0.45,
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.022, length=0.018),
        origin=Origin(xyz=(0.0, 0.009, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=hardware_color,
        name="handle_hub",
    )
    handle.visual(
        Box((0.018, 0.012, 0.060)),
        origin=Origin(xyz=(0.0, 0.024, 0.015)),
        material=hardware_color,
        name="handle_stem",
    )
    handle.visual(
        Cylinder(radius=0.007, length=0.110),
        origin=Origin(xyz=(0.0, 0.028, 0.045), rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware_color,
        name="handle_bar",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.110, 0.030, 0.090)),
        mass=0.80,
        origin=Origin(xyz=(0.0, 0.015, 0.020)),
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(-opening_w / 2.0, frame_t, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.2,
            lower=0.0,
            upper=1.85,
        ),
    )
    model.articulation(
        "body_to_shelf",
        ArticulationType.FIXED,
        parent=body,
        child=shelf,
        origin=Origin(xyz=(0.0, cavity_back_y + shelf_depth / 2.0, 0.035)),
    )
    model.articulation(
        "door_to_dial",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=dial,
        origin=Origin(xyz=(door_w / 2.0, 0.0, 0.075)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0),
    )
    model.articulation(
        "door_to_handle",
        ArticulationType.REVOLUTE,
        parent=door,
        child=handle,
        origin=Origin(xyz=(door_w / 2.0, 0.0, -0.055)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-1.2,
            upper=1.2,
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

    body = object_model.get_part("body")
    door = object_model.get_part("door")
    shelf = object_model.get_part("shelf")
    dial = object_model.get_part("dial")
    handle = object_model.get_part("handle")

    door_hinge = object_model.get_articulation("body_to_door")
    dial_spin = object_model.get_articulation("door_to_dial")
    handle_spin = object_model.get_articulation("door_to_handle")

    ctx.check("body part exists", body is not None)
    ctx.check("door part exists", door is not None)
    ctx.check("shelf part exists", shelf is not None)
    ctx.check("dial part exists", dial is not None)
    ctx.check("handle part exists", handle is not None)

    ctx.check(
        "door hinge is a vertical revolute joint",
        door_hinge.articulation_type == ArticulationType.REVOLUTE and tuple(door_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"type={door_hinge.articulation_type}, axis={door_hinge.axis}",
    )
    ctx.check(
        "dial spins continuously on the door-normal axis",
        dial_spin.articulation_type == ArticulationType.CONTINUOUS and tuple(dial_spin.axis) == (0.0, 1.0, 0.0),
        details=f"type={dial_spin.articulation_type}, axis={dial_spin.axis}",
    )
    ctx.check(
        "handle rotates on a horizontal door-normal shaft",
        handle_spin.articulation_type == ArticulationType.REVOLUTE and tuple(handle_spin.axis) == (0.0, 1.0, 0.0),
        details=f"type={handle_spin.articulation_type}, axis={handle_spin.axis}",
    )

    ctx.expect_contact(
        dial,
        door,
        elem_a="dial_base",
        elem_b="door_slab",
        name="dial is mounted against the door face",
    )
    ctx.expect_contact(
        handle,
        door,
        elem_a="handle_hub",
        elem_b="door_slab",
        name="handle hub is mounted against the door face",
    )
    ctx.expect_contact(
        shelf,
        body,
        elem_a="shelf_panel",
        elem_b="body_back",
        name="shelf is supported by the back of the safe body",
    )
    ctx.expect_within(
        shelf,
        body,
        axes="xz",
        inner_elem="shelf_panel",
        margin=0.0,
        name="shelf stays within the safe body width and height",
    )

    closed_door_aabb = ctx.part_element_world_aabb(door, elem="door_slab")
    with ctx.pose({door_hinge: 1.15}):
        open_door_aabb = ctx.part_element_world_aabb(door, elem="door_slab")

    closed_center_y = None
    open_center_y = None
    if closed_door_aabb is not None:
        closed_center_y = 0.5 * (closed_door_aabb[0][1] + closed_door_aabb[1][1])
    if open_door_aabb is not None:
        open_center_y = 0.5 * (open_door_aabb[0][1] + open_door_aabb[1][1])
    ctx.check(
        "door swings outward when opened",
        closed_center_y is not None and open_center_y is not None and open_center_y > closed_center_y + 0.10,
        details=f"closed_center_y={closed_center_y}, open_center_y={open_center_y}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
