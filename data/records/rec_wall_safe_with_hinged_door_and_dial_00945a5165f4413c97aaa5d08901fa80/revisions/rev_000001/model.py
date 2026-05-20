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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_safe")

    steel = model.material("steel", rgba=(0.22, 0.23, 0.25, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.13, 0.14, 0.15, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.72, 0.74, 0.76, 1.0))
    black = model.material("black", rgba=(0.07, 0.07, 0.08, 1.0))

    outer_w = 0.36
    outer_h = 0.60
    outer_d = 0.16
    back_t = 0.010
    wall_t = 0.012
    frame_t = 0.018

    opening_w = 0.26
    opening_h = 0.48
    opening_z0 = 0.060
    opening_z1 = opening_z0 + opening_h

    body = model.part("body")
    body.visual(
        Box((outer_w, back_t, outer_h)),
        origin=Origin(xyz=(0.0, -outer_d + back_t / 2.0, outer_h / 2.0)),
        material=steel,
        name="back_panel",
    )
    body.visual(
        Box((wall_t, outer_d - back_t, outer_h)),
        origin=Origin(
            xyz=(-outer_w / 2.0 + wall_t / 2.0, -(outer_d - back_t) / 2.0, outer_h / 2.0)
        ),
        material=steel,
        name="left_wall",
    )
    body.visual(
        Box((wall_t, outer_d - back_t, outer_h)),
        origin=Origin(
            xyz=(outer_w / 2.0 - wall_t / 2.0, -(outer_d - back_t) / 2.0, outer_h / 2.0)
        ),
        material=steel,
        name="right_wall",
    )
    body.visual(
        Box((outer_w - 2.0 * wall_t, outer_d - back_t, wall_t)),
        origin=Origin(xyz=(0.0, -(outer_d - back_t) / 2.0, wall_t / 2.0)),
        material=steel,
        name="bottom_wall",
    )
    body.visual(
        Box((outer_w - 2.0 * wall_t, outer_d - back_t, wall_t)),
        origin=Origin(xyz=(0.0, -(outer_d - back_t) / 2.0, outer_h - wall_t / 2.0)),
        material=steel,
        name="top_wall",
    )

    frame_side_w = (outer_w - opening_w) / 2.0
    frame_bottom_h = opening_z0
    frame_top_h = outer_h - opening_z1

    body.visual(
        Box((frame_side_w, frame_t, opening_h)),
        origin=Origin(
            xyz=(-(opening_w / 2.0 + frame_side_w / 2.0), -frame_t / 2.0, opening_z0 + opening_h / 2.0)
        ),
        material=dark_steel,
        name="left_jamb",
    )
    body.visual(
        Box((frame_side_w, frame_t, opening_h)),
        origin=Origin(
            xyz=((opening_w / 2.0 + frame_side_w / 2.0), -frame_t / 2.0, opening_z0 + opening_h / 2.0)
        ),
        material=dark_steel,
        name="right_jamb",
    )
    body.visual(
        Box((opening_w, frame_t, frame_bottom_h)),
        origin=Origin(xyz=(0.0, -frame_t / 2.0, frame_bottom_h / 2.0)),
        material=dark_steel,
        name="bottom_rail",
    )
    body.visual(
        Box((opening_w, frame_t, frame_top_h)),
        origin=Origin(xyz=(0.0, -frame_t / 2.0, opening_z1 + frame_top_h / 2.0)),
        material=dark_steel,
        name="top_rail",
    )

    door = model.part("door")
    door_w = opening_w + 0.024
    door_h = opening_h + 0.024
    door_t = 0.028
    door_face_gap = 0.002
    door_z0 = opening_z0 - 0.012

    door.visual(
        Box((door_w, door_t, door_h)),
        origin=Origin(xyz=(door_w / 2.0, door_face_gap + door_t / 2.0, door_h / 2.0)),
        material=steel,
        name="door_panel",
    )
    door.visual(
        Box((door_w * 0.82, 0.004, door_h * 0.84)),
        origin=Origin(
            xyz=(door_w / 2.0, door_face_gap + door_t - 0.002, door_h / 2.0)
        ),
        material=dark_steel,
        name="door_face_plate",
    )
    door.visual(
        Cylinder(radius=0.008, length=0.100),
        origin=Origin(xyz=(0.0, door_face_gap + 0.010, 0.108)),
        material=satin_metal,
        name="hinge_knuckle_lower",
    )
    door.visual(
        Cylinder(radius=0.008, length=0.100),
        origin=Origin(xyz=(0.0, door_face_gap + 0.010, door_h - 0.108)),
        material=satin_metal,
        name="hinge_knuckle_upper",
    )

    body.visual(
        Cylinder(radius=0.008, length=0.120),
        origin=Origin(xyz=(-door_w / 2.0 - 0.008, 0.004, door_z0 + door_h / 2.0)),
        material=satin_metal,
        name="body_hinge_knuckle",
    )

    dial = model.part("dial")
    dial_ring_len = 0.012
    dial.visual(
        Cylinder(radius=0.033, length=dial_ring_len),
        origin=Origin(xyz=(0.0, dial_ring_len / 2.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=black,
        name="dial_ring",
    )
    dial.visual(
        Cylinder(radius=0.014, length=0.018),
        origin=Origin(xyz=(0.0, 0.009, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="dial_hub",
    )
    dial.visual(
        Box((0.005, 0.0025, 0.010)),
        origin=Origin(xyz=(0.0, dial_ring_len + 0.00125, 0.022)),
        material=satin_metal,
        name="dial_marker",
    )

    flap = model.part("document_flap")
    flap_w = 0.200
    flap_h = 0.085
    flap_t = 0.006
    flap_rod_r = 0.004
    flap.visual(
        Cylinder(radius=flap_rod_r, length=flap_w),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_metal,
        name="flap_rod",
    )
    flap.visual(
        Box((flap_w, flap_t, flap_h)),
        origin=Origin(xyz=(0.0, flap_t / 2.0, -flap_h / 2.0)),
        material=steel,
        name="flap_panel",
    )
    flap.visual(
        Box((flap_w * 0.70, flap_t * 0.75, 0.016)),
        origin=Origin(xyz=(0.0, flap_t, -flap_h * 0.62)),
        material=dark_steel,
        name="flap_pull_lip",
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(-door_w / 2.0, 0.0, door_z0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.5, lower=0.0, upper=1.75),
    )
    model.articulation(
        "door_to_dial",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=dial,
        origin=Origin(xyz=(door_w * 0.60, door_face_gap + door_t, door_h * 0.56)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )
    model.articulation(
        "body_to_document_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flap,
        origin=Origin(xyz=(0.0, -0.022, opening_z1)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=0.0, upper=1.20),
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
    dial = object_model.get_part("dial")
    flap = object_model.get_part("document_flap")
    door_hinge = object_model.get_articulation("body_to_door")
    dial_joint = object_model.get_articulation("door_to_dial")
    flap_hinge = object_model.get_articulation("body_to_document_flap")

    ctx.check("body part exists", body is not None)
    ctx.check("door part exists", door is not None)
    ctx.check("dial part exists", dial is not None)
    ctx.check("document flap part exists", flap is not None)

    ctx.check(
        "door hinge uses vertical axis",
        tuple(round(v, 6) for v in door_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"axis={door_hinge.axis}",
    )
    ctx.check(
        "dial spins about its face axis",
        tuple(round(v, 6) for v in dial_joint.axis) == (0.0, 1.0, 0.0),
        details=f"axis={dial_joint.axis}",
    )
    ctx.check(
        "flap hinge uses horizontal axis",
        tuple(round(v, 6) for v in flap_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"axis={flap_hinge.axis}",
    )

    with ctx.pose({door_hinge: 0.0, flap_hinge: 0.0}):
        ctx.expect_gap(
            door,
            body,
            axis="y",
            min_gap=0.001,
            max_gap=0.004,
            positive_elem="door_panel",
            negative_elem="top_rail",
            name="door sits just proud of the safe face",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="xz",
            min_overlap=0.27,
            elem_a="door_panel",
            name="door covers the tall front opening footprint",
        )
        ctx.expect_contact(
            dial,
            door,
            elem_a="dial_ring",
            elem_b="door_panel",
            name="dial mounts directly on the door face",
        )

    rest_flap_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
    flap_inside_opening = (
        rest_flap_aabb is not None
        and rest_flap_aabb[1][1] < -0.010
        and rest_flap_aabb[1][2] <= 0.541
    )
    ctx.check(
        "document flap stays inside the front cavity at rest",
        flap_inside_opening,
        details=f"rest_flap_aabb={rest_flap_aabb}",
    )

    closed_door_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({door_hinge: 1.45}):
        open_door_aabb = ctx.part_element_world_aabb(door, elem="door_panel")

    door_swings_out = (
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][1] > closed_door_aabb[1][1] + 0.18
    )
    ctx.check(
        "door swings outward on the left hinge",
        door_swings_out,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    closed_flap_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
    with ctx.pose({flap_hinge: 1.0}):
        open_flap_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")

    flap_moves = (
        closed_flap_aabb is not None
        and open_flap_aabb is not None
        and open_flap_aabb[1][1] > closed_flap_aabb[1][1] + 0.05
    )
    ctx.check(
        "document flap rotates away from its closed position",
        flap_moves,
        details=f"closed={closed_flap_aabb}, open={open_flap_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
