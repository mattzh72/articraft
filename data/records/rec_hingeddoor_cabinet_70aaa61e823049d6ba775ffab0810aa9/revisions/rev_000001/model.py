from __future__ import annotations

import math

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
    model = ArticulatedObject(name="detailed_hinged_cabinet")

    warm_oak = model.material("warm_oak", rgba=(0.62, 0.38, 0.18, 1.0))
    end_grain = model.material("end_grain", rgba=(0.50, 0.29, 0.13, 1.0))
    dark_interior = model.material("dark_oiled_interior", rgba=(0.25, 0.16, 0.08, 1.0))
    shadow = model.material("deep_shadow", rgba=(0.055, 0.040, 0.030, 1.0))
    brass = model.material("brushed_brass", rgba=(0.86, 0.64, 0.28, 1.0))
    dark_gap = model.material("revealed_door_gap", rgba=(0.018, 0.014, 0.010, 1.0))

    cabinet = model.part("cabinet")

    # Overall furniture proportions: about a tall sideboard / storage cabinet.
    width = 1.20
    depth = 0.50
    height = 1.80
    side_t = 0.040
    top_t = 0.050
    back_t = 0.030
    face_y = -depth / 2.0
    back_y = depth / 2.0

    # Carcase: true open-front shell, not a solid block.  Shelves and dividers
    # are visible through the working door gap/open poses.
    cabinet.visual(
        Box((side_t, depth, height)),
        origin=Origin(xyz=(-width / 2.0 + side_t / 2.0, 0.0, height / 2.0)),
        material=warm_oak,
        name="side_panel_0",
    )
    cabinet.visual(
        Box((side_t, depth, height)),
        origin=Origin(xyz=(width / 2.0 - side_t / 2.0, 0.0, height / 2.0)),
        material=warm_oak,
        name="side_panel_1",
    )
    cabinet.visual(
        Box((width, depth, top_t)),
        origin=Origin(xyz=(0.0, 0.0, top_t / 2.0)),
        material=warm_oak,
        name="bottom_panel",
    )
    cabinet.visual(
        Box((width, depth, top_t)),
        origin=Origin(xyz=(0.0, 0.0, height - top_t / 2.0)),
        material=warm_oak,
        name="top_panel",
    )
    cabinet.visual(
        Box((width, back_t, height)),
        origin=Origin(xyz=(0.0, back_y - back_t / 2.0, height / 2.0)),
        material=dark_interior,
        name="back_panel",
    )
    cabinet.visual(
        Box((0.024, depth - back_t - 0.055, height - 2.0 * top_t)),
        origin=Origin(xyz=(0.0, -0.012, height / 2.0)),
        material=dark_interior,
        name="center_divider",
    )

    shelf_depth = depth - back_t - 0.055
    shelf_y = face_y + 0.035 + shelf_depth / 2.0
    shelf_w = (width - 2.0 * side_t - 0.024) / 2.0
    for z in (0.62, 1.12):
        cabinet.visual(
            Box((shelf_w, shelf_depth, 0.028)),
            origin=Origin(xyz=(-(0.024 / 2.0 + shelf_w / 2.0), shelf_y, z)),
            material=dark_interior,
            name=f"shelf_0_{int(z * 100)}",
        )
        cabinet.visual(
            Box((shelf_w, shelf_depth, 0.028)),
            origin=Origin(xyz=((0.024 / 2.0 + shelf_w / 2.0), shelf_y, z)),
            material=dark_interior,
            name=f"shelf_1_{int(z * 100)}",
        )

    # Face frame and molding give the body the thickness and joinery of real
    # cabinet furniture.
    frame_depth = 0.034
    frame_y = face_y - frame_depth / 2.0
    cabinet.visual(
        Box((0.060, frame_depth, height - 0.10)),
        origin=Origin(xyz=(-width / 2.0 + 0.030, frame_y, height / 2.0)),
        material=warm_oak,
        name="front_stile_0",
    )
    cabinet.visual(
        Box((0.060, frame_depth, height - 0.10)),
        origin=Origin(xyz=(width / 2.0 - 0.030, frame_y, height / 2.0)),
        material=warm_oak,
        name="front_stile_1",
    )
    cabinet.visual(
        Box((0.052, frame_depth, height - 0.14)),
        origin=Origin(xyz=(0.0, frame_y, height / 2.0)),
        material=warm_oak,
        name="center_stile",
    )
    cabinet.visual(
        Box((width, frame_depth, 0.070)),
        origin=Origin(xyz=(0.0, frame_y, 0.062)),
        material=warm_oak,
        name="front_bottom_rail",
    )
    cabinet.visual(
        Box((width, frame_depth, 0.070)),
        origin=Origin(xyz=(0.0, frame_y, height - 0.062)),
        material=warm_oak,
        name="front_top_rail",
    )
    cabinet.visual(
        Box((width + 0.080, depth + 0.060, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, height + 0.0175)),
        material=end_grain,
        name="overhanging_top",
    )
    cabinet.visual(
        Box((width + 0.030, 0.060, 0.075)),
        origin=Origin(xyz=(0.0, face_y - 0.035, 0.040)),
        material=end_grain,
        name="toe_kick",
    )
    cabinet.visual(
        Box((width + 0.070, depth + 0.030, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=end_grain,
        name="plinth_shadow_foot",
    )

    # Small dark reveals behind the door seams make the front read as a real
    # assembled cabinet with interior volume.
    cabinet.visual(
        Box((0.018, 0.008, height - 0.28)),
        origin=Origin(xyz=(0.0, face_y - 0.038, height / 2.0)),
        material=dark_gap,
        name="center_reveal",
    )
    cabinet.visual(
        Box((width - 0.18, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, face_y - 0.037, 0.090)),
        material=dark_gap,
        name="bottom_reveal",
    )
    cabinet.visual(
        Box((width - 0.18, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, face_y - 0.037, height - 0.090)),
        material=dark_gap,
        name="top_reveal",
    )

    # Static half of each butt hinge: brass plates fixed to the face frame, plus
    # alternating knuckles.  The moving door carries the middle knuckles.
    hinge_x = 0.565
    hinge_y = face_y - 0.055
    hinge_barrel_x = 0.591
    hinge_radius = 0.014
    hinge_poses = (0.48, 1.32)
    for side, sx in enumerate((-1.0, 1.0)):
        for i, zc in enumerate(hinge_poses):
            cabinet.visual(
                Box((0.024, 0.058, 0.260)),
                origin=Origin(xyz=(sx * hinge_barrel_x, face_y - 0.036, zc)),
                material=brass,
                name=f"hinge_leaf_{side}_{i}",
            )
            cabinet.visual(
                Cylinder(radius=hinge_radius, length=0.070),
                origin=Origin(xyz=(sx * hinge_barrel_x, hinge_y, zc - 0.080)),
                material=brass,
                name=f"hinge_knuckle_lower_{side}_{i}",
            )
            cabinet.visual(
                Cylinder(radius=hinge_radius, length=0.070),
                origin=Origin(xyz=(sx * hinge_barrel_x, hinge_y, zc + 0.080)),
                material=brass,
                name=f"hinge_knuckle_upper_{side}_{i}",
            )

    def build_door(name: str, sign: float):
        """Create one door. sign=-1 is the negative-X door, sign=+1 positive-X."""
        door = model.part(name)
        door_w = 0.555
        door_h = 1.620
        door_t = 0.038
        # Direction from hinge toward the meeting stile in the child frame.
        inward = -sign
        panel_center_x = inward * door_w / 2.0
        handle_x = inward * (door_w - 0.090)
        hinge_side_x = inward * 0.001

        door.visual(
            Box((door_w, door_t, door_h)),
            origin=Origin(xyz=(panel_center_x, 0.0, 0.0)),
            material=warm_oak,
            name="door_slab",
        )
        # Raised frame members on the front face.
        front_y = -door_t / 2.0 - 0.006
        stile_w = 0.060
        rail_h = 0.070
        for xoff, label in ((inward * stile_w / 2.0, "hinge_stile"), (inward * (door_w - stile_w / 2.0), "meeting_stile")):
            door.visual(
                Box((stile_w, 0.014, door_h - 0.040)),
                origin=Origin(xyz=(xoff, front_y, 0.0)),
                material=end_grain,
                name=label,
            )
        for zoff, label in ((door_h / 2.0 - rail_h / 2.0, "top_rail"), (0.0, "middle_rail"), (-door_h / 2.0 + rail_h / 2.0, "bottom_rail")):
            door.visual(
                Box((door_w - 0.045, 0.014, rail_h)),
                origin=Origin(xyz=(panel_center_x, front_y, zoff)),
                material=end_grain,
                name=label,
            )
        # Two recessed floating-looking panels are actually backed by the slab.
        inset_w = door_w - 0.155
        inset_h = (door_h - 0.240) / 2.0
        for zoff, label in ((0.385, "upper_inset_panel"), (-0.385, "lower_inset_panel")):
            door.visual(
                Box((inset_w, 0.006, inset_h)),
                origin=Origin(xyz=(panel_center_x, front_y + 0.003, zoff)),
                material=dark_interior,
                name=label,
            )
            door.visual(
                Box((inset_w + 0.030, 0.008, 0.022)),
                origin=Origin(xyz=(panel_center_x, front_y - 0.004, zoff + inset_h / 2.0 + 0.018)),
                material=end_grain,
                name=f"{label}_cap_top",
            )
            door.visual(
                Box((inset_w + 0.030, 0.008, 0.022)),
                origin=Origin(xyz=(panel_center_x, front_y - 0.004, zoff - inset_h / 2.0 - 0.018)),
                material=end_grain,
                name=f"{label}_cap_bottom",
            )

        # A believable brass pull: two standoffs anchored into the slab, with a
        # vertical round grip bar between them.
        post_y = -door_t / 2.0 - 0.019
        for zoff in (-0.175, 0.175):
            door.visual(
                Cylinder(radius=0.008, length=0.038),
                origin=Origin(xyz=(handle_x, post_y, zoff), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=brass,
                name=f"handle_post_{'lower' if zoff < 0 else 'upper'}",
            )
        door.visual(
            Cylinder(radius=0.012, length=0.430),
            origin=Origin(xyz=(handle_x, -door_t / 2.0 - 0.042, 0.0)),
            material=brass,
            name="handle_grip",
        )

        # Moving hinge leaf and middle knuckles, mounted to the hinge-side stile.
        for i, zc_world in enumerate(hinge_poses):
            zc = zc_world - height / 2.0
            door.visual(
                Box((0.028, 0.012, 0.210)),
                origin=Origin(xyz=(hinge_side_x, 0.015, zc)),
                material=brass,
                name=f"door_hinge_leaf_{i}",
            )
        return door

    door_0 = build_door("door_0", sign=-1.0)
    door_1 = build_door("door_1", sign=1.0)

    # Hinges lie on the two outside vertical stiles. Positive motion opens both
    # doors outward toward the viewer, with opposite axis signs for mirrored
    # leaves.
    model.articulation(
        "cabinet_to_door_0",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door_0,
        origin=Origin(xyz=(-hinge_x, face_y - 0.070, height / 2.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.4, lower=0.0, upper=1.75),
    )
    model.articulation(
        "cabinet_to_door_1",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door_1,
        origin=Origin(xyz=(hinge_x, face_y - 0.070, height / 2.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.4, lower=0.0, upper=1.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    door_0 = object_model.get_part("door_0")
    door_1 = object_model.get_part("door_1")
    cabinet = object_model.get_part("cabinet")
    hinge_0 = object_model.get_articulation("cabinet_to_door_0")
    hinge_1 = object_model.get_articulation("cabinet_to_door_1")

    # Closed doors sit just proud of the face frame with a slim, practical
    # reveal, and the two leaves do not collide at the meeting stile.
    ctx.expect_gap(
        cabinet,
        door_0,
        axis="y",
        min_gap=0.004,
        max_gap=0.035,
        positive_elem="front_top_rail",
        negative_elem="door_slab",
        name="door_0 sits proud of face frame",
    )
    ctx.expect_gap(
        cabinet,
        door_1,
        axis="y",
        min_gap=0.004,
        max_gap=0.035,
        positive_elem="front_top_rail",
        negative_elem="door_slab",
        name="door_1 sits proud of face frame",
    )
    ctx.expect_gap(
        door_1,
        door_0,
        axis="x",
        min_gap=0.006,
        max_gap=0.055,
        positive_elem="door_slab",
        negative_elem="door_slab",
        name="clean center reveal between doors",
    )
    ctx.expect_overlap(
        door_0,
        cabinet,
        axes="z",
        min_overlap=1.50,
        elem_a="door_slab",
        elem_b="back_panel",
        name="door_0 is full cabinet height",
    )
    ctx.expect_overlap(
        door_1,
        cabinet,
        axes="z",
        min_overlap=1.50,
        elem_a="door_slab",
        elem_b="back_panel",
        name="door_1 is full cabinet height",
    )

    rest_0 = ctx.part_world_aabb(door_0)
    rest_1 = ctx.part_world_aabb(door_1)
    with ctx.pose({hinge_0: 1.20, hinge_1: 1.20}):
        open_0 = ctx.part_world_aabb(door_0)
        open_1 = ctx.part_world_aabb(door_1)
        ctx.expect_gap(
            door_1,
            door_0,
            axis="x",
            min_gap=0.10,
            positive_elem="door_slab",
            negative_elem="door_slab",
            name="opened leaves swing away from each other",
        )

    ctx.check(
        "door_0 opens outward",
        rest_0 is not None
        and open_0 is not None
        and open_0[0][1] < rest_0[0][1] - 0.20,
        details=f"rest={rest_0}, open={open_0}",
    )
    ctx.check(
        "door_1 opens outward",
        rest_1 is not None
        and open_1 is not None
        and open_1[0][1] < rest_1[0][1] - 0.20,
        details=f"rest={rest_1}, open={open_1}",
    )

    return ctx.report()


object_model = build_object_model()
