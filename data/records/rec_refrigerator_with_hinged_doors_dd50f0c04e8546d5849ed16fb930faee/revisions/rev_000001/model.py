from __future__ import annotations

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
    model = ArticulatedObject(name="stainless_french_door_refrigerator")

    stainless = model.material("brushed_stainless", rgba=(0.66, 0.68, 0.67, 1.0))
    stainless_dark = model.material("shadowed_stainless", rgba=(0.46, 0.48, 0.48, 1.0))
    seam_dark = model.material("black_recess", rgba=(0.02, 0.023, 0.025, 1.0))
    rail_metal = model.material("galvanized_slide", rgba=(0.55, 0.57, 0.58, 1.0))
    bin_plastic = model.material("smoke_plastic", rgba=(0.70, 0.78, 0.82, 0.55))

    width = 0.91
    depth = 0.70
    height = 1.82
    side_t = 0.035
    back_t = 0.030
    front_y = -depth / 2.0 + 0.015
    back_y = depth / 2.0 - back_t / 2.0
    hinge_y = -0.405
    outer_x = width / 2.0

    cabinet = model.part("cabinet")

    # Tall hollow cabinet carcass: side walls, back, top, bottom, and the front
    # rails leave the refrigerator and freezer openings open for the doors,
    # bins, drawer tub, and slide hardware.
    cabinet.visual(
        Box((side_t, depth - 0.010, height)),
        origin=Origin(xyz=(-outer_x + side_t / 2.0, 0.010, height / 2.0)),
        material=stainless,
        name="side_wall_0",
    )
    cabinet.visual(
        Box((side_t, depth - 0.010, height)),
        origin=Origin(xyz=(outer_x - side_t / 2.0, 0.010, height / 2.0)),
        material=stainless,
        name="side_wall_1",
    )
    cabinet.visual(
        Box((width, back_t, height)),
        origin=Origin(xyz=(0.0, back_y, height / 2.0)),
        material=stainless_dark,
        name="back_panel",
    )
    cabinet.visual(
        Box((width, depth, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, height - 0.0275)),
        material=stainless,
        name="top_cap",
    )
    cabinet.visual(
        Box((width, depth, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=stainless_dark,
        name="bottom_plinth",
    )
    cabinet.visual(
        Box((width - 2 * side_t, 0.055, 0.040)),
        origin=Origin(xyz=(0.0, front_y + 0.0275, 0.680)),
        material=seam_dark,
        name="middle_seam_rail",
    )
    cabinet.visual(
        Box((width - 2 * side_t, 0.060, 0.105)),
        origin=Origin(xyz=(0.0, front_y + 0.030, 1.735)),
        material=seam_dark,
        name="recessed_header",
    )
    cabinet.visual(
        Box((0.018, 0.045, 1.000)),
        origin=Origin(xyz=(0.0, front_y + 0.022, 1.190)),
        material=seam_dark,
        name="center_mullion",
    )

    # Rounded stainless front vertical cabinet edges, set just behind the door
    # back planes so they are visible from the sides without colliding with the
    # articulated doors.
    for idx, x in enumerate((-outer_x + side_t / 2.0, outer_x - side_t / 2.0)):
        cabinet.visual(
            Cylinder(radius=0.018, length=height - 0.12),
            origin=Origin(xyz=(x, front_y + 0.020, height / 2.0 + 0.010)),
            material=stainless,
            name=f"rounded_edge_{idx}",
        )

    # Recessed metal channel guides inside the lower freezer bay.
    for sign, web_name, top_name, bottom_name in (
        (-1.0, "guide_web_0", "guide_lip_top_0", "guide_lip_bottom_0"),
        (1.0, "guide_web_1", "guide_lip_top_1", "guide_lip_bottom_1"),
    ):
        x_web = sign * (outer_x - side_t - 0.005)
        x_lip = sign * (outer_x - side_t - 0.022)
        cabinet.visual(
            Box((0.010, 0.560, 0.045)),
            origin=Origin(xyz=(x_web, -0.050, 0.370)),
            material=rail_metal,
            name=web_name,
        )
        cabinet.visual(
            Box((0.036, 0.560, 0.008)),
            origin=Origin(xyz=(x_lip, -0.050, 0.394)),
            material=rail_metal,
            name=top_name,
        )
        cabinet.visual(
            Box((0.036, 0.560, 0.008)),
            origin=Origin(xyz=(x_lip, -0.050, 0.346)),
            material=rail_metal,
            name=bottom_name,
        )

    # Fixed cabinet-side hinge leaves at the top and bottom of both outer edges.
    for side_idx, sign in enumerate((-1.0, 1.0)):
        for hinge_idx, z in enumerate((0.810, 1.570)):
            cabinet.visual(
                Box((0.006, 0.045, 0.125)),
                origin=Origin(xyz=(sign * (outer_x + 0.003), front_y - 0.020, z)),
                material=rail_metal,
                name=f"cabinet_hinge_{side_idx}_{hinge_idx}",
            )

    upper_bottom = 0.700
    upper_height = 0.980
    upper_center_z = upper_bottom + upper_height / 2.0
    center_gap = 0.012
    door_w = (width - center_gap) / 2.0
    door_t = 0.065

    def add_refrigerator_door(name: str, sign: float) -> object:
        """Create one French-door leaf in a hinge-line local frame.

        sign=-1 is the door on the negative-X side; sign=+1 is the mirrored
        door on the positive-X side.  Both local frames put the hinge at local
        x=0 and the stainless slab extends toward the center seam.
        """

        door = model.part(name)
        inward = -sign
        panel_center_x = inward * door_w / 2.0
        handle_center_x = inward * (door_w - 0.115)

        door.visual(
            Box((door_w, door_t, upper_height)),
            origin=Origin(xyz=(panel_center_x, door_t / 2.0, 0.0)),
            material=stainless,
            name="outer_face",
        )
        # Dark vertical gasket at the leading seam.
        door.visual(
            Box((0.008, 0.004, upper_height - 0.020)),
            origin=Origin(xyz=(inward * (door_w - 0.004), -0.002, 0.0)),
            material=seam_dark,
            name="leading_gasket",
        )
        # Subtle raised brush bands make the slab read as stainless without
        # turning the door into disconnected decorative geometry.
        for band_idx, z in enumerate((-0.320, -0.120, 0.120, 0.320)):
            door.visual(
                Box((door_w - 0.050, 0.001, 0.003)),
                origin=Origin(xyz=(panel_center_x, -0.0005, z)),
                material=stainless_dark,
                name=f"brush_band_{band_idx}",
            )

        # Flat horizontal handle near the center leading edge, with two proud
        # mounts connecting it to the stainless front face.
        door.visual(
            Box((0.230, 0.026, 0.032)),
            origin=Origin(xyz=(handle_center_x, -0.060, 0.070)),
            material=stainless_dark,
            name="handle_bar",
        )
        for mount_idx, offset in enumerate((-0.075, 0.075)):
            door.visual(
                Box((0.026, 0.060, 0.046)),
                origin=Origin(xyz=(handle_center_x + inward * offset, -0.030, 0.070)),
                material=stainless_dark,
                name=f"handle_mount_{mount_idx}",
            )

        # Door-side hinge barrels and leaves at the top and bottom.  The barrel
        # centers sit on the joint axis, so the visible pin hardware rotates
        # credibly with the leaf.
        for hinge_idx, z in enumerate((-0.380, 0.380)):
            door.visual(
                Cylinder(radius=0.011, length=0.125),
                origin=Origin(xyz=(0.0, 0.0, z)),
                material=rail_metal,
                name=f"hinge_barrel_{hinge_idx}",
            )
            door.visual(
                Box((0.040, 0.006, 0.110)),
                origin=Origin(xyz=(inward * 0.020, 0.003, z)),
                material=rail_metal,
                name=f"hinge_leaf_{hinge_idx}",
            )

        # Flat panel door bins fixed to the inside face.  Each shelf has a ledge
        # touching the door liner plus a small front lip and end cheeks.
        for shelf_idx, z in enumerate((-0.250, 0.080)):
            shelf_x = panel_center_x
            shelf_w = door_w * 0.68
            door.visual(
                Box((shelf_w, 0.090, 0.014)),
                origin=Origin(xyz=(shelf_x, door_t + 0.045, z)),
                material=bin_plastic,
                name=f"bin_shelf_{shelf_idx}",
            )
            door.visual(
                Box((shelf_w, 0.012, 0.055)),
                origin=Origin(xyz=(shelf_x, door_t + 0.084, z + 0.027)),
                material=bin_plastic,
                name=f"bin_lip_{shelf_idx}",
            )
            for cap_idx, cap_sign in enumerate((-1.0, 1.0)):
                door.visual(
                    Box((0.014, 0.090, 0.055)),
                    origin=Origin(
                        xyz=(
                            shelf_x + cap_sign * shelf_w / 2.0,
                            door_t + 0.045,
                            z + 0.027,
                        )
                    ),
                    material=bin_plastic,
                    name=f"bin_end_{shelf_idx}_{cap_idx}",
                )
        return door

    door_0 = add_refrigerator_door("door_0", sign=-1.0)
    door_1 = add_refrigerator_door("door_1", sign=1.0)

    model.articulation(
        "cabinet_to_door_0",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door_0,
        origin=Origin(xyz=(-outer_x, hinge_y, upper_center_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.5, lower=0.0, upper=1.85),
    )
    model.articulation(
        "cabinet_to_door_1",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door_1,
        origin=Origin(xyz=(outer_x, hinge_y, upper_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.5, lower=0.0, upper=1.85),
    )

    freezer_drawer = model.part("freezer_drawer")
    drawer_height = 0.560
    drawer_center_z = 0.380
    freezer_drawer.visual(
        Box((width - 0.030, door_t, drawer_height)),
        origin=Origin(xyz=(0.0, door_t / 2.0, 0.0)),
        material=stainless,
        name="drawer_face",
    )
    for band_idx, z in enumerate((-0.190, 0.0, 0.190)):
        freezer_drawer.visual(
            Box((width - 0.090, 0.001, 0.003)),
            origin=Origin(xyz=(0.0, -0.0005, z)),
            material=stainless_dark,
            name=f"drawer_brush_{band_idx}",
        )
    freezer_drawer.visual(
        Box((0.720, 0.028, 0.034)),
        origin=Origin(xyz=(0.0, -0.062, 0.080)),
        material=stainless_dark,
        name="drawer_handle",
    )
    for mount_idx, x in enumerate((-0.275, 0.0, 0.275)):
        freezer_drawer.visual(
            Box((0.030, 0.062, 0.048)),
            origin=Origin(xyz=(x, -0.031, 0.080)),
            material=stainless_dark,
            name=f"drawer_mount_{mount_idx}",
        )

    # Drawer tub and its side runners remain hidden behind the full-width face
    # when closed, but slide on the visible cabinet-mounted channel guides.
    freezer_drawer.visual(
        Box((0.720, 0.500, 0.025)),
        origin=Origin(xyz=(0.0, 0.315, -0.205)),
        material=seam_dark,
        name="tub_floor",
    )
    freezer_drawer.visual(
        Box((0.025, 0.500, 0.320)),
        origin=Origin(xyz=(-0.3650, 0.315, -0.045)),
        material=seam_dark,
        name="tub_side_0",
    )
    freezer_drawer.visual(
        Box((0.025, 0.500, 0.320)),
        origin=Origin(xyz=(0.3650, 0.315, -0.045)),
        material=seam_dark,
        name="tub_side_1",
    )
    freezer_drawer.visual(
        Box((0.720, 0.025, 0.320)),
        origin=Origin(xyz=(0.0, 0.565, -0.045)),
        material=seam_dark,
        name="tub_back",
    )
    for x, runner_name in ((-0.393, "drawer_runner_0"), (0.393, "drawer_runner_1")):
        freezer_drawer.visual(
            Box((0.034, 0.500, 0.014)),
            origin=Origin(xyz=(x, 0.315, -0.010)),
            material=rail_metal,
            name=runner_name,
        )

    model.articulation(
        "cabinet_to_drawer",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=freezer_drawer,
        origin=Origin(xyz=(0.0, hinge_y, drawer_center_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.420),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    door_0 = object_model.get_part("door_0")
    door_1 = object_model.get_part("door_1")
    drawer = object_model.get_part("freezer_drawer")
    hinge_0 = object_model.get_articulation("cabinet_to_door_0")
    hinge_1 = object_model.get_articulation("cabinet_to_door_1")
    slide = object_model.get_articulation("cabinet_to_drawer")

    ctx.expect_gap(
        door_1,
        door_0,
        axis="x",
        min_gap=0.006,
        max_gap=0.020,
        positive_elem="outer_face",
        negative_elem="outer_face",
        name="equal doors keep a narrow center seam",
    )
    ctx.expect_gap(
        door_0,
        drawer,
        axis="z",
        min_gap=0.025,
        max_gap=0.055,
        positive_elem="outer_face",
        negative_elem="drawer_face",
        name="upper doors align above freezer seam",
    )
    ctx.expect_within(
        drawer,
        cabinet,
        axes="x",
        margin=0.010,
        inner_elem="drawer_face",
        outer_elem="top_cap",
        name="drawer face stays within cabinet width reference",
    )
    ctx.expect_overlap(
        drawer,
        cabinet,
        axes="y",
        min_overlap=0.300,
        elem_a="drawer_runner_0",
        elem_b="guide_web_0",
        name="freezer runner remains engaged in side guide",
    )

    closed_drawer_pos = ctx.part_world_position(drawer)
    closed_0 = ctx.part_element_world_aabb(door_0, elem="outer_face")
    closed_1 = ctx.part_element_world_aabb(door_1, elem="outer_face")
    with ctx.pose({hinge_0: 1.10}):
        opened_0 = ctx.part_element_world_aabb(door_0, elem="outer_face")
    with ctx.pose({hinge_1: 1.10}):
        opened_1 = ctx.part_element_world_aabb(door_1, elem="outer_face")
    with ctx.pose({slide: 0.380}):
        extended_drawer_pos = ctx.part_world_position(drawer)
        ctx.expect_overlap(
            drawer,
            cabinet,
            axes="y",
            min_overlap=0.080,
            elem_a="drawer_runner_0",
            elem_b="guide_web_0",
            name="extended drawer runner is still captured",
        )

    ctx.check(
        "door_0 swings outward",
        closed_0 is not None
        and opened_0 is not None
        and opened_0[0][1] < closed_0[0][1] - 0.050,
        details=f"closed={closed_0}, opened={opened_0}",
    )
    ctx.check(
        "door_1 swings outward",
        closed_1 is not None
        and opened_1 is not None
        and opened_1[0][1] < closed_1[0][1] - 0.050,
        details=f"closed={closed_1}, opened={opened_1}",
    )
    ctx.check(
        "freezer drawer slides forward",
        closed_drawer_pos is not None
        and extended_drawer_pos is not None
        and extended_drawer_pos[1] < closed_drawer_pos[1] - 0.300,
        details=f"closed={closed_drawer_pos}, extended={extended_drawer_pos}",
    )

    return ctx.report()


object_model = build_object_model()
