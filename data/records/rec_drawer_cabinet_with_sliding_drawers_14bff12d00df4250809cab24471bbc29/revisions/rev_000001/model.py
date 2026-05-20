from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_24_bin_parts_sorter")

    steel = model.material("powder_coated_steel", rgba=(0.58, 0.61, 0.62, 1.0))
    dark_steel = model.material("dark_slide_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    drawer_paint = model.material("blue_gray_drawer_paint", rgba=(0.30, 0.39, 0.48, 1.0))
    label_card = model.material("off_white_label_cards", rgba=(0.90, 0.87, 0.76, 1.0))
    black = model.material("black_plastic_pulls", rgba=(0.02, 0.022, 0.024, 1.0))

    body = model.part("cabinet")

    # Cabinet coordinate frame: +X points out through the drawer fronts,
    # Y spans the four columns, and Z is vertical from the floor.
    body_w = 0.80
    body_d = 0.48
    body_h = 1.65
    body_bottom = 0.12
    body_top = body_bottom + body_h
    wall_t = 0.025
    front_x = body_d / 2.0
    back_x = -body_d / 2.0
    body_z = body_bottom + body_h / 2.0

    # Main narrow, tall steel shell: side walls, bottom/top, back panel, and
    # a connected front lattice around the drawer openings.
    body.visual(
        Box((body_d, wall_t, body_h)),
        origin=Origin(xyz=(0.0, body_w / 2.0 - wall_t / 2.0, body_z)),
        material=steel,
        name="side_wall_0",
    )
    body.visual(
        Box((body_d, wall_t, body_h)),
        origin=Origin(xyz=(0.0, -body_w / 2.0 + wall_t / 2.0, body_z)),
        material=steel,
        name="side_wall_1",
    )
    body.visual(
        Box((body_d, body_w, wall_t)),
        origin=Origin(xyz=(0.0, 0.0, body_bottom + wall_t / 2.0)),
        material=steel,
        name="bottom_plate",
    )
    body.visual(
        Box((body_d, body_w, wall_t)),
        origin=Origin(xyz=(0.0, 0.0, body_top - wall_t / 2.0)),
        material=steel,
        name="top_plate",
    )
    body.visual(
        Box((wall_t, body_w, body_h)),
        origin=Origin(xyz=(back_x + wall_t / 2.0, 0.0, body_z)),
        material=steel,
        name="back_panel",
    )

    row_pitch = 0.24
    col_pitch = 0.17
    opening_bottom = 0.225
    opening_top = opening_bottom + 6 * row_pitch
    opening_half_w = 2 * col_pitch
    front_frame_depth = 0.024
    frame_x = front_x - front_frame_depth / 2.0
    bar_t = 0.010

    # Front side stiles and aprons tie the drawer lattice into the shell.
    side_stile_w = (body_w / 2.0 - wall_t) - opening_half_w
    for sign, name in ((1.0, "front_stile_0"), (-1.0, "front_stile_1")):
        y_center = sign * (opening_half_w + side_stile_w / 2.0)
        body.visual(
            Box((front_frame_depth, side_stile_w, opening_top - opening_bottom)),
            origin=Origin(xyz=(frame_x, y_center, (opening_bottom + opening_top) / 2.0)),
            material=steel,
            name=name,
        )
    body.visual(
        Box((front_frame_depth, 2 * opening_half_w + 2 * side_stile_w, opening_bottom - (body_bottom + wall_t))),
        origin=Origin(
            xyz=(
                frame_x,
                0.0,
                (body_bottom + wall_t + opening_bottom) / 2.0,
            )
        ),
        material=steel,
        name="front_lower_apron",
    )
    body.visual(
        Box((front_frame_depth, 2 * opening_half_w + 2 * side_stile_w, (body_top - wall_t) - opening_top)),
        origin=Origin(
            xyz=(
                frame_x,
                0.0,
                (opening_top + body_top - wall_t) / 2.0,
            )
        ),
        material=steel,
        name="front_upper_apron",
    )

    for i in range(1, 4):
        y = -opening_half_w + i * col_pitch
        body.visual(
            Box((front_frame_depth, bar_t, opening_top - opening_bottom)),
            origin=Origin(xyz=(frame_x, y, (opening_bottom + opening_top) / 2.0)),
            material=steel,
            name=f"column_divider_{i}",
        )
    for i in range(1, 6):
        z = opening_bottom + i * row_pitch
        body.visual(
            Box((front_frame_depth, 2 * opening_half_w + 2 * side_stile_w, bar_t)),
            origin=Origin(xyz=(frame_x, 0.0, z)),
            material=steel,
            name=f"row_divider_{i}",
        )

    # Two fixed floor feet: long left/right channel skids bolted to the base.
    for idx, y in enumerate((-0.24, 0.24)):
        body.visual(
            Box((0.62, 0.11, 0.045)),
            origin=Origin(xyz=(-0.02, y, 0.0225)),
            material=dark_steel,
            name=f"floor_foot_{idx}",
        )
        body.visual(
            Box((0.52, 0.060, 0.075)),
            origin=Origin(xyz=(-0.02, y, 0.0825)),
            material=dark_steel,
            name=f"foot_web_{idx}",
        )

    # Fixed outer slide rails mounted in every bay, one at each drawer side.
    for row in range(6):
        zc = opening_bottom + row_pitch * (row + 0.5)
        for col in range(4):
            yc = -1.5 * col_pitch + col * col_pitch
            for side, sign in enumerate((-1.0, 1.0)):
                body.visual(
                    Box((0.435, 0.006, 0.012)),
                    origin=Origin(xyz=(0.0025, yc + sign * 0.081, zc - 0.074)),
                    material=dark_steel,
                    name=f"fixed_slide_{row}_{col}_{side}",
                )

    # Removable drawers with hollow tray bodies, face pulls, label cards, and
    # moving slide members.  Each child frame sits at the drawer front center.
    face_w = 0.150
    face_h = 0.215
    face_t = 0.020
    drawer_origin_x = front_x + 0.007 + face_t / 2.0
    tray_len = 0.432
    tray_side_t = 0.008
    tray_w = 0.136
    tray_wall_h = 0.170
    bottom_t = 0.012
    tray_center_x = -face_t / 2.0 - tray_len / 2.0
    tray_front_x = -face_t / 2.0

    for row in range(6):
        zc = opening_bottom + row_pitch * (row + 0.5)
        for col in range(4):
            yc = -1.5 * col_pitch + col * col_pitch
            drawer = model.part(f"drawer_{row}_{col}")
            drawer.visual(
                Box((face_t, face_w, face_h)),
                origin=Origin(),
                material=drawer_paint,
                name="front_panel",
            )
            drawer.visual(
                Box((tray_len, tray_w, bottom_t)),
                origin=Origin(xyz=(tray_center_x, 0.0, -0.090)),
                material=drawer_paint,
                name="tray_bottom",
            )
            drawer.visual(
                Box((tray_len, tray_side_t, tray_wall_h)),
                origin=Origin(xyz=(tray_center_x, 0.068, -0.010)),
                material=drawer_paint,
                name="tray_side_0",
            )
            drawer.visual(
                Box((tray_len, tray_side_t, tray_wall_h)),
                origin=Origin(xyz=(tray_center_x, -0.068, -0.010)),
                material=drawer_paint,
                name="tray_side_1",
            )
            drawer.visual(
                Box((0.012, tray_w, tray_wall_h)),
                origin=Origin(xyz=(tray_front_x - tray_len + 0.006, 0.0, -0.010)),
                material=drawer_paint,
                name="tray_back",
            )
            drawer.visual(
                Box((0.003, 0.095, 0.030)),
                origin=Origin(xyz=(0.011, 0.0, 0.050)),
                material=label_card,
                name="label_card",
            )
            drawer.visual(
                Box((0.018, 0.010, 0.032)),
                origin=Origin(xyz=(0.014, -0.042, -0.050)),
                material=black,
                name="handle_post_0",
            )
            drawer.visual(
                Box((0.018, 0.010, 0.032)),
                origin=Origin(xyz=(0.014, 0.042, -0.050)),
                material=black,
                name="handle_post_1",
            )
            drawer.visual(
                Box((0.024, 0.095, 0.014)),
                origin=Origin(xyz=(0.027, 0.0, -0.050)),
                material=black,
                name="handle_bar",
            )
            drawer.visual(
                Box((0.360, 0.006, 0.010)),
                origin=Origin(xyz=(-0.220, 0.075, -0.074)),
                material=dark_steel,
                name="moving_slide_0",
            )
            drawer.visual(
                Box((0.360, 0.006, 0.010)),
                origin=Origin(xyz=(-0.220, -0.075, -0.074)),
                material=dark_steel,
                name="moving_slide_1",
            )

            model.articulation(
                f"cabinet_to_drawer_{row}_{col}",
                ArticulationType.PRISMATIC,
                parent=body,
                child=drawer,
                origin=Origin(xyz=(drawer_origin_x, yc, zc)),
                axis=(1.0, 0.0, 0.0),
                motion_limits=MotionLimits(effort=70.0, velocity=0.35, lower=0.0, upper=0.300),
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    drawers = [p for p in object_model.parts if p.name.startswith("drawer_")]
    drawer_joints = [j for j in object_model.articulations if j.name.startswith("cabinet_to_drawer_")]
    ctx.check("twenty four removable drawers", len(drawers) == 24, details=f"found {len(drawers)}")
    ctx.check("twenty four drawer slide joints", len(drawer_joints) == 24, details=f"found {len(drawer_joints)}")

    sample_drawer = object_model.get_part("drawer_0_0")
    sample_joint = object_model.get_articulation("cabinet_to_drawer_0_0")
    cabinet = object_model.get_part("cabinet")

    ctx.expect_gap(
        sample_drawer,
        cabinet,
        axis="x",
        positive_elem="front_panel",
        negative_elem="front_lower_apron",
        max_gap=0.010,
        max_penetration=0.0,
        name="closed drawer front sits just proud of cabinet",
    )
    ctx.expect_within(
        sample_drawer,
        cabinet,
        axes="yz",
        inner_elem="front_panel",
        margin=0.0,
        name="drawer front is contained in one cabinet bay",
    )

    rest_pos = ctx.part_world_position(sample_drawer)
    with ctx.pose({sample_joint: 0.300}):
        extended_pos = ctx.part_world_position(sample_drawer)
        ctx.expect_overlap(
            sample_drawer,
            cabinet,
            axes="x",
            elem_a="tray_bottom",
            min_overlap=0.10,
            name="extended drawer remains retained in the cabinet",
        )
    ctx.check(
        "drawer pulls outward on steel slides",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.25,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
