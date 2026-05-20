from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="library_card_catalog_desk")

    oak = Material("oiled_oak", color=(0.56, 0.34, 0.16, 1.0))
    dark_oak = Material("dark_end_grain", color=(0.32, 0.19, 0.09, 1.0))
    brass = Material("aged_brass", color=(0.82, 0.62, 0.28, 1.0))
    paper = Material("index_card_paper", color=(0.93, 0.89, 0.75, 1.0))
    label = Material("cream_label", color=(0.98, 0.94, 0.78, 1.0))

    desk = model.part("desk")

    # Overall scale: a real standing library catalog desk with a broad writing top.
    width = 1.20
    depth = 0.60
    cabinet_bottom = 0.10
    cabinet_top = 0.78
    cabinet_height = cabinet_top - cabinet_bottom
    board_t = 0.035
    front_y = -depth / 2.0
    back_y = depth / 2.0

    def add_box(part, name, size, xyz, material):
        part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)

    # Wide desk work surface with darker edge banding and modest side overhangs.
    add_box(desk, "work_surface", (1.65, 0.82, 0.070), (0.0, -0.03, 0.815), oak)
    add_box(desk, "front_edge", (1.67, 0.030, 0.080), (0.0, -0.455, 0.815), dark_oak)
    add_box(desk, "rear_edge", (1.67, 0.025, 0.065), (0.0, 0.380, 0.812), dark_oak)
    add_box(desk, "side_edge_0", (0.030, 0.82, 0.075), (-0.835, -0.03, 0.812), dark_oak)
    add_box(desk, "side_edge_1", (0.030, 0.82, 0.075), (0.835, -0.03, 0.812), dark_oak)
    add_box(desk, "side_brace_0", (0.045, 0.34, 0.090), (-0.705, -0.19, 0.735), oak)
    add_box(desk, "side_brace_1", (0.045, 0.34, 0.090), (0.705, -0.19, 0.735), oak)

    # Cabinet carcass: real open-front frame rather than a solid block.
    add_box(desk, "side_panel_0", (board_t, depth, cabinet_height), (-width / 2.0 + board_t / 2.0, 0.0, 0.44), oak)
    add_box(desk, "side_panel_1", (board_t, depth, cabinet_height), (width / 2.0 - board_t / 2.0, 0.0, 0.44), oak)
    add_box(desk, "top_panel", (width, depth, board_t), (0.0, 0.0, cabinet_top - board_t / 2.0), oak)
    add_box(desk, "bottom_panel", (width, depth, board_t), (0.0, 0.0, cabinet_bottom + board_t / 2.0), oak)
    add_box(desk, "back_panel", (width, board_t, cabinet_height), (0.0, back_y - board_t / 2.0, 0.44), oak)
    # The recessed plinth reaches the underside of the cabinet so the base reads
    # as supported rather than as a detached foot block.
    add_box(desk, "toe_plinth", (1.10, 0.52, 0.080), (0.0, 0.015, 0.060), dark_oak)

    inner_x_min = -width / 2.0 + board_t
    inner_x_max = width / 2.0 - board_t
    inner_z_min = cabinet_bottom + board_t
    inner_z_max = cabinet_top - board_t
    inner_w = inner_x_max - inner_x_min
    inner_h = inner_z_max - inner_z_min
    divider_t = 0.025
    columns = 4
    rows = 3
    opening_w = (inner_w - divider_t * (columns - 1)) / columns
    opening_h = (inner_h - divider_t * (rows - 1)) / rows

    col_centers = [
        inner_x_min + opening_w / 2.0 + c * (opening_w + divider_t)
        for c in range(columns)
    ]
    row_centers = [
        inner_z_min + opening_h / 2.0 + r * (opening_h + divider_t)
        for r in range(rows)
    ]

    for c in range(1, columns):
        x = inner_x_min + c * opening_w + (c - 0.5) * divider_t
        add_box(desk, f"vertical_divider_{c}", (divider_t, depth, inner_h), (x, 0.0, (inner_z_min + inner_z_max) / 2.0), oak)

    for r in range(1, rows):
        z = inner_z_min + r * opening_h + (r - 0.5) * divider_t
        add_box(desk, f"horizontal_divider_{r}", (width, depth, divider_t), (0.0, 0.0, z), oak)

    face_t = 0.024
    face_w = opening_w - 0.012
    face_h = opening_h - 0.014
    drawer_w = face_w - 0.025
    drawer_h = face_h - 0.026
    drawer_depth = 0.500
    wall_t = 0.012
    bottom_t = 0.012
    rail_w = 0.018
    rail_depth = 0.490
    rail_y = front_y + 0.015 + rail_depth / 2.0
    pull_y = -face_t / 2.0 - 0.026

    for r, zc in enumerate(row_centers):
        opening_bottom = zc - opening_h / 2.0
        drawer_bottom_z = zc - drawer_h / 2.0
        rail_bottom = opening_bottom - 0.001
        rail_h = drawer_bottom_z - rail_bottom
        rail_z = rail_bottom + rail_h / 2.0
        for c, xc in enumerate(col_centers):
            # Each drawer has its own pair of wooden guide rails fixed inside the cabinet.
            for side, sx in enumerate((-1.0, 1.0)):
                rail_x = xc + sx * (drawer_w / 2.0 - 0.020)
                add_box(
                    desk,
                    f"guide_{r}_{c}_{side}",
                    (rail_w, rail_depth, rail_h),
                    (rail_x, rail_y, rail_z),
                    dark_oak,
                )

            drawer = model.part(f"drawer_{r}_{c}")

            # The child frame is centered on the closed front panel.  The tray extends
            # back into the cabinet along +Y and can slide outward along -Y.
            add_box(drawer, "front", (face_w, face_t, face_h), (0.0, 0.0, 0.0), oak)
            add_box(
                drawer,
                "tray_bottom",
                (drawer_w, drawer_depth, bottom_t),
                (0.0, face_t / 2.0 + drawer_depth / 2.0, -drawer_h / 2.0 + bottom_t / 2.0),
                oak,
            )
            add_box(
                drawer,
                "side_wall_0",
                (wall_t, drawer_depth, drawer_h),
                (-drawer_w / 2.0 + wall_t / 2.0, face_t / 2.0 + drawer_depth / 2.0, 0.0),
                oak,
            )
            add_box(
                drawer,
                "side_wall_1",
                (wall_t, drawer_depth, drawer_h),
                (drawer_w / 2.0 - wall_t / 2.0, face_t / 2.0 + drawer_depth / 2.0, 0.0),
                oak,
            )
            add_box(
                drawer,
                "back_wall",
                (drawer_w, wall_t, drawer_h),
                (0.0, face_t / 2.0 + drawer_depth - wall_t / 2.0, 0.0),
                oak,
            )
            add_box(
                drawer,
                "card_stack",
                (drawer_w - 0.040, 0.320, 0.080),
                (0.0, face_t / 2.0 + 0.205, -drawer_h / 2.0 + bottom_t + 0.040),
                paper,
            )

            # Brass label frame and pull hardware mounted through the front face.
            metal_t = 0.004
            label_y = -face_t / 2.0 - metal_t / 2.0 + 0.0005
            add_box(drawer, "label_card", (0.112, metal_t, 0.034), (0.0, label_y - 0.0002, 0.030), label)
            add_box(drawer, "label_top", (0.126, metal_t, 0.006), (0.0, label_y, 0.050), brass)
            add_box(drawer, "label_bottom", (0.126, metal_t, 0.006), (0.0, label_y, 0.010), brass)
            add_box(drawer, "label_side_0", (0.006, metal_t, 0.046), (-0.063, label_y, 0.030), brass)
            add_box(drawer, "label_side_1", (0.006, metal_t, 0.046), (0.063, label_y, 0.030), brass)
            add_box(drawer, "pull_post_0", (0.014, 0.026, 0.018), (-0.044, -face_t / 2.0 - 0.013, -0.040), brass)
            add_box(drawer, "pull_post_1", (0.014, 0.026, 0.018), (0.044, -face_t / 2.0 - 0.013, -0.040), brass)
            drawer.visual(
                Cylinder(radius=0.008, length=0.115),
                origin=Origin(xyz=(0.0, pull_y, -0.040), rpy=(0.0, 1.5707963268, 0.0)),
                material=brass,
                name="pull_bar",
            )

            model.articulation(
                f"drawer_slide_{r}_{c}",
                ArticulationType.PRISMATIC,
                parent=desk,
                child=drawer,
                origin=Origin(xyz=(xc, front_y - face_t / 2.0, zc)),
                axis=(0.0, -1.0, 0.0),
                motion_limits=MotionLimits(effort=45.0, velocity=0.35, lower=0.0, upper=0.220),
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    desk = object_model.get_part("desk")
    drawer_names = [f"drawer_{r}_{c}" for r in range(3) for c in range(4)]
    slide_names = [f"drawer_slide_{r}_{c}" for r in range(3) for c in range(4)]

    ctx.check("twelve separate catalog drawers", len(drawer_names) == 12)
    ctx.check(
        "twelve prismatic drawer slides",
        all(object_model.get_articulation(name).articulation_type == ArticulationType.PRISMATIC for name in slide_names),
    )

    for r in range(3):
        for c in range(4):
            drawer = object_model.get_part(f"drawer_{r}_{c}")
            slide = object_model.get_articulation(f"drawer_slide_{r}_{c}")
            rail_name = f"guide_{r}_{c}_0"

            ctx.expect_gap(
                drawer,
                desk,
                axis="z",
                positive_elem="tray_bottom",
                negative_elem=rail_name,
                max_gap=0.001,
                max_penetration=0.0,
                name=f"drawer {r},{c} rests on its guide rail",
            )
            ctx.expect_overlap(
                drawer,
                desk,
                axes="xy",
                elem_a="tray_bottom",
                elem_b=rail_name,
                min_overlap=0.010,
                name=f"drawer {r},{c} is seated over its guide rail",
            )

            rest_pos = ctx.part_world_position(drawer)
            with ctx.pose({slide: 0.220}):
                ctx.expect_overlap(
                    drawer,
                    desk,
                    axes="y",
                    elem_a="tray_bottom",
                    elem_b=rail_name,
                    min_overlap=0.180,
                    name=f"drawer {r},{c} remains engaged when extended",
                )
                extended_pos = ctx.part_world_position(drawer)
            ctx.check(
                f"drawer {r},{c} slides toward the reader",
                rest_pos is not None and extended_pos is not None and extended_pos[1] < rest_pos[1] - 0.20,
                details=f"rest={rest_pos}, extended={extended_pos}",
            )

    return ctx.report()


object_model = build_object_model()
