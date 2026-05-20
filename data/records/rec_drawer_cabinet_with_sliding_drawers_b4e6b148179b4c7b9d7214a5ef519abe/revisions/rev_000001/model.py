from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tall_steel_filing_cabinet")

    painted_steel = model.material("painted_steel", rgba=(0.55, 0.60, 0.63, 1.0))
    edge_shadow = model.material("dark_recess", rgba=(0.055, 0.060, 0.065, 1.0))
    rail_steel = model.material("galvanized_rail", rgba=(0.72, 0.74, 0.72, 1.0))
    label_card = model.material("paper_label", rgba=(0.82, 0.84, 0.78, 1.0))

    cabinet = model.part("cabinet")

    # Realistic four-drawer vertical file proportions in meters.
    depth = 0.62
    width = 0.48
    height = 1.34
    wall = 0.025
    front_x = -depth / 2.0
    back_x = depth / 2.0
    outer_y = width / 2.0

    def add_box(part, size, xyz, name, material: Material = painted_steel) -> None:
        part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)

    # Steel cabinet shell: side sheets, top/bottom, back, and proud front frame.
    add_box(cabinet, (depth, wall, height), (0.0, outer_y - wall / 2.0, height / 2.0), "side_panel_0")
    add_box(cabinet, (depth, wall, height), (0.0, -outer_y + wall / 2.0, height / 2.0), "side_panel_1")
    add_box(cabinet, (depth, width, wall), (0.0, 0.0, height - wall / 2.0), "top_panel")
    add_box(cabinet, (depth, width, wall), (0.0, 0.0, wall / 2.0), "bottom_panel")
    add_box(cabinet, (wall, width, height), (back_x - wall / 2.0, 0.0, height / 2.0), "back_panel")

    frame_depth = 0.016
    stile_width = 0.028
    frame_x = front_x - frame_depth / 2.0
    add_box(cabinet, (frame_depth, stile_width, height), (frame_x, outer_y - stile_width / 2.0, height / 2.0), "front_stile_0")
    add_box(cabinet, (frame_depth, stile_width, height), (frame_x, -outer_y + stile_width / 2.0, height / 2.0), "front_stile_1")

    drawer_front_h = 0.285
    drawer_gap = 0.018
    lower_margin = 0.075
    drawer_pitch = drawer_front_h + drawer_gap
    drawer_centers_z = [lower_margin + drawer_front_h / 2.0 + i * drawer_pitch for i in range(4)]

    # Narrow horizontal front rails separating the four full-width drawers.
    add_box(cabinet, (frame_depth, width, 0.020), (frame_x, 0.0, lower_margin - 0.017), "front_rail_bottom")
    for i in range(3):
        rail_z = drawer_centers_z[i] + drawer_front_h / 2.0 + drawer_gap / 2.0
        add_box(cabinet, (frame_depth, width, 0.010), (frame_x, 0.0, rail_z), f"front_rail_{i}")
    add_box(cabinet, (frame_depth, width, 0.020), (frame_x, 0.0, drawer_centers_z[-1] + drawer_front_h / 2.0 + 0.018), "front_rail_top")

    # Fixed guide rails bolted to the side sheets in each drawer bay.
    fixed_rail_len = 0.54
    fixed_rail_x = front_x + 0.045 + fixed_rail_len / 2.0
    for i, zc in enumerate(drawer_centers_z):
        rail_z = zc - 0.130
        add_box(cabinet, (fixed_rail_len, 0.018, 0.018), (fixed_rail_x, outer_y - wall - 0.009, rail_z), f"guide_rail_{i}_0", rail_steel)
        add_box(cabinet, (fixed_rail_len, 0.018, 0.018), (fixed_rail_x, -outer_y + wall + 0.009, rail_z), f"guide_rail_{i}_1", rail_steel)

    # Drawers slide out from the front; their local frame is on the closed front plane.
    drawer_front_w = 0.418
    front_thick = 0.024
    drawer_body_len = 0.58
    drawer_body_w = 0.370
    drawer_body_h = 0.205
    handle_w = 0.300
    handle_h = 0.052
    handle_z = 0.055
    handle_wall = 0.012
    handle_back_x = -0.006

    for i, zc in enumerate(drawer_centers_z):
        drawer = model.part(f"drawer_{i}")

        # Front sheet with an actual rectangular handle opening assembled from
        # connected steel regions around the cutout.
        slot_bottom = handle_z - handle_h / 2.0
        slot_top = handle_z + handle_h / 2.0
        add_box(
            drawer,
            (front_thick, drawer_front_w, drawer_front_h / 2.0 + slot_bottom),
            (-front_thick / 2.0, 0.0, (-drawer_front_h / 2.0 + slot_bottom) / 2.0),
            "front_lower",
        )
        add_box(
            drawer,
            (front_thick, drawer_front_w, drawer_front_h / 2.0 - slot_top),
            (-front_thick / 2.0, 0.0, (drawer_front_h / 2.0 + slot_top) / 2.0),
            "front_upper",
        )
        side_web_w = (drawer_front_w - handle_w) / 2.0
        add_box(
            drawer,
            (front_thick, side_web_w, handle_h),
            (-front_thick / 2.0, -(handle_w / 2.0 + side_web_w / 2.0), handle_z),
            "front_web_0",
        )
        add_box(
            drawer,
            (front_thick, side_web_w, handle_h),
            (-front_thick / 2.0, handle_w / 2.0 + side_web_w / 2.0, handle_z),
            "front_web_1",
        )

        # Recessed pull cup: dark back wall and return lips set behind the face.
        add_box(drawer, (0.004, handle_w, handle_h), (handle_back_x, 0.0, handle_z), "handle_back", edge_shadow)
        add_box(drawer, (0.018, handle_w, handle_wall), (-0.012, 0.0, slot_top + handle_wall / 2.0), "handle_lip_top", edge_shadow)
        add_box(drawer, (0.018, handle_w, handle_wall), (-0.012, 0.0, slot_bottom - handle_wall / 2.0), "handle_lip_bottom", edge_shadow)

        # Small label card typical of filing drawers.
        add_box(drawer, (0.003, 0.120, 0.030), (-front_thick - 0.0015, 0.0, -0.060), "label_card", label_card)

        # Hidden metal drawer box and sliding members, connected to the front.
        add_box(drawer, (drawer_body_len, drawer_body_w, 0.012), (drawer_body_len / 2.0, 0.0, -0.112), "drawer_floor")
        add_box(drawer, (drawer_body_len, 0.012, drawer_body_h), (drawer_body_len / 2.0, drawer_body_w / 2.0 - 0.006, -0.015), "drawer_side_0")
        add_box(drawer, (drawer_body_len, 0.012, drawer_body_h), (drawer_body_len / 2.0, -drawer_body_w / 2.0 + 0.006, -0.015), "drawer_side_1")
        add_box(drawer, (0.012, drawer_body_w, drawer_body_h), (drawer_body_len + 0.006, 0.0, -0.015), "drawer_back")
        add_box(drawer, (drawer_body_len, 0.010, 0.014), (drawer_body_len / 2.0, drawer_body_w / 2.0 + 0.007, -0.126), "runner_0", rail_steel)
        add_box(drawer, (drawer_body_len, 0.010, 0.014), (drawer_body_len / 2.0, -drawer_body_w / 2.0 - 0.007, -0.126), "runner_1", rail_steel)

        model.articulation(
            f"cabinet_to_drawer_{i}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=drawer,
            origin=Origin(xyz=(front_x, 0.0, zc)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.42),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")

    for i in range(4):
        drawer = object_model.get_part(f"drawer_{i}")
        joint = object_model.get_articulation(f"cabinet_to_drawer_{i}")

        ctx.expect_overlap(
            drawer,
            cabinet,
            axes="z",
            elem_a="runner_0",
            elem_b=f"guide_rail_{i}_0",
            min_overlap=0.010,
            name=f"drawer_{i} floor is vertically carried by its guide bay",
        )
        ctx.expect_within(
            drawer,
            cabinet,
            axes="y",
            inner_elem="drawer_floor",
            outer_elem="back_panel",
            margin=0.010,
            name=f"drawer_{i} stays centered between side sheets",
        )

        rest_pos = ctx.part_world_position(drawer)
        with ctx.pose({joint: 0.42}):
            ctx.expect_overlap(
                drawer,
                cabinet,
                axes="x",
                elem_a="runner_0",
                elem_b=f"guide_rail_{i}_0",
                min_overlap=0.055,
                name=f"drawer_{i} remains inserted on rails at full travel",
            )
            extended_pos = ctx.part_world_position(drawer)

        ctx.check(
            f"drawer_{i} slides outward from cabinet front",
            rest_pos is not None and extended_pos is not None and extended_pos[0] < rest_pos[0] - 0.35,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    return ctx.report()


object_model = build_object_model()
