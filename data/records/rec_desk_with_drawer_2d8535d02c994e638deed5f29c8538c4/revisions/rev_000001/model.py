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


PEDESTAL_X = (-0.55, 0.55)
DRAWER_Z = (0.165, 0.385, 0.605)
DRAWER_TRAVEL = 0.36
DRAWER_FRONT_Y = -0.355


def _add_box(part, name: str, size, xyz, material) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _add_pedestal_frame(frame, center_x: float, tag: int, materials: dict[str, Material]) -> None:
    wood = materials["wood"]
    edge = materials["edge"]
    shadow = materials["shadow"]
    metal = materials["metal"]

    width = 0.46
    depth = 0.66
    half_w = width / 2.0
    half_d = depth / 2.0
    side_t = 0.025
    panel_h = 0.68
    panel_z = 0.39

    # Structural carcass: side, back, bottom, and top panels form an open-front pedestal.
    _add_box(frame, f"pedestal_{tag}_outer_side", (side_t, depth, panel_h), (center_x - half_w + side_t / 2.0, 0.0, panel_z), wood)
    _add_box(frame, f"pedestal_{tag}_inner_side", (side_t, depth, panel_h), (center_x + half_w - side_t / 2.0, 0.0, panel_z), wood)
    _add_box(frame, f"pedestal_{tag}_back", (width, side_t, panel_h), (center_x, half_d - side_t / 2.0, panel_z), wood)
    _add_box(frame, f"pedestal_{tag}_bottom", (width, depth, 0.035), (center_x, 0.0, 0.0675), wood)
    _add_box(frame, f"pedestal_{tag}_top", (width, depth, 0.030), (center_x, 0.0, 0.715), wood)

    # Face-frame rails visible between the three drawer fronts.
    for idx, zc in enumerate((0.275, 0.495)):
        _add_box(frame, f"pedestal_{tag}_divider_{idx}", (0.435, 0.026, 0.025), (center_x, -0.322, zc), edge)
    _add_box(frame, f"pedestal_{tag}_top_rail", (0.435, 0.026, 0.026), (center_x, -0.322, 0.704), edge)
    _add_box(frame, f"pedestal_{tag}_base_rail", (0.435, 0.026, 0.026), (center_x, -0.322, 0.096), edge)

    # Metal guide rails, one pair per drawer, attached to the inside faces.
    inner_left_x = center_x - half_w + side_t
    inner_right_x = center_x + half_w - side_t
    rail_x_offset = 0.013
    for row, zc in enumerate(DRAWER_Z):
        rail_z = zc - 0.087
        _add_box(
            frame,
            f"rail_{tag}_{row}_0",
            (0.028, 0.520, 0.018),
            (inner_left_x + rail_x_offset, -0.025, rail_z),
            metal,
        )
        _add_box(
            frame,
            f"rail_{tag}_{row}_1",
            (0.028, 0.520, 0.018),
            (inner_right_x - rail_x_offset, -0.025, rail_z),
            metal,
        )

    # Small adjustable feet under each pedestal.
    for ix, x in enumerate((center_x - 0.17, center_x + 0.17)):
        for iy, y in enumerate((-0.25, 0.25)):
            _add_box(frame, f"foot_{tag}_{ix}_{iy}", (0.070, 0.070, 0.050), (x, y, 0.025), edge)


def _add_drawer(model: ArticulatedObject, frame, center_x: float, pedestal: int, row: int, zc: float, materials: dict[str, Material]):
    wood = materials["wood"]
    edge = materials["edge"]
    metal = materials["metal"]

    drawer = model.part(f"drawer_{pedestal}_{row}")

    # The drawer part frame is at the visible front center when closed.
    _add_box(drawer, "front", (0.370, 0.035, 0.190), (0.0, 0.0, 0.0), wood)
    _add_box(drawer, "front_inset", (0.315, 0.010, 0.135), (0.0, -0.0215, 0.0), edge)

    # A shallow open-topped drawer box extends into the pedestal.
    _add_box(drawer, "bottom", (0.320, 0.520, 0.018), (0.0, 0.310, -0.060), wood)
    _add_box(drawer, "side_0", (0.018, 0.520, 0.130), (-0.169, 0.310, 0.005), wood)
    _add_box(drawer, "side_1", (0.018, 0.520, 0.130), (0.169, 0.310, 0.005), wood)
    _add_box(drawer, "back", (0.320, 0.018, 0.130), (0.0, 0.565, 0.005), wood)
    _add_box(drawer, "front_cleat_0", (0.018, 0.075, 0.060), (-0.169, 0.0375, 0.040), wood)
    _add_box(drawer, "front_cleat_1", (0.018, 0.075, 0.060), (0.169, 0.0375, 0.040), wood)

    # Sliding members riding above the fixed metal guide rails.
    _add_box(drawer, "runner_0", (0.028, 0.480, 0.018), (-0.192, 0.305, -0.069), metal)
    _add_box(drawer, "runner_1", (0.028, 0.480, 0.018), (0.192, 0.305, -0.069), metal)

    # A fixed pull handle made of connected metal blocks.
    _add_box(drawer, "handle_bar", (0.160, 0.018, 0.018), (0.0, -0.043, 0.006), metal)
    _add_box(drawer, "handle_post_0", (0.018, 0.050, 0.018), (-0.070, -0.021, 0.006), metal)
    _add_box(drawer, "handle_post_1", (0.018, 0.050, 0.018), (0.070, -0.021, 0.006), metal)

    model.articulation(
        f"frame_to_drawer_{pedestal}_{row}",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=drawer,
        origin=Origin(xyz=(center_x, DRAWER_FRONT_Y, zc)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=DRAWER_TRAVEL),
    )

    return drawer


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_desk")

    materials = {
        "wood": model.material("warm_oak", color=(0.62, 0.38, 0.18, 1.0)),
        "edge": model.material("dark_edge_banding", color=(0.24, 0.14, 0.07, 1.0)),
        "shadow": model.material("recess_shadow", color=(0.055, 0.045, 0.035, 1.0)),
        "metal": model.material("blackened_metal", color=(0.03, 0.03, 0.032, 1.0)),
    }

    frame = model.part("desk_frame")

    # Wide work surface with dark edge banding and a modest rear privacy panel.
    _add_box(frame, "work_surface", (1.70, 0.78, 0.055), (0.0, 0.0, 0.7575), materials["wood"])
    _add_box(frame, "front_edge", (1.72, 0.035, 0.070), (0.0, -0.395, 0.747), materials["edge"])
    _add_box(frame, "rear_edge", (1.72, 0.030, 0.055), (0.0, 0.392, 0.748), materials["edge"])
    _add_box(frame, "side_edge_0", (0.035, 0.78, 0.055), (-0.865, 0.0, 0.748), materials["edge"])
    _add_box(frame, "side_edge_1", (0.035, 0.78, 0.055), (0.865, 0.0, 0.748), materials["edge"])
    _add_box(frame, "modesty_panel", (0.72, 0.030, 0.440), (0.0, 0.318, 0.465), materials["wood"])

    for tag, center_x in enumerate(PEDESTAL_X):
        _add_pedestal_frame(frame, center_x, tag, materials)

    for pedestal, center_x in enumerate(PEDESTAL_X):
        for row, zc in enumerate(DRAWER_Z):
            _add_drawer(model, frame, center_x, pedestal, row, zc, materials)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("desk_frame")
    drawer_joints = [object_model.get_articulation(f"frame_to_drawer_{p}_{r}") for p in range(2) for r in range(3)]
    ctx.check("six sliding drawers", len(drawer_joints) == 6, details=f"count={len(drawer_joints)}")
    for joint in drawer_joints:
        ctx.check(
            f"{joint.name} is prismatic",
            joint.articulation_type == ArticulationType.PRISMATIC,
            details=f"type={joint.articulation_type}",
        )

    for pedestal in range(2):
        for row in range(3):
            drawer = object_model.get_part(f"drawer_{pedestal}_{row}")
            joint = object_model.get_articulation(f"frame_to_drawer_{pedestal}_{row}")
            ctx.expect_overlap(
                drawer,
                frame,
                axes="y",
                elem_a="runner_0",
                elem_b=f"rail_{pedestal}_{row}_0",
                min_overlap=0.40,
                name=f"drawer_{pedestal}_{row} closed rail engagement",
            )
            rest_pos = ctx.part_world_position(drawer)
            with ctx.pose({joint: DRAWER_TRAVEL}):
                ctx.expect_overlap(
                    drawer,
                    frame,
                    axes="y",
                    elem_a="runner_0",
                    elem_b=f"rail_{pedestal}_{row}_0",
                    min_overlap=0.08,
                    name=f"drawer_{pedestal}_{row} retained rail engagement",
                )
                open_pos = ctx.part_world_position(drawer)
            ctx.check(
                f"drawer_{pedestal}_{row} slides outward",
                rest_pos is not None and open_pos is not None and open_pos[1] < rest_pos[1] - 0.30,
                details=f"rest={rest_pos}, open={open_pos}",
            )

    return ctx.report()


object_model = build_object_model()
