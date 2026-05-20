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
    model = ArticulatedObject(name="fork_entry_dock_ramp")

    steel = model.material("painted_dark_steel", rgba=(0.18, 0.20, 0.21, 1.0))
    tread = model.material("worn_tread_steel", rgba=(0.36, 0.38, 0.38, 1.0))
    yellow = model.material("safety_yellow", rgba=(1.0, 0.72, 0.05, 1.0))
    black = model.material("black_rubber_caps", rgba=(0.02, 0.02, 0.018, 1.0))

    deck = model.part("deck")

    # Realistic pallet/dock ramp proportions: broad enough for a pallet truck,
    # long enough to make the slope moderate, and visibly made from steel plate.
    deck_len = 2.40
    deck_width = 1.35
    deck_thickness = 0.080
    slope = math.radians(12.0)
    lower_mid_z = 0.18

    cos_s = math.cos(slope)
    sin_s = math.sin(slope)

    def deck_xyz(u: float, v: float, n: float) -> tuple[float, float, float]:
        """World point from deck-local length/across/normal coordinates."""
        return (u * cos_s - n * sin_s, v, lower_mid_z + u * sin_s + n * cos_s)

    def deck_origin(u: float, v: float, n: float) -> Origin:
        return Origin(xyz=deck_xyz(u, v, n), rpy=(0.0, -slope, 0.0))

    deck.visual(
        Box((deck_len, deck_width, deck_thickness)),
        origin=deck_origin(deck_len / 2.0, 0.0, 0.0),
        material=steel,
        name="deck_panel",
    )

    # Raised anti-slip ribs welded to the inclined deck.
    for i, u in enumerate((0.34, 0.62, 0.90, 1.18, 1.46, 1.74, 2.02)):
        deck.visual(
            Box((0.038, deck_width - 0.16, 0.014)),
            origin=deck_origin(u, 0.0, deck_thickness / 2.0 + 0.007),
            material=tread,
            name=f"tread_rib_{i}",
        )

    # Low side curb strips give the flat panel believable stiffened long edges.
    for side, label in ((1.0, "edge_0"), (-1.0, "edge_1")):
        deck.visual(
            Box((deck_len, 0.055, 0.090)),
            origin=deck_origin(
                deck_len / 2.0,
                side * (deck_width / 2.0 + 0.020),
                deck_thickness / 2.0 + 0.045,
            ),
            material=steel,
            name=f"{label}_curb",
        )

    # Two horizontal rectangular fork-pocket tubes under the lower half of the
    # ramp.  They are open-ended sleeves, built from four plates each, so their
    # front openings read as real pallet-truck entry pockets rather than solid
    # blocks.
    tube_len = 0.92
    tube_start_x = 0.08
    tube_center_x = tube_start_x + tube_len / 2.0
    tube_width = 0.205
    tube_height = 0.115
    wall = 0.018
    tube_bottom_z = 0.030
    tube_top_z = tube_bottom_z + tube_height
    tube_center_z = tube_bottom_z + tube_height / 2.0
    pocket_y = 0.285

    def underside_z_at_world_x(x: float) -> float:
        u = x / cos_s
        return lower_mid_z + u * sin_s - (deck_thickness / 2.0) * cos_s

    for idx, yc in enumerate((pocket_y, -pocket_y)):
        deck.visual(
            Box((tube_len, tube_width, wall)),
            origin=Origin(
                xyz=(tube_center_x, yc, tube_bottom_z + wall / 2.0),
            ),
            material=steel,
            name=f"fork_pocket_{idx}_bottom",
        )
        deck.visual(
            Box((tube_len, tube_width, wall)),
            origin=Origin(
                xyz=(tube_center_x, yc, tube_top_z - wall / 2.0),
            ),
            material=steel,
            name=f"fork_pocket_{idx}_top",
        )
        for side, wall_label in ((1.0, "outer"), (-1.0, "inner")):
            deck.visual(
                Box((tube_len, wall, tube_height)),
                origin=Origin(
                    xyz=(
                        tube_center_x,
                        yc + side * (tube_width / 2.0 - wall / 2.0),
                        tube_center_z,
                    ),
                ),
                material=steel,
                name=f"fork_pocket_{idx}_{wall_label}_wall",
            )
        for j, x in enumerate((0.28, 0.78)):
            deck_under = underside_z_at_world_x(x)
            hanger_height = max(deck_under - tube_top_z + 0.018, 0.040)
            deck.visual(
                Box((0.060, tube_width + 0.050, hanger_height)),
                origin=Origin(
                    xyz=(x, yc, tube_top_z - 0.004 + hanger_height / 2.0),
                ),
                material=steel,
                name=f"fork_pocket_{idx}_hanger_{j}",
            )

    # Fixed rail hinge brackets at both long edges.  They stop below the moving
    # hinge barrels, leaving the revolute joint free while visually supporting it.
    hinge_n = deck_thickness / 2.0 + 0.125
    hinge_y_offset = deck_width / 2.0 + 0.070
    bracket_n_center = deck_thickness / 2.0 + 0.040
    for idx, side in enumerate((1.0, -1.0)):
        for j, u in enumerate((0.24, 1.20, 2.16)):
            deck.visual(
                Box((0.095, 0.070, 0.080)),
                origin=deck_origin(u, side * hinge_y_offset, bracket_n_center),
                material=yellow,
                name=f"rail_bracket_{idx}_{j}",
            )
        deck.visual(
            Box((deck_len - 0.22, 0.030, 0.035)),
            origin=deck_origin(deck_len / 2.0, side * hinge_y_offset, hinge_n - 0.0470),
            material=yellow,
            name=f"hinge_mount_bar_{idx}",
        )

    def add_guardrail(name: str, side: float) -> object:
        rail = model.part(name)
        rail_len = deck_len - 0.30
        rail_x = deck_len / 2.0
        height = 0.92
        y = side * 0.018
        tube = 0.045

        rail.visual(
            Cylinder(radius=0.026, length=rail_len),
            origin=Origin(xyz=(rail_x, y, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=yellow,
            name="hinge_barrel",
        )
        rail.visual(
            Box((rail_len, 0.020, 0.010)),
            origin=Origin(xyz=(rail_x, y, -0.0245)),
            material=yellow,
            name="hinge_saddle",
        )
        for x, label in ((0.22, "post_0"), (deck_len / 2.0, "post_1"), (2.18, "post_2")):
            rail.visual(
                Box((tube, tube, height)),
                origin=Origin(xyz=(x, y, height / 2.0)),
                material=yellow,
                name=label,
            )
        rail.visual(
            Box((rail_len, tube, tube)),
            origin=Origin(xyz=(rail_x, y, height)),
            material=yellow,
            name="top_rail",
        )
        rail.visual(
            Box((rail_len, tube * 0.85, tube * 0.85)),
            origin=Origin(xyz=(rail_x, y, height * 0.53)),
            material=yellow,
            name="mid_rail",
        )
        # A diagonal brace makes each folding side frame read as a welded guard
        # rail rather than a plain rectangular outline.
        x0, z0 = 0.30, 0.08
        x1, z1 = 2.10, height * 0.86
        dx, dz = x1 - x0, z1 - z0
        diag_len = math.hypot(dx, dz)
        rail.visual(
            Box((diag_len, tube * 0.65, tube * 0.65)),
            origin=Origin(
                xyz=((x0 + x1) / 2.0, y, (z0 + z1) / 2.0),
                rpy=(0.0, -math.atan2(dz, dx), 0.0),
            ),
            material=yellow,
            name="diagonal_brace",
        )
        for x, label in ((0.20, "end_cap_0"), (2.20, "end_cap_1")):
            rail.visual(
                Box((0.020, tube * 1.05, tube * 1.05)),
                origin=Origin(xyz=(x, y, height)),
                material=black,
                name=label,
            )
        return rail

    guardrail_0 = add_guardrail("guardrail_0", 1.0)
    guardrail_1 = add_guardrail("guardrail_1", -1.0)

    hinge_origin_0 = Origin(
        xyz=deck_xyz(0.0, hinge_y_offset, hinge_n),
        rpy=(0.0, -slope, 0.0),
    )
    hinge_origin_1 = Origin(
        xyz=deck_xyz(0.0, -hinge_y_offset, hinge_n),
        rpy=(0.0, -slope, 0.0),
    )

    model.articulation(
        "deck_to_guardrail_0",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=guardrail_0,
        origin=hinge_origin_0,
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.2, lower=0.0, upper=1.45),
    )
    model.articulation(
        "deck_to_guardrail_1",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=guardrail_1,
        origin=hinge_origin_1,
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.2, lower=0.0, upper=1.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    guardrail_0 = object_model.get_part("guardrail_0")
    guardrail_1 = object_model.get_part("guardrail_1")
    hinge_0 = object_model.get_articulation("deck_to_guardrail_0")
    hinge_1 = object_model.get_articulation("deck_to_guardrail_1")

    ctx.check(
        "two folding guardrail hinges",
        hinge_0.articulation_type == ArticulationType.REVOLUTE
        and hinge_1.articulation_type == ArticulationType.REVOLUTE
        and hinge_0.motion_limits is not None
        and hinge_1.motion_limits is not None
        and hinge_0.motion_limits.upper >= 1.3
        and hinge_1.motion_limits.upper >= 1.3,
        details="Each long-edge guardrail should fold on a revolute hinge through a large arc.",
    )

    deck_aabb = ctx.part_element_world_aabb(deck, elem="deck_panel")
    pocket_0 = ctx.part_element_world_aabb(deck, elem="fork_pocket_0_top")
    pocket_1 = ctx.part_element_world_aabb(deck, elem="fork_pocket_1_top")
    if deck_aabb and pocket_0 and pocket_1:
        ctx.check(
            "inclined deck has real ramp rise",
            deck_aabb[1][2] - deck_aabb[0][2] > 0.45
            and deck_aabb[1][0] - deck_aabb[0][0] > 2.25,
            details=f"deck_aabb={deck_aabb}",
        )
        ctx.check(
            "fork pockets sit below deck underside",
            pocket_0[1][2] < deck_aabb[0][2] + 0.06 and pocket_1[1][2] < deck_aabb[0][2] + 0.06,
            details=f"deck_aabb={deck_aabb}, pocket_0={pocket_0}, pocket_1={pocket_1}",
        )
        center_y_0 = (pocket_0[0][1] + pocket_0[1][1]) / 2.0
        center_y_1 = (pocket_1[0][1] + pocket_1[1][1]) / 2.0
        ctx.check(
            "two separated fork entry pockets",
            abs(center_y_0 - center_y_1) > 0.45,
            details=f"pocket centers y={center_y_0}, {center_y_1}",
        )
    else:
        ctx.fail("fork pocket geometry measurable", "Expected deck and fork pocket element AABBs.")

    def aabb_center(aabb, axis_index: int) -> float:
        return (aabb[0][axis_index] + aabb[1][axis_index]) / 2.0

    top_0_rest = ctx.part_element_world_aabb(guardrail_0, elem="top_rail")
    top_1_rest = ctx.part_element_world_aabb(guardrail_1, elem="top_rail")
    with ctx.pose({hinge_0: 1.35, hinge_1: 1.35}):
        top_0_folded = ctx.part_element_world_aabb(guardrail_0, elem="top_rail")
        top_1_folded = ctx.part_element_world_aabb(guardrail_1, elem="top_rail")
    if top_0_rest and top_1_rest and top_0_folded and top_1_folded:
        ctx.check(
            "guardrails fold inward and lower",
            aabb_center(top_0_folded, 1) < aabb_center(top_0_rest, 1) - 0.45
            and aabb_center(top_1_folded, 1) > aabb_center(top_1_rest, 1) + 0.45
            and aabb_center(top_0_folded, 2) < aabb_center(top_0_rest, 2) - 0.45
            and aabb_center(top_1_folded, 2) < aabb_center(top_1_rest, 2) - 0.45,
            details=f"rest={top_0_rest}, {top_1_rest}; folded={top_0_folded}, {top_1_folded}",
        )
    else:
        ctx.fail("guardrail top rails measurable", "Expected top rail AABBs in rest and folded poses.")

    return ctx.report()


object_model = build_object_model()
