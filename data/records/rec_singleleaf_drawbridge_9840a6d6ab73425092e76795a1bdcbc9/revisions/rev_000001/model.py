from __future__ import annotations

import math

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


PI = math.pi


def _mat(model: ArticulatedObject, name: str, rgba: tuple[float, float, float, float]) -> Material:
    return model.material(name, rgba=rgba)


def _box_beam(
    part,
    *,
    name: str,
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
    thickness: tuple[float, float],
    material,
    overrun: float = 0.04,
) -> None:
    """Rectangular beam whose local X axis runs between two points."""

    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz) + overrun
    cx = (p0[0] + p1[0]) * 0.5
    cy = (p0[1] + p1[1]) * 0.5
    cz = (p0[2] + p1[2]) * 0.5
    yaw = math.atan2(dy, dx)
    hyp = math.sqrt(dx * dx + dy * dy)
    pitch = -math.atan2(dz, hyp)
    part.visual(
        Box((length, thickness[0], thickness[1])),
        origin=Origin(xyz=(cx, cy, cz), rpy=(0.0, pitch, yaw)),
        material=material,
        name=name,
    )


def _y_cylinder(part, *, name: str, center: tuple[float, float, float], radius: float, length: float, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(-PI / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="engineered_bascule_bridge")

    concrete = _mat(model, "weathered_concrete", (0.55, 0.56, 0.54, 1.0))
    dark_concrete = _mat(model, "dark_counterweight_concrete", (0.38, 0.39, 0.38, 1.0))
    asphalt = _mat(model, "black_asphalt", (0.035, 0.038, 0.04, 1.0))
    steel = _mat(model, "painted_blue_steel", (0.06, 0.16, 0.25, 1.0))
    dark_steel = _mat(model, "dark_bearing_steel", (0.03, 0.035, 0.04, 1.0))
    water = _mat(model, "river_water", (0.04, 0.24, 0.36, 0.62))
    yellow = _mat(model, "road_yellow", (1.0, 0.78, 0.05, 1.0))
    white = _mat(model, "road_white", (0.92, 0.92, 0.86, 1.0))
    safety = _mat(model, "safety_red", (0.75, 0.08, 0.04, 1.0))

    support = model.part("support")

    # Civil base and river plane: all static supports tie down into this bed.
    support.visual(
        Box((32.0, 9.0, 0.24)),
        origin=Origin(xyz=(7.0, 0.0, -0.12)),
        material=concrete,
        name="foundation_slab",
    )
    support.visual(
        Box((24.0, 8.2, 0.035)),
        origin=Origin(xyz=(6.8, 0.0, 0.02)),
        material=water,
        name="river_surface",
    )

    # Approach embankments and decks leave an open counterweight pit at the hinge.
    support.visual(
        Box((5.3, 5.8, 2.72)),
        origin=Origin(xyz=(-5.85, 0.0, 1.36)),
        material=concrete,
        name="west_abutment",
    )
    support.visual(
        Box((8.0, 5.8, 2.72)),
        origin=Origin(xyz=(20.2, 0.0, 1.36)),
        material=concrete,
        name="east_abutment",
    )
    support.visual(
        Box((5.3, 4.8, 0.28)),
        origin=Origin(xyz=(-5.85, 0.0, 2.86)),
        material=concrete,
        name="west_approach_slab",
    )
    support.visual(
        Box((5.1, 4.1, 0.045)),
        origin=Origin(xyz=(-5.85, 0.0, 3.012)),
        material=asphalt,
        name="west_approach_road",
    )
    support.visual(
        Box((8.0, 4.8, 0.28)),
        origin=Origin(xyz=(20.2, 0.0, 2.86)),
        material=concrete,
        name="east_approach_slab",
    )
    support.visual(
        Box((7.8, 4.1, 0.045)),
        origin=Origin(xyz=(20.2, 0.0, 3.012)),
        material=asphalt,
        name="east_approach_road",
    )

    for x, length, prefix in [(-5.85, 5.0, "west"), (20.2, 7.6, "east")]:
        support.visual(
            Box((length, 0.055, 0.012)),
            origin=Origin(xyz=(x, -0.18, 3.039)),
            material=yellow,
            name=f"{prefix}_center_stripe",
        )
        for y in (-1.82, 1.82):
            support.visual(
                Box((length, 0.05, 0.012)),
                origin=Origin(xyz=(x, y, 3.040)),
                material=white,
                name=f"{prefix}_edge_stripe_{'north' if y > 0 else 'south'}",
            )

    # Hinge pier side walls, bearing pedestals, and a far rest pier.
    for y in (-3.05, 3.05):
        side = "north" if y > 0 else "south"
        support.visual(
            Box((3.2, 0.62, 2.75)),
            origin=Origin(xyz=(-0.7, y, 1.375)),
            material=concrete,
            name=f"{side}_pit_wall",
        )
        support.visual(
            Box((1.25, 1.25, 1.00)),
            origin=Origin(xyz=(0.0, y, 2.03)),
            material=concrete,
            name=f"{side}_bearing_pedestal",
        )
        # Paired cheek plates form a visible yoke around the trunnion without
        # intersecting the moving axle.
        sign = 1.0 if y > 0 else -1.0
        for offset, cue in [(2.78, "inner"), (3.18, "outer")]:
            support.visual(
                Box((0.82, 0.18, 1.12)),
                origin=Origin(xyz=(0.0, sign * offset, 3.0)),
                material=dark_steel,
                name=f"{side}_{cue}_cheek",
            )
        support.visual(
            Box((0.95, 0.76, 0.18)),
            origin=Origin(xyz=(0.0, y, 3.58)),
            material=dark_steel,
            name=f"{side}_bearing_cap",
        )

    support.visual(
        Box((1.2, 5.5, 2.55)),
        origin=Origin(xyz=(15.85, 0.0, 1.275)),
        material=concrete,
        name="rest_pier",
    )
    support.visual(
        Box((1.85, 5.4, 0.18)),
        origin=Origin(xyz=(15.85, 0.0, 2.74)),
        material=dark_steel,
        name="rest_cap",
    )

    # Machinery portal and bracing gives the hinge end engineered scale.
    for y in (-3.25, 3.25):
        side = "north" if y > 0 else "south"
        support.visual(
            Box((0.32, 0.32, 4.25)),
            origin=Origin(xyz=(-1.75, y, 4.65)),
            material=steel,
            name=f"{side}_tower_column",
        )
        support.visual(
            Box((1.6, 0.95, 0.95)),
            origin=Origin(xyz=(-2.05, y, 3.03)),
            material=concrete,
            name=f"{side}_machinery_house",
        )
        _box_beam(
            support,
            name=f"{side}_tower_backstay",
            p0=(-1.75, y, 6.75),
            p1=(-3.35, y, 2.82),
            thickness=(0.18, 0.18),
            material=steel,
            overrun=0.08,
        )
        _box_beam(
            support,
            name=f"{side}_tower_frontstay",
            p0=(-1.75, y, 6.75),
            p1=(0.15, y, 2.85),
            thickness=(0.16, 0.16),
            material=steel,
            overrun=0.08,
        )
        _y_cylinder(
            support,
            name=f"{side}_drive_wheel",
            center=(-0.52, y, 3.05),
            radius=0.52,
            length=0.12,
            material=dark_steel,
        )

    support.visual(
        Box((0.34, 6.9, 0.34)),
        origin=Origin(xyz=(-1.75, 0.0, 6.72)),
        material=steel,
        name="tower_crosshead",
    )
    support.visual(
        Box((0.18, 6.6, 0.18)),
        origin=Origin(xyz=(-1.75, 0.0, 5.25)),
        material=steel,
        name="service_walkway",
    )

    # Fixed approach guard rails.
    for x, length, prefix in [(-5.85, 5.2, "west"), (20.2, 7.8, "east")]:
        for y in (-2.35, 2.35):
            support.visual(
                Box((length, 0.12, 0.16)),
                origin=Origin(xyz=(x, y, 3.07)),
                material=steel,
                name=f"{prefix}_curb_{'north' if y > 0 else 'south'}",
            )
            support.visual(
                Box((length, 0.08, 0.08)),
                origin=Origin(xyz=(x, y, 3.67)),
                material=steel,
                name=f"{prefix}_rail_{'north' if y > 0 else 'south'}",
            )
            for i, px in enumerate([x - length * 0.42, x - length * 0.18, x + length * 0.06, x + length * 0.30]):
                support.visual(
                    Box((0.08, 0.08, 0.66)),
                    origin=Origin(xyz=(px, y, 3.37)),
                    material=steel,
                    name=f"{prefix}_post_{'north' if y > 0 else 'south'}_{i}",
                )

    leaf = model.part("leaf")

    # Movable deck, authored in a hinge-line frame.  The span extends along +X;
    # the counterweight extends along -X and below the pivot.
    leaf.visual(
        Box((15.35, 4.65, 0.34)),
        origin=Origin(xyz=(7.78, 0.0, 0.0)),
        material=concrete,
        name="deck_slab",
    )
    leaf.visual(
        Box((15.05, 4.05, 0.05)),
        origin=Origin(xyz=(7.84, 0.0, 0.190)),
        material=asphalt,
        name="road_surface",
    )
    for y in (-2.32, 2.32):
        side = "north" if y > 0 else "south"
        leaf.visual(
            Box((15.25, 0.24, 0.20)),
            origin=Origin(xyz=(7.78, y, 0.31)),
            material=steel,
            name=f"{side}_curb",
        )
        leaf.visual(
            Box((15.0, 0.18, 0.20)),
            origin=Origin(xyz=(7.8, y, 0.58)),
            material=steel,
            name=f"{side}_lower_chord",
        )
        leaf.visual(
            Box((14.75, 0.16, 0.18)),
            origin=Origin(xyz=(7.9, y, 1.82)),
            material=steel,
            name=f"{side}_upper_chord",
        )
        leaf.visual(
            Box((14.9, 0.08, 0.08)),
            origin=Origin(xyz=(7.85, y, 1.34)),
            material=steel,
            name=f"{side}_handrail",
        )
        posts_x = [0.75, 2.65, 4.55, 6.45, 8.35, 10.25, 12.15, 14.05, 15.1]
        for i, px in enumerate(posts_x):
            leaf.visual(
                Box((0.16, 0.16, 1.72)),
                origin=Origin(xyz=(px, y, 1.02)),
                material=steel,
                name=f"{side}_truss_post_{i}",
            )
        for i in range(len(posts_x) - 1):
            p0 = (posts_x[i], y, 0.58)
            p1 = (posts_x[i + 1], y, 1.82)
            _box_beam(
                leaf,
                name=f"{side}_diagonal_{i}",
                p0=p0,
                p1=p1,
                thickness=(0.12, 0.12),
                material=steel,
                overrun=0.08,
            )

    # Cross beams and longitudinal stringers under the deck are visible when open.
    for i, x in enumerate([0.95, 2.4, 4.15, 5.9, 7.65, 9.4, 11.15, 12.9, 14.65]):
        leaf.visual(
            Box((0.18, 5.0, 0.22)),
            origin=Origin(xyz=(x, 0.0, -0.28)),
            material=steel,
            name=f"floorbeam_{i}",
        )
    for y in (-1.2, 0.0, 1.2):
        leaf.visual(
            Box((13.8, 0.13, 0.16)),
            origin=Origin(xyz=(7.2, y, -0.42)),
            material=steel,
            name=f"longitudinal_stringer_{int((y + 1.2) * 10)}",
        )

    # Road markings on the movable leaf.
    leaf.visual(
        Box((14.75, 0.055, 0.014)),
        origin=Origin(xyz=(7.92, -0.16, 0.221)),
        material=yellow,
        name="center_stripe_a",
    )
    leaf.visual(
        Box((14.75, 0.055, 0.014)),
        origin=Origin(xyz=(7.92, 0.16, 0.221)),
        material=yellow,
        name="center_stripe_b",
    )
    for y in (-1.82, 1.82):
        leaf.visual(
            Box((14.75, 0.045, 0.014)),
            origin=Origin(xyz=(7.92, y, 0.222)),
            material=white,
            name=f"edge_stripe_{'north' if y > 0 else 'south'}",
        )

    # Hinge trunnions and local hub plates ride in the fixed cheek-yokes.
    for y in (-2.50, 2.50):
        side = "north" if y > 0 else "south"
        _y_cylinder(
            leaf,
            name=f"{side}_trunnion",
            center=(0.0, y, 0.0),
            radius=0.22,
            length=0.35,
            material=dark_steel,
        )
        leaf.visual(
            Box((0.55, 0.30, 0.68)),
            origin=Origin(xyz=(0.16, 2.35 if y > 0 else -2.35, 0.02)),
            material=steel,
            name=f"{side}_hub_plate",
        )

    # Counterweight block and the steel bascule arms that tie it to the leaf.
    leaf.visual(
        Box((2.00, 4.05, 1.00)),
        origin=Origin(xyz=(-1.15, 0.0, -1.05)),
        material=dark_concrete,
        name="counterweight_block",
    )
    for y in (-1.55, 1.55):
        side = "north" if y > 0 else "south"
        leaf.visual(
            Box((2.95, 0.28, 0.28)),
            origin=Origin(xyz=(-1.22, y, -0.58)),
            material=steel,
            name=f"{side}_counterweight_arm",
        )
        leaf.visual(
            Box((0.30, 0.30, 0.88)),
            origin=Origin(xyz=(0.10, y, -0.24)),
            material=steel,
            name=f"{side}_arm_web",
        )
        _box_beam(
            leaf,
            name=f"{side}_arm_diagonal",
            p0=(-2.45, y, -0.60),
            p1=(0.20, y, 0.28),
            thickness=(0.16, 0.16),
            material=steel,
            overrun=0.08,
        )
    leaf.visual(
        Box((0.22, 4.35, 0.24)),
        origin=Origin(xyz=(-0.30, 0.0, -0.58)),
        material=steel,
        name="counterweight_cross_tie",
    )
    leaf.visual(
        Box((2.10, 4.15, 0.08)),
        origin=Origin(xyz=(-1.15, 0.0, -0.53)),
        material=safety,
        name="counterweight_warning_band",
    )

    model.articulation(
        "support_to_leaf",
        ArticulationType.REVOLUTE,
        parent=support,
        child=leaf,
        origin=Origin(xyz=(0.0, 0.0, 3.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=600000.0, velocity=0.18, lower=0.0, upper=1.1),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    leaf = object_model.get_part("leaf")
    hinge = object_model.get_articulation("support_to_leaf")

    limits = hinge.motion_limits
    ctx.check(
        "bascule hinge has realistic lift range",
        limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and 1.0 <= limits.upper <= 1.25,
        details=f"limits={limits}",
    )

    ctx.expect_gap(
        leaf,
        support,
        axis="z",
        positive_elem="deck_slab",
        negative_elem="rest_cap",
        min_gap=0.0,
        max_gap=0.10,
        name="closed leaf nearly bears on far rest cap",
    )
    ctx.expect_overlap(
        leaf,
        support,
        axes="xy",
        elem_a="deck_slab",
        elem_b="rest_cap",
        min_overlap=0.45,
        name="closed leaf aligns over rest pier",
    )

    closed_deck = ctx.part_element_world_aabb(leaf, elem="deck_slab")
    closed_counterweight = ctx.part_element_world_aabb(leaf, elem="counterweight_block")
    with ctx.pose({hinge: limits.upper if limits and limits.upper is not None else 1.1}):
        ctx.expect_gap(
            leaf,
            support,
            axis="z",
            positive_elem="counterweight_block",
            negative_elem="river_surface",
            min_gap=0.20,
            name="open counterweight remains clear of river pit",
        )
        ctx.expect_gap(
            leaf,
            support,
            axis="z",
            positive_elem="deck_slab",
            negative_elem="rest_cap",
            min_gap=0.12,
            name="open leaf clears far rest cap",
        )
        open_deck = ctx.part_element_world_aabb(leaf, elem="deck_slab")
        open_counterweight = ctx.part_element_world_aabb(leaf, elem="counterweight_block")

    ctx.check(
        "leaf rises like a bascule span",
        closed_deck is not None
        and open_deck is not None
        and open_deck[1][2] > closed_deck[1][2] + 7.0,
        details=f"closed_deck={closed_deck}, open_deck={open_deck}",
    )
    ctx.check(
        "counterweight swings downward as leaf opens",
        closed_counterweight is not None
        and open_counterweight is not None
        and open_counterweight[0][2] < closed_counterweight[0][2] - 0.5,
        details=f"closed_counterweight={closed_counterweight}, open_counterweight={open_counterweight}",
    )

    return ctx.report()


object_model = build_object_model()
