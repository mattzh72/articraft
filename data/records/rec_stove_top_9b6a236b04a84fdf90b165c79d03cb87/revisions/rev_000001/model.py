from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


WIDTH = 1.90
DEPTH = 1.05
FRONT_FACE_Y = -0.60
DECK_TOP_Z = 0.22
KNOB_Z = 0.105
KNOB_XS = (-0.72, -0.43, -0.14, 0.14, 0.43, 0.72)
BURNER_CENTERS = (
    (-0.60, -0.18),
    (0.00, -0.18),
    (0.60, -0.18),
    (-0.60, 0.25),
    (0.00, 0.25),
    (0.60, 0.25),
)


def _add_grate(body, x: float, y: float, i: int, cast_iron) -> None:
    """Add one heavy restaurant-style cast grate, with feet on the deck."""
    z_bar = DECK_TOP_Z + 0.083
    bar_h = 0.030
    bar_w = 0.034
    span_x = 0.42
    span_y = 0.34

    # Perimeter members and cross bars form a continuous cast-iron grate.
    body.visual(
        Box((span_x, bar_w, bar_h)),
        origin=Origin(xyz=(x, y - span_y / 2, z_bar)),
        material=cast_iron,
        name=f"grate_{i}_front_bar",
    )
    body.visual(
        Box((span_x, bar_w, bar_h)),
        origin=Origin(xyz=(x, y + span_y / 2, z_bar)),
        material=cast_iron,
        name=f"grate_{i}_rear_bar",
    )
    body.visual(
        Box((bar_w, span_y, bar_h)),
        origin=Origin(xyz=(x - span_x / 2, y, z_bar)),
        material=cast_iron,
        name=f"grate_{i}_side_bar_0",
    )
    body.visual(
        Box((bar_w, span_y, bar_h)),
        origin=Origin(xyz=(x + span_x / 2, y, z_bar)),
        material=cast_iron,
        name=f"grate_{i}_side_bar_1",
    )
    body.visual(
        Box((span_x, bar_w, bar_h)),
        origin=Origin(xyz=(x, y, z_bar)),
        material=cast_iron,
        name=f"grate_{i}_cross_bar_x",
    )
    body.visual(
        Box((bar_w, span_y, bar_h)),
        origin=Origin(xyz=(x, y, z_bar)),
        material=cast_iron,
        name=f"grate_{i}_cross_bar_y",
    )

    foot_h = z_bar - bar_h / 2 - DECK_TOP_Z
    for j, (dx, dy) in enumerate(
        (
            (-span_x / 2, -span_y / 2),
            (span_x / 2, -span_y / 2),
            (-span_x / 2, span_y / 2),
            (span_x / 2, span_y / 2),
        )
    ):
        body.visual(
            Box((0.055, 0.055, foot_h)),
            origin=Origin(xyz=(x + dx, y + dy, DECK_TOP_Z + foot_h / 2)),
            material=cast_iron,
            name=f"grate_{i}_foot_{j}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="six_burner_commercial_gas_stovetop")

    stainless = model.material("brushed_stainless", rgba=(0.62, 0.64, 0.62, 1.0))
    dark_stainless = model.material("dark_stainless", rgba=(0.30, 0.31, 0.30, 1.0))
    cast_iron = model.material("seasoned_cast_iron", rgba=(0.015, 0.014, 0.013, 1.0))
    burner_black = model.material("black_burner_caps", rgba=(0.025, 0.023, 0.020, 1.0))
    brass = model.material("aged_brass", rgba=(0.78, 0.58, 0.25, 1.0))
    white_mark = model.material("white_indicator_paint", rgba=(0.92, 0.88, 0.76, 1.0))

    burner_ring_mesh = mesh_from_geometry(
        TorusGeometry(0.092, 0.010, radial_segments=20, tubular_segments=48),
        "burner_port_ring",
    )
    collar_mesh = mesh_from_geometry(
        TorusGeometry(0.034, 0.0045, radial_segments=16, tubular_segments=36),
        "front_shaft_bezel",
    )
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.078,
            0.038,
            body_style="skirted",
            top_diameter=0.060,
            edge_radius=0.002,
            skirt=KnobSkirt(0.092, 0.009, flare=0.06, chamfer=0.0015),
            grip=KnobGrip(style="fluted", count=20, depth=0.0024),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0009),
            bore=KnobBore(style="d_shaft", diameter=0.014, flat_depth=0.0025),
            center=False,
        ),
        "heavy_range_knob",
    )

    body = model.part("body")
    body.visual(
        Box((WIDTH, DEPTH, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=dark_stainless,
        name="lower_pan",
    )
    body.visual(
        Box((WIDTH, DEPTH, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.190)),
        material=stainless,
        name="flat_cooktop_deck",
    )
    body.visual(
        Box((WIDTH, 0.12, 0.18)),
        origin=Origin(xyz=(0.0, -0.54, 0.13)),
        material=stainless,
        name="front_control_rail",
    )
    body.visual(
        Box((WIDTH, 0.055, 0.095)),
        origin=Origin(xyz=(0.0, 0.525, 0.267)),
        material=stainless,
        name="rear_splash_lip",
    )
    body.visual(
        Box((0.045, DEPTH, 0.055)),
        origin=Origin(xyz=(-0.972, 0.0, 0.247)),
        material=stainless,
        name="side_lip_0",
    )
    body.visual(
        Box((0.045, DEPTH, 0.055)),
        origin=Origin(xyz=(0.972, 0.0, 0.247)),
        material=stainless,
        name="side_lip_1",
    )

    # Six burner wells and raised cast grates in two rows.
    for i, (x, y) in enumerate(BURNER_CENTERS):
        body.visual(
            Cylinder(radius=0.160, length=0.010),
            origin=Origin(xyz=(x, y, DECK_TOP_Z + 0.005)),
            material=dark_stainless,
            name=f"burner_{i}_recess_pan",
        )
        body.visual(
            Cylinder(radius=0.070, length=0.020),
            origin=Origin(xyz=(x, y, DECK_TOP_Z + 0.020)),
            material=burner_black,
            name=f"burner_{i}_cap",
        )
        body.visual(
            burner_ring_mesh,
            origin=Origin(xyz=(x, y, DECK_TOP_Z + 0.020)),
            material=brass,
            name=f"burner_{i}_port_ring",
        )
        body.visual(
            Cylinder(radius=0.035, length=0.012),
            origin=Origin(xyz=(x, y, DECK_TOP_Z + 0.036)),
            material=burner_black,
            name=f"burner_{i}_center_cap",
        )
        _add_grate(body, x, y, i, cast_iron)

    # Front shaft bezels mounted around the six knob axes.
    for i, x in enumerate(KNOB_XS):
        body.visual(
            collar_mesh,
            origin=Origin(xyz=(x, FRONT_FACE_Y - 0.0045, KNOB_Z), rpy=(math.pi / 2, 0.0, 0.0)),
            material=stainless,
            name=f"knob_{i}_bezel",
        )

    # Six independently continuous rotary knobs on front-facing shafts.
    for i, x in enumerate(KNOB_XS):
        knob = model.part(f"knob_{i}")
        shaft_len = 0.055
        knob.visual(
            Cylinder(radius=0.015, length=shaft_len),
            origin=Origin(xyz=(0.0, -shaft_len / 2, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
            material=dark_stainless,
            name="shaft",
        )
        knob.visual(
            knob_mesh,
            origin=Origin(xyz=(0.0, -shaft_len + 0.002, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
            material=cast_iron,
            name="cap",
        )
        knob.visual(
            Box((0.007, 0.003, 0.045)),
            origin=Origin(xyz=(0.0, -shaft_len - 0.037, 0.015)),
            material=white_mark,
            name="pointer_mark",
        )
        model.articulation(
            f"body_to_knob_{i}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=knob,
            origin=Origin(xyz=(x, FRONT_FACE_Y, KNOB_Z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=1.2, velocity=7.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")

    knob_parts = [object_model.get_part(f"knob_{i}") for i in range(6)]
    knob_joints = [object_model.get_articulation(f"body_to_knob_{i}") for i in range(6)]
    ctx.check("six separate control knobs", len(knob_parts) == 6)
    ctx.check("six continuous knob shafts", all(j.articulation_type == ArticulationType.CONTINUOUS for j in knob_joints))
    ctx.check("knob axes face the operator", all(tuple(j.axis) == (0.0, -1.0, 0.0) for j in knob_joints))

    burner_caps = [v for v in body.visuals if v.name and v.name.endswith("_center_cap")]
    burner_rows = sorted({round(y, 3) for _, y in BURNER_CENTERS})
    burner_cols = sorted({round(x, 3) for x, _ in BURNER_CENTERS})
    ctx.check("six burners on the cooktop", len(burner_caps) == 6)
    ctx.check("burners arranged as two rows", len(burner_rows) == 2 and len(burner_cols) == 3)

    for i, knob in enumerate(knob_parts):
        ctx.expect_gap(
            body,
            knob,
            axis="y",
            min_gap=0.035,
            positive_elem="front_control_rail",
            negative_elem="cap",
            name=f"knob_{i} cap stands proud of front rail",
        )
        ctx.expect_overlap(
            body,
            knob,
            axes="xz",
            elem_a="front_control_rail",
            elem_b="shaft",
            min_overlap=0.020,
            name=f"knob_{i} shaft is centered on the rail",
        )

    with ctx.pose({"body_to_knob_0": math.pi / 2}):
        aabb = ctx.part_element_world_aabb("knob_0", elem="pointer_mark")
        rotated_marker_is_wide = False
        if aabb is not None:
            lo, hi = aabb
            rotated_marker_is_wide = (hi[0] - lo[0]) > (hi[2] - lo[2]) + 0.015
        ctx.check(
            "knob pointer rotates with continuous shaft",
            rotated_marker_is_wide,
            details=f"pointer_mark aabb at quarter turn={aabb}",
        )

    return ctx.report()


object_model = build_object_model()
