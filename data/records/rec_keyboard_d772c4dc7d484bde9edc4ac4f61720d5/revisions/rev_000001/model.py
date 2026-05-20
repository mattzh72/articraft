from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_side_loft,
)
import cadquery as cq


KEY_UNIT = 0.019
KEY_GAP = 0.002
KEY_HEIGHT = 0.0085
KEY_BOTTOM_Z = 0.02995


def _tapered_keycap_geometry(width: float, depth: float, height: float) -> MeshGeometry:
    """Small sloped-sided keycap with a smaller top face."""
    geom = MeshGeometry()
    inset = min(0.0032, width * 0.16, depth * 0.16)
    bw, bd = width / 2.0, depth / 2.0
    tw, td = (width / 2.0) - inset, (depth / 2.0) - inset

    # Bottom loop, then top loop.
    pts = [
        (-bw, -bd, 0.0),
        (bw, -bd, 0.0),
        (bw, bd, 0.0),
        (-bw, bd, 0.0),
        (-tw, -td, height),
        (tw, -td, height),
        (tw, td, height),
        (-tw, td, height),
    ]
    for x, y, z in pts:
        geom.add_vertex(x, y, z)

    # Bottom, top, and four trapezoid sides.
    for face in (
        (0, 2, 1),
        (0, 3, 2),
        (4, 5, 6),
        (4, 6, 7),
        (0, 1, 5),
        (0, 5, 4),
        (1, 2, 6),
        (1, 6, 5),
        (2, 3, 7),
        (2, 7, 6),
        (3, 0, 4),
        (3, 4, 7),
    ):
        geom.add_face(*face)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tenkeyless_gaming_keyboard")

    model.material("case_graphite", rgba=(0.012, 0.014, 0.018, 1.0))
    model.material("top_plate", rgba=(0.025, 0.027, 0.033, 1.0))
    model.material("key_charcoal", rgba=(0.045, 0.047, 0.055, 1.0))
    model.material("key_modifier", rgba=(0.032, 0.034, 0.041, 1.0))
    model.material("key_accent", rgba=(0.11, 0.15, 0.19, 1.0))
    model.material("legend_soft_white", rgba=(0.72, 0.82, 0.92, 1.0))
    model.material("rgb_cyan", rgba=(0.0, 0.75, 1.0, 0.85))
    model.material("rgb_magenta", rgba=(0.85, 0.0, 0.95, 0.75))
    model.material("rubber_black", rgba=(0.006, 0.006, 0.007, 1.0))
    model.material("roller_dark_metal", rgba=(0.12, 0.12, 0.13, 1.0))

    case = model.part("case")
    case_shell = superellipse_side_loft(
        [
            (-0.074, 0.000, 0.016, 0.370),
            (-0.040, 0.000, 0.019, 0.380),
            (0.030, 0.000, 0.023, 0.388),
            (0.074, 0.000, 0.026, 0.392),
        ],
        exponents=3.6,
        segments=72,
    )
    case.visual(
        mesh_from_geometry(case_shell, "case_shell"),
        material="case_graphite",
        name="case_shell",
    )
    case.visual(
        Box((0.382, 0.150, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0143)),
        material="case_graphite",
        name="chassis_core",
    )
    case.visual(
        Box((0.352, 0.118, 0.0022)),
        origin=Origin(xyz=(-0.010, -0.006, 0.0257)),
        material="top_plate",
        name="key_deck",
    )
    case.visual(
        Box((0.345, 0.004, 0.0022)),
        origin=Origin(xyz=(-0.006, -0.075, 0.0095)),
        material="rgb_cyan",
        name="front_light_bar",
    )
    case.visual(
        Box((0.004, 0.105, 0.0020)),
        origin=Origin(xyz=(-0.189, -0.004, 0.0110)),
        material="rgb_magenta",
        name="side_light_bar_0",
    )
    case.visual(
        Box((0.004, 0.105, 0.0020)),
        origin=Origin(xyz=(0.189, -0.004, 0.0110)),
        material="rgb_magenta",
        name="side_light_bar_1",
    )

    # Raised cradle for the media roller at the rear-right corner.
    case.visual(
        Box((0.046, 0.024, 0.005)),
        origin=Origin(xyz=(0.176, 0.057, 0.0293)),
        material="top_plate",
        name="roller_plinth",
    )
    for suffix, x in (("inner", 0.159), ("outer", 0.195)):
        case.visual(
            Box((0.005, 0.016, 0.018)),
            origin=Origin(xyz=(x, 0.057, 0.0382)),
            material="case_graphite",
            name=f"roller_cheek_{suffix}",
        )

    # Rear underside hinge ears for the two folding feet.
    for i, x0 in enumerate((-0.105, 0.105)):
        for j, dx in enumerate((-0.024, 0.024)):
            case.visual(
                Box((0.006, 0.012, 0.008)),
                origin=Origin(xyz=(x0 + dx, 0.066, -0.002)),
                material="case_graphite",
                name=f"foot_ear_{i}_{j}",
            )

    key_meshes: dict[tuple[float, float], object] = {}

    def key_mesh(width: float, depth: float):
        key = (round(width, 4), round(depth, 4))
        if key not in key_meshes:
            key_meshes[key] = mesh_from_geometry(
                _tapered_keycap_geometry(width, depth, KEY_HEIGHT),
                f"keycap_{key[0]}_{key[1]}",
            )
        return key_meshes[key]

    def add_key(
        row: int,
        col: int,
        x: float,
        y: float,
        *,
        w_units: float = 1.0,
        d_units: float = 1.0,
        material: str = "key_charcoal",
        legend_len: float | None = None,
    ) -> None:
        width = KEY_UNIT * w_units - KEY_GAP
        depth = KEY_UNIT * d_units - KEY_GAP
        key = model.part(f"key_{row}_{col}")
        key.visual(
            key_mesh(width, depth),
            material=material,
            name="cap",
        )
        # A short plunger stem makes the cap read as seated in a switch socket.
        key.visual(
            Box((min(0.007, width * 0.42), min(0.007, depth * 0.42), 0.0034)),
            origin=Origin(xyz=(0.0, 0.0, -0.00145)),
            material="rubber_black",
            name="stem",
        )
        legend_width = legend_len if legend_len is not None else min(0.008, width * 0.45)
        key.visual(
            Box((legend_width, 0.0014, 0.00035)),
            origin=Origin(xyz=(0.0, -depth * 0.10, KEY_HEIGHT + 0.0001)),
            material="legend_soft_white",
            name="legend_mark",
        )
        model.articulation(
            f"case_to_key_{row}_{col}",
            ArticulationType.PRISMATIC,
            parent=case,
            child=key,
            origin=Origin(xyz=(x, y, KEY_BOTTOM_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=1.2, velocity=0.35, lower=-0.004, upper=0.0),
        )

    # Dense tenkeyless layout: main block, nav cluster, and function row.
    x0 = -0.158
    y_rows = [0.052, 0.031, 0.012, -0.007, -0.026, -0.049]

    function_xs = [
        -0.166,
        -0.126,
        -0.107,
        -0.088,
        -0.069,
        -0.038,
        -0.019,
        0.000,
        0.019,
        0.050,
        0.069,
        0.088,
        0.107,
        0.126,
        0.145,
    ]
    for col, x in enumerate(function_xs):
        mat = "key_modifier" if col in (0, 13, 14) else "key_charcoal"
        add_key(0, col, x, y_rows[0], material=mat)

    # Number row plus three-key navigation island.
    for col in range(13):
        add_key(1, col, x0 + KEY_UNIT * col, y_rows[1])
    add_key(1, 13, x0 + KEY_UNIT * 13.5, y_rows[1], w_units=2.0, material="key_modifier", legend_len=0.014)
    for col, x in enumerate((0.138, 0.157, 0.176), start=14):
        add_key(1, col, x, y_rows[1], material="key_modifier")

    # Q row.
    add_key(2, 0, x0 + KEY_UNIT * 0.25, y_rows[2], w_units=1.5, material="key_modifier", legend_len=0.012)
    for col in range(1, 13):
        mat = "key_accent" if col in (2, 3) else "key_charcoal"
        add_key(2, col, x0 + KEY_UNIT * (col + 0.5), y_rows[2], material=mat)
    add_key(2, 13, x0 + KEY_UNIT * 13.75, y_rows[2], w_units=1.5, material="key_modifier", legend_len=0.012)
    for col, x in enumerate((0.138, 0.157, 0.176), start=14):
        add_key(2, col, x, y_rows[2], material="key_modifier")

    # Home row.
    add_key(3, 0, x0 + KEY_UNIT * 0.375, y_rows[3], w_units=1.75, material="key_modifier", legend_len=0.013)
    for col in range(1, 12):
        mat = "key_accent" if col in (1, 2, 3) else "key_charcoal"
        add_key(3, col, x0 + KEY_UNIT * (col + 0.75), y_rows[3], material=mat)
    add_key(3, 12, x0 + KEY_UNIT * 13.125, y_rows[3], w_units=2.25, material="key_modifier", legend_len=0.016)

    # Shift row plus arrow-up.
    add_key(4, 0, x0 + KEY_UNIT * 0.625, y_rows[4], w_units=2.25, material="key_modifier", legend_len=0.016)
    for col in range(1, 11):
        mat = "key_accent" if col == 2 else "key_charcoal"
        add_key(4, col, x0 + KEY_UNIT * (col + 1.25), y_rows[4], material=mat)
    add_key(4, 11, x0 + KEY_UNIT * 13.375, y_rows[4], w_units=2.75, material="key_modifier", legend_len=0.018)
    add_key(4, 12, 0.157, y_rows[4], material="key_modifier")

    # Bottom row with long spacebar and arrow cluster.
    bottom_specs = [
        (0, 0.625, 1.25),
        (1, 1.875, 1.25),
        (2, 3.125, 1.25),
        (3, 4.375, 1.25),
        (4, 8.125, 6.25),
        (5, 11.875, 1.25),
        (6, 13.125, 1.25),
        (7, 14.375, 1.25),
    ]
    for col, center_units, width_units in bottom_specs:
        add_key(
            5,
            col,
            x0 + KEY_UNIT * center_units,
            y_rows[5],
            w_units=width_units,
            material="key_modifier",
            legend_len=0.020 if width_units > 3.0 else 0.010,
        )
    for col, x in enumerate((0.138, 0.157, 0.176), start=8):
        add_key(5, col, x, y_rows[5], material="key_modifier")

    # Ribbed media roller, held between the cheek brackets.
    roller = model.part("media_roller")
    roller_geom = KnobGeometry(
        0.013,
        0.030,
        body_style="cylindrical",
        grip=KnobGrip(style="ribbed", count=28, depth=0.0007, width=0.0010),
        edge_radius=0.0007,
    )
    roller.visual(
        mesh_from_geometry(roller_geom, "media_roller"),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material="roller_dark_metal",
        name="ribbed_barrel",
    )
    model.articulation(
        "case_to_media_roller",
        ArticulationType.CONTINUOUS,
        parent=case,
        child=roller,
        origin=Origin(xyz=(0.178, 0.057, 0.0392)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=12.0),
    )

    # Two folding rear feet; at rest they are deployed at a shallow angle.
    for i, x0_foot in enumerate((-0.105, 0.105)):
        foot = model.part(f"rear_foot_{i}")
        foot.visual(
            Cylinder(radius=0.003, length=0.052),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material="rubber_black",
            name="hinge_barrel",
        )
        foot.visual(
            Box((0.038, 0.033, 0.006)),
            origin=Origin(xyz=(0.0, 0.011, -0.013), rpy=(-0.82, 0.0, 0.0)),
            material="case_graphite",
            name="folding_leg",
        )
        foot.visual(
            Box((0.040, 0.010, 0.003)),
            origin=Origin(xyz=(0.0, 0.022, -0.026), rpy=(-0.82, 0.0, 0.0)),
            material="rubber_black",
            name="rubber_tip",
        )
        model.articulation(
            f"case_to_rear_foot_{i}",
            ArticulationType.REVOLUTE,
            parent=case,
            child=foot,
            origin=Origin(xyz=(x0_foot, 0.066, -0.004)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.6, velocity=3.0, lower=0.0, upper=1.05),
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
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    key_joints = [j for j in object_model.articulations if j.name.startswith("case_to_key_")]
    ctx.check(
        "dense tenkeyless key grid is articulated",
        len(key_joints) >= 80,
        details=f"found {len(key_joints)} key plunger joints",
    )

    sample_key = object_model.get_part("key_3_2")
    sample_joint = object_model.get_articulation("case_to_key_3_2")
    ctx.expect_gap(
        sample_key,
        "case",
        axis="z",
        max_gap=0.0008,
        max_penetration=0.0,
        positive_elem="stem",
        negative_elem="key_deck",
        name="key stem sits just above the switch deck",
    )
    rest_pos = ctx.part_world_position(sample_key)
    with ctx.pose({sample_joint: -0.004}):
        pressed_pos = ctx.part_world_position(sample_key)
    ctx.check(
        "representative key plunges downward",
        rest_pos is not None and pressed_pos is not None and pressed_pos[2] < rest_pos[2] - 0.003,
        details=f"rest={rest_pos}, pressed={pressed_pos}",
    )

    roller = object_model.get_part("media_roller")
    roller_joint = object_model.get_articulation("case_to_media_roller")
    ctx.expect_overlap(
        roller,
        "case",
        axes="xy",
        min_overlap=0.010,
        elem_a="ribbed_barrel",
        elem_b="roller_plinth",
        name="media roller is mounted over the upper-right cradle",
    )
    roller_rest = ctx.part_world_position(roller)
    with ctx.pose({roller_joint: 1.5}):
        roller_spun = ctx.part_world_position(roller)
    ctx.check(
        "media roller spins without translating",
        roller_rest is not None
        and roller_spun is not None
        and max(abs(a - b) for a, b in zip(roller_rest, roller_spun)) < 1e-6,
        details=f"rest={roller_rest}, spun={roller_spun}",
    )

    foot = object_model.get_part("rear_foot_0")
    foot_joint = object_model.get_articulation("case_to_rear_foot_0")
    ctx.expect_overlap(
        foot,
        "case",
        axes="x",
        min_overlap=0.004,
        elem_a="hinge_barrel",
        elem_b="foot_ear_0_0",
        name="rear foot hinge is captured by rear support ears",
    )
    foot_rest_aabb = ctx.part_element_world_aabb(foot, elem="rubber_tip")
    with ctx.pose({foot_joint: 0.9}):
        foot_folded_aabb = ctx.part_element_world_aabb(foot, elem="rubber_tip")
    ctx.check(
        "rear support foot rotates on hinge",
        foot_rest_aabb is not None
        and foot_folded_aabb is not None
        and foot_folded_aabb[0][2] > foot_rest_aabb[0][2] + 0.010,
        details=f"rest={foot_rest_aabb}, folded={foot_folded_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
