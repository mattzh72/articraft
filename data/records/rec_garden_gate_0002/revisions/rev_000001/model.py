from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)

POST_SIZE = 0.12
POST_HEIGHT = 1.30
POST_CAP_HEIGHT = 0.02
POST_CENTER_Z = POST_HEIGHT * 0.5

HINGE_POST_X = 0.0
LATCH_POST_X = 1.145

FOOTING_LENGTH = 1.39
FOOTING_WIDTH = 0.18
FOOTING_HEIGHT = 0.12
FOOTING_CENTER_X = 0.5725
FOOTING_CENTER_Z = -FOOTING_HEIGHT * 0.5

GATE_WIDTH = 0.95
GATE_HEIGHT = 0.92
GATE_THICKNESS = 0.035
FRAME_MEMBER = 0.04
PICKET_WIDTH = 0.022
PICKET_THICKNESS = 0.018
PICKET_COUNT = 8

HINGE_AXIS_X = 0.08
UPPER_HINGE_Z = 0.83
LOWER_HINGE_Z = 0.25
HINGE_RADIUS = 0.014
HINGE_PIN_RADIUS = 0.006
GATE_KNUCKLE_LEN = 0.028
POST_KNUCKLE_LEN = 0.016
HINGE_SEGMENT_GAP = 0.002

LATCH_CENTER_Z = 0.55
LATCH_STOP_LENGTH = 0.018
LATCH_STOP_THICKNESS = 0.024
GATE_FRAME_OFFSET_X = 0.034

SWING_LIMIT = math.radians(100.0)


def _box(part, name: str, size: tuple[float, float, float], xyz: tuple[float, float, float], material, rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _cylinder(
    part,
    name: str,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _strap_plate_mesh(name: str, *, root_x: float, length: float, root_height: float, tip_height: float, thickness: float):
    shoulder_x = root_x + length * 0.68
    tip_x = root_x + length
    half_root = root_height * 0.5
    half_tip = tip_height * 0.5
    profile = [
        (root_x, -half_root),
        (shoulder_x, -half_root),
        (tip_x, -half_tip),
        (tip_x, half_tip),
        (shoulder_x, half_root),
        (root_x, half_root),
    ]
    return _save_mesh(
        name,
        ExtrudeGeometry(profile, height=thickness, center=True).rotate_x(math.pi * 0.5),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="garden_gate", assets=ASSETS)

    concrete = model.material("concrete", rgba=(0.69, 0.69, 0.67, 1.0))
    timber_post = model.material("timber_post", rgba=(0.41, 0.28, 0.16, 1.0))
    timber_gate = model.material("timber_gate", rgba=(0.47, 0.31, 0.18, 1.0))
    hinge_steel = model.material("hinge_steel", rgba=(0.59, 0.61, 0.63, 1.0))
    latch_steel = model.material("latch_steel", rgba=(0.19, 0.20, 0.22, 1.0))

    footing = model.part("footing")
    footing.inertial = Inertial.from_geometry(
        Box((FOOTING_LENGTH, FOOTING_WIDTH, FOOTING_HEIGHT)),
        mass=95.0,
        origin=Origin(xyz=(FOOTING_CENTER_X, 0.0, FOOTING_CENTER_Z)),
    )
    _box(
        footing,
        "sill",
        (FOOTING_LENGTH, FOOTING_WIDTH, FOOTING_HEIGHT),
        (FOOTING_CENTER_X, 0.0, FOOTING_CENTER_Z),
        concrete,
    )

    hinge_post = model.part("hinge_post")
    hinge_post.inertial = Inertial.from_geometry(
        Box((POST_SIZE, POST_SIZE, POST_HEIGHT)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, POST_CENTER_Z)),
    )
    _box(
        hinge_post,
        "post_shaft",
        (POST_SIZE, POST_SIZE, POST_HEIGHT),
        (0.0, 0.0, POST_CENTER_Z),
        timber_post,
    )
    _box(
        hinge_post,
        "post_cap",
        (POST_SIZE + 0.02, POST_SIZE + 0.02, POST_CAP_HEIGHT),
        (0.0, 0.0, POST_HEIGHT + POST_CAP_HEIGHT * 0.5),
        timber_post,
    )

    upper_post_offset = GATE_KNUCKLE_LEN * 0.5 + HINGE_SEGMENT_GAP + POST_KNUCKLE_LEN * 0.5
    for suffix, z_world in (
        ("upper", UPPER_HINGE_Z + upper_post_offset),
        ("lower", UPPER_HINGE_Z - upper_post_offset),
    ):
        _cylinder(
            hinge_post,
            f"upper_post_knuckle_{suffix}",
            HINGE_RADIUS,
            POST_KNUCKLE_LEN,
            (HINGE_AXIS_X, 0.0, z_world),
            hinge_steel,
        )
    _cylinder(
        hinge_post,
        "upper_hinge_pin",
        HINGE_PIN_RADIUS,
        GATE_KNUCKLE_LEN + 2.0 * POST_KNUCKLE_LEN + 2.0 * HINGE_SEGMENT_GAP,
        (HINGE_AXIS_X, 0.0, UPPER_HINGE_Z),
        hinge_steel,
    )
    _box(
        hinge_post,
        "upper_post_strap",
        (0.018, 0.05, 0.055),
        (0.060, 0.0, UPPER_HINGE_Z),
        hinge_steel,
    )

    for suffix, z_world in (
        ("upper", LOWER_HINGE_Z + upper_post_offset),
        ("lower", LOWER_HINGE_Z - upper_post_offset),
    ):
        _cylinder(
            hinge_post,
            f"lower_post_knuckle_{suffix}",
            HINGE_RADIUS,
            POST_KNUCKLE_LEN,
            (HINGE_AXIS_X, 0.0, z_world),
            hinge_steel,
        )
    _cylinder(
        hinge_post,
        "lower_hinge_pin",
        HINGE_PIN_RADIUS,
        GATE_KNUCKLE_LEN + 2.0 * POST_KNUCKLE_LEN + 2.0 * HINGE_SEGMENT_GAP,
        (HINGE_AXIS_X, 0.0, LOWER_HINGE_Z),
        hinge_steel,
    )
    _box(
        hinge_post,
        "lower_post_strap",
        (0.018, 0.05, 0.055),
        (0.060, 0.0, LOWER_HINGE_Z),
        hinge_steel,
    )

    latch_post = model.part("latch_post")
    latch_post.inertial = Inertial.from_geometry(
        Box((POST_SIZE, POST_SIZE, POST_HEIGHT)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, POST_CENTER_Z)),
    )
    _box(
        latch_post,
        "post_shaft",
        (POST_SIZE, POST_SIZE, POST_HEIGHT),
        (0.0, 0.0, POST_CENTER_Z),
        timber_post,
    )
    _box(
        latch_post,
        "post_cap",
        (POST_SIZE + 0.02, POST_SIZE + 0.02, POST_CAP_HEIGHT),
        (0.0, 0.0, POST_HEIGHT + POST_CAP_HEIGHT * 0.5),
        timber_post,
    )
    _box(
        latch_post,
        "strike_plate",
        (0.008, 0.060, 0.120),
        (-0.056, 0.0, LATCH_CENTER_Z),
        latch_steel,
    )

    gate_leaf = model.part("gate_leaf")
    gate_leaf.inertial = Inertial.from_geometry(
        Box((GATE_WIDTH, GATE_THICKNESS, GATE_HEIGHT)),
        mass=18.0,
        origin=Origin(
            xyz=(
                GATE_FRAME_OFFSET_X + GATE_WIDTH * 0.5,
                0.0,
                (0.08 + GATE_HEIGHT) * 0.5 - UPPER_HINGE_Z,
            )
        ),
    )

    gate_center_z = (0.08 + 1.00) * 0.5 - UPPER_HINGE_Z
    _box(
        gate_leaf,
        "left_stile",
        (FRAME_MEMBER, GATE_THICKNESS, GATE_HEIGHT),
        (GATE_FRAME_OFFSET_X + FRAME_MEMBER * 0.5, 0.0, gate_center_z),
        timber_gate,
    )
    _box(
        gate_leaf,
        "right_stile",
        (FRAME_MEMBER, GATE_THICKNESS, GATE_HEIGHT),
        (GATE_FRAME_OFFSET_X + GATE_WIDTH - FRAME_MEMBER * 0.5, 0.0, gate_center_z),
        timber_gate,
    )
    _box(
        gate_leaf,
        "top_rail",
        (GATE_WIDTH, GATE_THICKNESS, FRAME_MEMBER),
        (GATE_FRAME_OFFSET_X + GATE_WIDTH * 0.5, 0.0, 1.00 - FRAME_MEMBER * 0.5 - UPPER_HINGE_Z),
        timber_gate,
    )
    _box(
        gate_leaf,
        "bottom_rail",
        (GATE_WIDTH, GATE_THICKNESS, FRAME_MEMBER),
        (GATE_FRAME_OFFSET_X + GATE_WIDTH * 0.5, 0.0, 0.08 + FRAME_MEMBER * 0.5 - UPPER_HINGE_Z),
        timber_gate,
    )

    picket_height = GATE_HEIGHT - 2.0 * FRAME_MEMBER
    picket_gap = (GATE_WIDTH - 2.0 * FRAME_MEMBER - PICKET_COUNT * PICKET_WIDTH) / (PICKET_COUNT + 1)
    picket_center_z = gate_center_z
    for index in range(PICKET_COUNT):
        x_min = FRAME_MEMBER + picket_gap + index * (PICKET_WIDTH + picket_gap)
        _box(
            gate_leaf,
            f"picket_{index + 1}",
            (PICKET_WIDTH, PICKET_THICKNESS, picket_height),
            (GATE_FRAME_OFFSET_X + x_min + PICKET_WIDTH * 0.5, 0.0, picket_center_z),
            timber_gate,
        )

    brace_x0 = GATE_FRAME_OFFSET_X + FRAME_MEMBER + 0.05
    brace_x1 = GATE_FRAME_OFFSET_X + GATE_WIDTH - FRAME_MEMBER - 0.04
    brace_z0 = 0.08 + FRAME_MEMBER + 0.04 - UPPER_HINGE_Z
    brace_z1 = 1.00 - FRAME_MEMBER - 0.05 - UPPER_HINGE_Z
    brace_dx = brace_x1 - brace_x0
    brace_dz = brace_z1 - brace_z0
    brace_len = math.hypot(brace_dx, brace_dz)
    brace_angle = math.atan2(brace_dz, brace_dx)
    _box(
        gate_leaf,
        "brace",
        (brace_len, 0.024, 0.035),
        ((brace_x0 + brace_x1) * 0.5, 0.0, (brace_z0 + brace_z1) * 0.5),
        timber_gate,
        rpy=(0.0, -brace_angle, 0.0),
    )

    _cylinder(
        gate_leaf,
        "upper_gate_knuckle",
        HINGE_RADIUS,
        GATE_KNUCKLE_LEN,
        (0.0, 0.0, 0.0),
        hinge_steel,
    )
    upper_leaf_strap_mesh = _strap_plate_mesh(
        "upper_leaf_strap.obj",
        root_x=HINGE_RADIUS,
        length=0.166 - HINGE_RADIUS,
        root_height=0.050,
        tip_height=0.016,
        thickness=0.050,
    )
    gate_leaf.visual(
        upper_leaf_strap_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=hinge_steel,
        name="upper_leaf_strap",
    )
    gate_leaf.visual(
        upper_leaf_strap_mesh,
        origin=Origin(xyz=(0.0, 0.0, LOWER_HINGE_Z - UPPER_HINGE_Z)),
        material=hinge_steel,
        name="lower_leaf_strap",
    )
    _box(
        gate_leaf,
        "latch_stop",
        (LATCH_STOP_LENGTH, LATCH_STOP_THICKNESS, 0.110),
        (GATE_FRAME_OFFSET_X + GATE_WIDTH + LATCH_STOP_LENGTH * 0.5 - 0.001, 0.0, LATCH_CENTER_Z - UPPER_HINGE_Z),
        latch_steel,
    )

    lower_hinge_barrel = model.part("lower_hinge_barrel")
    lower_hinge_barrel.inertial = Inertial.from_geometry(
        Cylinder(radius=HINGE_RADIUS, length=GATE_KNUCKLE_LEN),
        mass=0.8,
        origin=Origin(),
    )
    _cylinder(
        lower_hinge_barrel,
        "lower_gate_knuckle",
        HINGE_RADIUS,
        GATE_KNUCKLE_LEN,
        (0.0, 0.0, 0.0),
        hinge_steel,
    )

    model.articulation(
        "footing_to_hinge_post",
        ArticulationType.FIXED,
        parent=footing,
        child=hinge_post,
        origin=Origin(xyz=(HINGE_POST_X, 0.0, 0.0)),
    )
    model.articulation(
        "footing_to_latch_post",
        ArticulationType.FIXED,
        parent=footing,
        child=latch_post,
        origin=Origin(xyz=(LATCH_POST_X, 0.0, 0.0)),
    )
    model.articulation(
        "upper_hinge_barrel",
        ArticulationType.REVOLUTE,
        parent=hinge_post,
        child=gate_leaf,
        origin=Origin(xyz=(HINGE_AXIS_X, 0.0, UPPER_HINGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=24.0,
            velocity=1.8,
            lower=0.0,
            upper=SWING_LIMIT,
        ),
    )
    model.articulation(
        "lower_hinge_barrel",
        ArticulationType.REVOLUTE,
        parent=gate_leaf,
        child=lower_hinge_barrel,
        origin=Origin(xyz=(0.0, 0.0, LOWER_HINGE_Z - UPPER_HINGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=24.0,
            velocity=1.8,
            lower=0.0,
            upper=SWING_LIMIT,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)

    footing = object_model.get_part("footing")
    hinge_post = object_model.get_part("hinge_post")
    latch_post = object_model.get_part("latch_post")
    gate_leaf = object_model.get_part("gate_leaf")
    lower_hinge_barrel = object_model.get_part("lower_hinge_barrel")

    upper_hinge = object_model.get_articulation("upper_hinge_barrel")
    lower_hinge = object_model.get_articulation("lower_hinge_barrel")

    hinge_post_shaft = hinge_post.get_visual("post_shaft")
    latch_post_shaft = latch_post.get_visual("post_shaft")
    footing_sill = footing.get_visual("sill")
    upper_hinge_pin = hinge_post.get_visual("upper_hinge_pin")
    lower_hinge_pin = hinge_post.get_visual("lower_hinge_pin")
    strike_plate = latch_post.get_visual("strike_plate")
    right_stile = gate_leaf.get_visual("right_stile")
    upper_gate_knuckle = gate_leaf.get_visual("upper_gate_knuckle")
    lower_leaf_strap = gate_leaf.get_visual("lower_leaf_strap")
    latch_stop = gate_leaf.get_visual("latch_stop")
    lower_gate_knuckle = lower_hinge_barrel.get_visual("lower_gate_knuckle")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        hinge_post,
        gate_leaf,
        elem_a=upper_hinge_pin,
        elem_b=upper_gate_knuckle,
        reason="The upper hinge pin is represented as a solid pin passing through the gate knuckle.",
    )
    ctx.allow_overlap(
        hinge_post,
        lower_hinge_barrel,
        elem_a=lower_hinge_pin,
        elem_b=lower_gate_knuckle,
        reason="The lower hinge pin is represented as a solid pin passing through the lower hinge barrel.",
    )
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "all_parts_present",
        all(part is not None for part in (footing, hinge_post, latch_post, gate_leaf, lower_hinge_barrel)),
        "Expected footing, two posts, the gate leaf, and the lower hinge barrel.",
    )
    ctx.check(
        "two_hinge_joints_present",
        all(joint is not None for joint in (upper_hinge, lower_hinge)),
        "Expected upper and lower revolute hinge articulations.",
    )
    ctx.check(
        "hinge_axes_vertical",
        tuple(getattr(upper_hinge, "axis", ())) == (0.0, 0.0, 1.0)
        and tuple(getattr(lower_hinge, "axis", ())) == (0.0, 0.0, 1.0),
        "Both hinge barrels should rotate about the vertical Z axis.",
    )
    ctx.check(
        "hinge_swing_limits_match",
        abs(getattr(upper_hinge.motion_limits, "lower", -1.0) - 0.0) < 1e-9
        and abs(getattr(lower_hinge.motion_limits, "lower", -1.0) - 0.0) < 1e-9
        and abs(getattr(upper_hinge.motion_limits, "upper", 0.0) - SWING_LIMIT) < 1e-9
        and abs(getattr(lower_hinge.motion_limits, "upper", 0.0) - SWING_LIMIT) < 1e-9,
        "Both hinge barrels should share the same roughly 100 degree swing range.",
    )

    ctx.expect_gap(
        hinge_post,
        footing,
        axis="z",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem=hinge_post_shaft,
        negative_elem=footing_sill,
        name="hinge_post_seated_on_footing",
    )
    ctx.expect_gap(
        latch_post,
        footing,
        axis="z",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem=latch_post_shaft,
        negative_elem=footing_sill,
        name="latch_post_seated_on_footing",
    )
    ctx.expect_gap(gate_leaf, footing, axis="z", min_gap=0.079, max_gap=0.081)
    ctx.expect_gap(
        gate_leaf,
        lower_hinge_barrel,
        axis="x",
        min_gap=-1e-5,
        max_gap=0.001,
        positive_elem=lower_leaf_strap,
        negative_elem=lower_gate_knuckle,
        name="lower_barrel_seated_against_leaf_strap",
    )
    ctx.expect_gap(
        latch_post,
        gate_leaf,
        axis="x",
        min_gap=0.001,
        max_gap=0.005,
        positive_elem=strike_plate,
        negative_elem=latch_stop,
        name="closed_gate_meets_latch_post",
    )
    ctx.expect_gap(
        latch_post,
        gate_leaf,
        axis="x",
        min_gap=0.010,
        max_gap=0.030,
        positive_elem=latch_post_shaft,
        negative_elem=right_stile,
        name="gate_leaf_clears_latch_post_shaft",
    )

    gate_origin = ctx.part_world_position(gate_leaf)
    lower_origin = ctx.part_world_position(lower_hinge_barrel)
    same_axis_xy = gate_origin is not None and lower_origin is not None and abs(gate_origin[0] - lower_origin[0]) < 1e-6 and abs(gate_origin[1] - lower_origin[1]) < 1e-6
    ctx.check(
        "hinge_barrels_share_one_vertical_axis",
        same_axis_xy,
        "Upper and lower hinge barrel origins should stay on the same vertical line in plan.",
    )

    with ctx.pose({upper_hinge: SWING_LIMIT * 0.9, lower_hinge: SWING_LIMIT * 0.9}):
        ctx.expect_gap(gate_leaf, footing, axis="z", min_gap=0.079, max_gap=0.081)
        ctx.expect_gap(
            gate_leaf,
            latch_post,
            axis="y",
            min_gap=0.78,
            positive_elem=latch_stop,
            negative_elem=strike_plate,
            name="open_gate_swings_clear_of_latch_post",
        )
        open_gate_origin = ctx.part_world_position(gate_leaf)
        open_lower_origin = ctx.part_world_position(lower_hinge_barrel)
        open_same_axis_xy = (
            open_gate_origin is not None
            and open_lower_origin is not None
            and abs(open_gate_origin[0] - open_lower_origin[0]) < 1e-6
            and abs(open_gate_origin[1] - open_lower_origin[1]) < 1e-6
        )
        ctx.check(
            "hinge_axis_alignment_persists_in_open_pose",
            open_same_axis_xy,
            "The lower hinge barrel should stay aligned with the upper hinge axis as the gate swings.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
