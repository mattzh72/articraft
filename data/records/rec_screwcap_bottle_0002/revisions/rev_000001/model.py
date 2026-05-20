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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    section_loft,
    superellipse_profile,
)

ASSETS = AssetContext.from_script(__file__)
PROFILE_SEGMENTS = 56


def _superellipse_section(
    width: float,
    depth: float,
    z: float,
    *,
    exponent: float,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z)
        for x, y in superellipse_profile(
            width,
            depth,
            exponent=exponent,
            segments=PROFILE_SEGMENTS,
        )
    ]


def _write_mesh(name: str, geometry) -> object:
    return mesh_from_geometry(geometry, ASSETS.mesh_path(f"{name}.obj"))


def _build_body_shell_mesh() -> object:
    sections = [
        _superellipse_section(0.072, 0.058, 0.000, exponent=3.0),
        _superellipse_section(0.082, 0.066, 0.014, exponent=3.2),
        _superellipse_section(0.084, 0.068, 0.060, exponent=3.6),
        _superellipse_section(0.084, 0.068, 0.155, exponent=3.8),
        _superellipse_section(0.074, 0.060, 0.194, exponent=3.3),
        _superellipse_section(0.052, 0.046, 0.214, exponent=2.8),
    ]
    return _write_mesh(
        "rugged_utility_bottle_body_shell",
        repair_loft(section_loft(sections)),
    )


def _build_cap_shell_mesh() -> object:
    profile = [
        (0.000, 0.000),
        (0.0265, 0.000),
        (0.0290, 0.004),
        (0.0295, 0.026),
        (0.0260, 0.034),
        (0.000, 0.034),
    ]
    return _write_mesh(
        "rugged_utility_bottle_cap_shell",
        LatheGeometry(profile, segments=60),
    )


def _add_knurl_band(part, *, radius: float, z_center: float, height: float, count: int, material) -> None:
    for index in range(count):
        angle = math.tau * index / count
        part.visual(
            Box((0.004, 0.009, height)),
            origin=Origin(
                xyz=(radius * math.cos(angle), radius * math.sin(angle), z_center),
                rpy=(0.0, 0.0, angle),
            ),
            material=material,
            name=f"knurl_{index:02d}",
        )


def _add_side_fastener(part, *, name: str, x: float, y: float, z: float, material) -> None:
    part.visual(
        Cylinder(radius=0.0042, length=0.003),
        origin=Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_utility_screwcap_bottle", assets=ASSETS)

    body_polymer = model.material("body_polymer", rgba=(0.43, 0.46, 0.47, 1.0))
    cap_polymer = model.material("cap_polymer", rgba=(0.11, 0.12, 0.13, 1.0))
    rubber = model.material("rubber", rgba=(0.09, 0.10, 0.11, 1.0))
    painted_metal = model.material("painted_metal", rgba=(0.28, 0.31, 0.34, 1.0))
    fastener_steel = model.material("fastener_steel", rgba=(0.57, 0.59, 0.62, 1.0))

    bottle = model.part("bottle")
    bottle.visual(_build_body_shell_mesh(), material=body_polymer, name="body_shell")
    bottle.visual(
        Cylinder(radius=0.026, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.216)),
        material=body_polymer,
        name="shoulder_finish",
    )
    bottle.visual(
        Cylinder(radius=0.0175, length=0.033),
        origin=Origin(xyz=(0.0, 0.0, 0.2355)),
        material=body_polymer,
        name="neck_finish",
    )
    for index, z in enumerate((0.227, 0.233, 0.239), start=1):
        bottle.visual(
            Cylinder(radius=0.0192, length=0.003),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=body_polymer,
            name=f"thread_band_{index}",
        )
    for face_name, y in (("front", 0.036), ("rear", -0.036)):
        for index, x in enumerate((-0.020, 0.0, 0.020), start=1):
            bottle.visual(
                Box((0.012, 0.006, 0.115)),
                origin=Origin(xyz=(x, y, 0.102)),
                material=rubber,
                name=f"{face_name}_grip_{index}",
            )
    bottle.inertial = Inertial.from_geometry(
        Box((0.086, 0.072, 0.255)),
        mass=0.68,
        origin=Origin(xyz=(0.0, 0.0, 0.118)),
    )

    armor = model.part("armor")
    armor.visual(
        Box((0.020, 0.072, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=painted_metal,
        name="base_skid",
    )
    armor.visual(
        Box((0.082, 0.008, 0.020)),
        origin=Origin(xyz=(0.0, 0.036, 0.010)),
        material=rubber,
        name="front_bumper",
    )
    armor.visual(
        Box((0.082, 0.008, 0.020)),
        origin=Origin(xyz=(0.0, -0.036, 0.010)),
        material=rubber,
        name="rear_bumper",
    )
    armor.visual(
        Box((0.008, 0.018, 0.210)),
        origin=Origin(xyz=(0.045, 0.0, 0.112)),
        material=painted_metal,
        name="right_rail",
    )
    armor.visual(
        Box((0.008, 0.018, 0.210)),
        origin=Origin(xyz=(-0.045, 0.0, 0.112)),
        material=painted_metal,
        name="left_rail",
    )
    armor.visual(
        Box((0.040, 0.018, 0.028)),
        origin=Origin(xyz=(0.026, 0.0, 0.014)),
        material=painted_metal,
        name="right_lower_brace",
    )
    armor.visual(
        Box((0.040, 0.018, 0.028)),
        origin=Origin(xyz=(-0.026, 0.0, 0.014)),
        material=painted_metal,
        name="left_lower_brace",
    )
    armor.visual(
        Box((0.082, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, 0.031, 0.206)),
        material=painted_metal,
        name="front_bridge",
    )
    armor.visual(
        Box((0.082, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, -0.031, 0.206)),
        material=painted_metal,
        name="rear_bridge",
    )
    armor.visual(
        Box((0.010, 0.020, 0.034)),
        origin=Origin(xyz=(0.046, 0.0, 0.226)),
        material=painted_metal,
        name="right_guard",
    )
    armor.visual(
        Box((0.010, 0.020, 0.034)),
        origin=Origin(xyz=(-0.046, 0.0, 0.226)),
        material=painted_metal,
        name="left_guard",
    )
    for side_name, x in (("right", 0.043), ("left", -0.043)):
        for edge_name, y in (("front", 0.020), ("rear", -0.020)):
            armor.visual(
                Box((0.012, 0.024, 0.016)),
                origin=Origin(xyz=(x, y, 0.206)),
                material=painted_metal,
                name=f"{side_name}_{edge_name}_gusset",
            )
    for side_name, x in (("right", 0.049), ("left", -0.049)):
        for index, z in enumerate((0.074, 0.148), start=1):
            _add_side_fastener(
                armor,
                name=f"{side_name}_rail_fastener_{index}",
                x=x,
                y=0.0,
                z=z,
                material=fastener_steel,
            )
        for index, y in enumerate((-0.006, 0.006), start=1):
            _add_side_fastener(
                armor,
                name=f"{side_name}_guard_fastener_{index}",
                x=x,
                y=y,
                z=0.226,
                material=fastener_steel,
            )
    armor.inertial = Inertial.from_geometry(
        Box((0.094, 0.080, 0.235)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
    )
    model.articulation(
        "bottle_to_armor",
        ArticulationType.FIXED,
        parent="bottle",
        child="armor",
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    cap = model.part("cap")
    cap.visual(
        _build_cap_shell_mesh(),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=cap_polymer,
        name="cap_skirt",
    )
    cap.visual(
        Cylinder(radius=0.021, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=cap_polymer,
        name="cap_inner_sleeve",
    )
    for index, z in enumerate((0.007, 0.013, 0.019), start=1):
        cap.visual(
            Cylinder(radius=0.0222, length=0.0025),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=cap_polymer,
            name=f"cap_thread_{index}",
        )
    _add_knurl_band(
        cap,
        radius=0.0305,
        z_center=0.024,
        height=0.022,
        count=18,
        material=cap_polymer,
    )
    cap.visual(
        Box((0.010, 0.008, 0.018)),
        origin=Origin(xyz=(0.033, 0.0, 0.024)),
        material=cap_polymer,
        name="tether_lug",
    )
    cap.visual(
        Box((0.030, 0.006, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        material=painted_metal,
        name="top_reinforcement_bar",
    )
    cap.visual(
        Box((0.006, 0.030, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        material=painted_metal,
        name="top_reinforcement_cross",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=0.031, length=0.046),
        mass=0.09,
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
    )
    model.articulation(
        "cap_spin",
        ArticulationType.CONTINUOUS,
        parent="bottle",
        child="cap",
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.5, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_part_geometry_disconnected()
    ctx.allow_overlap(
        "bottle",
        "cap",
        reason="The cap's inner sleeve intentionally nests over the threaded neck in the closed serviceable pose.",
    )
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_aabb_contact("armor", "bottle")
    ctx.expect_origin_distance("armor", "bottle", axes="xy", max_dist=0.001)
    ctx.expect_aabb_overlap("armor", "bottle", axes="xy", min_overlap=0.060)
    ctx.expect_aabb_overlap("armor", "bottle", axes="z", min_overlap=0.210)

    ctx.expect_aabb_contact("cap", "bottle")
    ctx.expect_origin_distance("cap", "bottle", axes="xy", max_dist=0.0015)
    ctx.expect_aabb_overlap("cap", "bottle", axes="xy", min_overlap=0.034)
    ctx.expect_aabb_gap(
        "cap",
        "bottle",
        axis="z",
        min_gap=0.0002,
        max_gap=0.006,
        positive_elem="cap_skirt",
        negative_elem="thread_band_1",
        name="cap_shell_starts_just_above_lower_thread_band",
    )
    ctx.expect_aabb_gap(
        "cap",
        "bottle",
        axis="z",
        max_gap=0.001,
        max_penetration=0.005,
        positive_elem="cap_inner_sleeve",
        negative_elem="shoulder_finish",
        name="cap_inner_sleeve_reaches_down_to_shoulder_finish_for_engagement_depth",
    )
    ctx.expect_aabb_gap(
        "cap",
        "bottle",
        axis="z",
        max_gap=0.0005,
        max_penetration=0.003,
        positive_elem="cap_thread_2",
        negative_elem="thread_band_2",
        name="mid_thread_pair_occupies_same_engagement_band",
    )
    ctx.expect_aabb_gap(
        "cap",
        "bottle",
        axis="z",
        max_gap=0.0005,
        max_penetration=0.003,
        positive_elem="cap_thread_3",
        negative_elem="thread_band_3",
        name="upper_thread_pair_occupies_same_engagement_band",
    )

    with ctx.pose(cap_spin=math.pi / 2.0):
        ctx.expect_origin_distance("cap", "bottle", axes="xy", max_dist=0.0015)
        ctx.expect_aabb_overlap("cap", "bottle", axes="xy", min_overlap=0.034)

    with ctx.pose(cap_spin=0.0):
        ctx.expect_aabb_gap(
            "armor",
            "cap",
            axis="x",
            min_gap=0.002,
            max_gap=0.010,
            positive_elem="right_guard",
            negative_elem="tether_lug",
            name="tether_lug_clears_right_guard_in_closed_orientation",
        )

    with ctx.pose(cap_spin=math.pi):
        ctx.expect_aabb_gap(
            "cap",
            "armor",
            axis="x",
            min_gap=0.002,
            max_gap=0.010,
            positive_elem="tether_lug",
            negative_elem="left_guard",
            name="tether_lug_tracks_to_left_guard_after_half_turn",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
