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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _add_vertical_bolt(
    part,
    *,
    xy: tuple[float, float],
    base_z: float,
    shank_length: float,
    shank_radius: float,
    head_radius: float,
    head_length: float,
    material,
) -> None:
    x, y = xy
    part.visual(
        Cylinder(radius=shank_radius, length=shank_length),
        origin=Origin(xyz=(x, y, base_z + shank_length * 0.5)),
        material=material,
    )
    part.visual(
        Cylinder(radius=head_radius, length=head_length),
        origin=Origin(xyz=(x, y, base_z + shank_length + head_length * 0.5)),
        material=material,
    )


def _add_fastener_ring(
    part,
    *,
    radius: float,
    z0: float,
    count: int,
    shank_length: float,
    shank_radius: float,
    head_radius: float,
    head_length: float,
    material,
) -> None:
    for i in range(count):
        angle = 2.0 * math.pi * i / count
        _add_vertical_bolt(
            part,
            xy=(radius * math.cos(angle), radius * math.sin(angle)),
            base_z=z0,
            shank_length=shank_length,
            shank_radius=shank_radius,
            head_radius=head_radius,
            head_length=head_length,
            material=material,
        )


def _yz_section(x: float, width: float, height: float, radius: float) -> list[tuple[float, float, float]]:
    return [
        (x, y, z)
        for z, y in rounded_rect_profile(height, width, radius, corner_segments=10)
    ]


def _build_head_shell_mesh():
    ASSETS.mesh_dir.mkdir(parents=True, exist_ok=True)
    shell_geom = section_loft(
        [
            _yz_section(-0.085, 0.112, 0.100, 0.018),
            _yz_section(-0.020, 0.158, 0.140, 0.032),
            _yz_section(0.080, 0.192, 0.180, 0.052),
            _yz_section(0.155, 0.180, 0.168, 0.046),
        ]
    )
    return mesh_from_geometry(shell_geom, ASSETS.mesh_path("searchlight_head_shell.obj"))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_searchlight_tower", assets=ASSETS)

    concrete = model.material("concrete", rgba=(0.63, 0.64, 0.63, 1.0))
    painted_steel = model.material("painted_steel", rgba=(0.36, 0.40, 0.39, 1.0))
    dark_paint = model.material("dark_paint", rgba=(0.18, 0.19, 0.20, 1.0))
    galvanized = model.material("galvanized", rgba=(0.62, 0.65, 0.67, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.74, 0.86, 0.92, 0.35))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    head_shell_mesh = _build_head_shell_mesh()

    foundation = model.part("foundation")
    foundation.visual(
        Box((0.78, 0.78, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=concrete,
        name="base_pad",
    )
    foundation.visual(
        Box((0.34, 0.34, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.24)),
        material=dark_paint,
        name="pedestal_body",
    )
    foundation.visual(
        Box((0.30, 0.30, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.31)),
        material=galvanized,
        name="pedestal_cap",
    )
    foundation.visual(
        Box((0.22, 0.16, 0.28)),
        origin=Origin(xyz=(-0.27, 0.0, 0.32)),
        material=painted_steel,
        name="service_cabinet",
    )
    foundation.visual(
        Box((0.14, 0.004, 0.20)),
        origin=Origin(xyz=(-0.27, 0.082, 0.33)),
        material=dark_paint,
    )
    foundation.visual(
        Cylinder(radius=0.006, length=0.03),
        origin=Origin(xyz=(-0.23, 0.086, 0.33), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
    )
    _add_member(
        foundation,
        (-0.27, 0.0, 0.46),
        (-0.20, 0.0, 0.35),
        0.012,
        galvanized,
    )
    for sx in (-0.155, 0.155):
        for sy in (-0.155, 0.155):
            _add_vertical_bolt(
                foundation,
                xy=(sx, sy),
                base_z=0.32,
                shank_length=0.016,
                shank_radius=0.009,
                head_radius=0.015,
                head_length=0.012,
                material=galvanized,
            )
    foundation.inertial = Inertial.from_geometry(
        Box((0.78, 0.78, 0.48)),
        mass=120.0,
        origin=Origin(xyz=(0.0, 0.0, 0.24)),
    )

    mast = model.part("mast")
    mast.visual(
        Box((0.28, 0.28, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=galvanized,
        name="mast_base_flange",
    )
    mast.visual(
        Box((0.19, 0.19, 1.48)),
        origin=Origin(xyz=(0.0, 0.0, 0.764)),
        material=painted_steel,
        name="mast_shell",
    )
    mast.visual(
        Box((0.03, 0.045, 1.16)),
        origin=Origin(xyz=(0.0, 0.105, 0.74)),
        material=dark_paint,
        name="service_conduit",
    )
    for sy in (-0.034, 0.034):
        _add_member(mast, (-0.101, sy, 0.15), (-0.101, sy, 1.28), 0.006, galvanized)
    for i in range(15):
        z = 0.18 + i * 0.072
        _add_member(mast, (-0.101, -0.034, z), (-0.101, 0.034, z), 0.004, galvanized)
    mast.visual(
        Box((0.038, 0.12, 0.18)),
        origin=Origin(xyz=(0.076, 0.0, 0.114)),
        material=galvanized,
        name="gusset_east",
    )
    mast.visual(
        Box((0.038, 0.12, 0.18)),
        origin=Origin(xyz=(-0.076, 0.0, 0.114)),
        material=galvanized,
        name="gusset_west",
    )
    mast.visual(
        Box((0.12, 0.038, 0.18)),
        origin=Origin(xyz=(0.0, 0.076, 0.114)),
        material=galvanized,
        name="gusset_north",
    )
    mast.visual(
        Box((0.12, 0.038, 0.18)),
        origin=Origin(xyz=(0.0, -0.076, 0.114)),
        material=galvanized,
        name="gusset_south",
    )
    mast.visual(
        Box((0.03, 0.18, 0.20)),
        origin=Origin(xyz=(0.11, 0.0, 1.15)),
        material=galvanized,
        name="platform_receiver",
    )
    mast.visual(
        Box((0.26, 0.26, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 1.518)),
        material=galvanized,
        name="top_plate",
    )
    mast.visual(
        Cylinder(radius=0.082, length=0.042),
        origin=Origin(xyz=(0.0, 0.0, 1.539)),
        material=dark_paint,
        name="mast_hub",
    )
    for sx in (-0.085, 0.085):
        for sy in (-0.085, 0.085):
            _add_vertical_bolt(
                mast,
                xy=(sx, sy),
                base_z=0.024,
                shank_length=0.030,
                shank_radius=0.007,
                head_radius=0.012,
                head_length=0.008,
                material=galvanized,
            )
    mast.inertial = Inertial.from_geometry(
        Box((0.30, 0.30, 1.58)),
        mass=65.0,
        origin=Origin(xyz=(0.0, 0.0, 0.79)),
    )

    platform = model.part("service_platform")
    platform.visual(
        Box((0.06, 0.18, 0.08)),
        origin=Origin(xyz=(0.03, 0.0, 0.03)),
        material=galvanized,
        name="upper_bracket",
    )
    platform.visual(
        Box((0.08, 0.16, 0.07)),
        origin=Origin(xyz=(0.04, 0.0, -0.025)),
        material=galvanized,
        name="lower_bracket",
    )
    platform.visual(
        Box((0.035, 0.16, 0.05)),
        origin=Origin(xyz=(0.022, 0.0, -0.002)),
        material=galvanized,
        name="bracket_web",
    )
    _add_member(platform, (0.01, -0.11, -0.08), (0.22, -0.11, 0.055), 0.012, galvanized)
    _add_member(platform, (0.01, 0.11, -0.08), (0.22, 0.11, 0.055), 0.012, galvanized)
    platform.visual(
        Box((0.40, 0.30, 0.022)),
        origin=Origin(xyz=(0.24, 0.0, 0.06)),
        material=dark_paint,
        name="platform_deck",
    )
    platform.visual(
        Box((0.40, 0.008, 0.04)),
        origin=Origin(xyz=(0.24, 0.146, 0.091)),
        material=painted_steel,
    )
    platform.visual(
        Box((0.40, 0.008, 0.04)),
        origin=Origin(xyz=(0.24, -0.146, 0.091)),
        material=painted_steel,
    )
    platform.visual(
        Box((0.008, 0.30, 0.04)),
        origin=Origin(xyz=(0.436, 0.0, 0.091)),
        material=painted_steel,
    )
    post_points = [
        (0.06, -0.135),
        (0.06, 0.135),
        (0.25, -0.135),
        (0.25, 0.135),
        (0.43, -0.135),
        (0.43, 0.135),
    ]
    for x, y in post_points:
        platform.visual(
            Cylinder(radius=0.008, length=0.15),
            origin=Origin(xyz=(x, y, 0.146)),
            material=galvanized,
        )
    rail_loops = [
        ((0.06, -0.135, 0.221), (0.43, -0.135, 0.221)),
        ((0.06, 0.135, 0.221), (0.43, 0.135, 0.221)),
        ((0.43, -0.135, 0.221), (0.43, 0.135, 0.221)),
        ((0.06, -0.135, 0.151), (0.43, -0.135, 0.151)),
        ((0.06, 0.135, 0.151), (0.43, 0.135, 0.151)),
        ((0.43, -0.135, 0.151), (0.43, 0.135, 0.151)),
    ]
    for a, b in rail_loops:
        _add_member(platform, a, b, 0.0065, galvanized)
    platform.visual(
        Box((0.11, 0.12, 0.16)),
        origin=Origin(xyz=(0.13, -0.07, 0.151)),
        material=painted_steel,
        name="control_box",
    )
    platform.visual(
        Box((0.07, 0.004, 0.10)),
        origin=Origin(xyz=(0.13, -0.008, 0.155)),
        material=dark_paint,
    )
    platform.inertial = Inertial.from_geometry(
        Box((0.46, 0.32, 0.34)),
        mass=18.0,
        origin=Origin(xyz=(0.22, 0.0, 0.10)),
    )

    pan_stage = model.part("pan_stage")
    pan_stage.visual(
        Cylinder(radius=0.128, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=dark_paint,
        name="rotary_table",
    )
    pan_stage.visual(
        Cylinder(radius=0.072, length=0.094),
        origin=Origin(xyz=(0.0, 0.0, 0.066)),
        material=painted_steel,
        name="pan_hub",
    )
    pan_stage.visual(
        Box((0.11, 0.10, 0.09)),
        origin=Origin(xyz=(-0.10, 0.0, 0.075)),
        material=painted_steel,
        name="slew_drive",
    )
    pan_stage.visual(
        Cylinder(radius=0.035, length=0.09),
        origin=Origin(xyz=(-0.16, 0.0, 0.075), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_paint,
    )
    pan_stage.visual(
        Box((0.08, 0.14, 0.11)),
        origin=Origin(xyz=(0.01, 0.0, 0.125)),
        material=painted_steel,
        name="tilt_pedestal",
    )
    pan_stage.visual(
        Box((0.08, 0.034, 0.36)),
        origin=Origin(xyz=(0.085, -0.125, 0.29)),
        material=painted_steel,
        name="yoke_left_arm",
    )
    pan_stage.visual(
        Box((0.08, 0.034, 0.36)),
        origin=Origin(xyz=(0.085, 0.125, 0.29)),
        material=painted_steel,
        name="yoke_right_arm",
    )
    pan_stage.visual(
        Box((0.06, 0.22, 0.05)),
        origin=Origin(xyz=(0.02, 0.0, 0.495)),
        material=painted_steel,
        name="yoke_crossbar",
    )
    _add_member(pan_stage, (-0.02, -0.08, 0.16), (0.045, -0.125, 0.23), 0.012, galvanized)
    _add_member(pan_stage, (-0.02, 0.08, 0.16), (0.045, 0.125, 0.23), 0.012, galvanized)
    pan_stage.visual(
        Cylinder(radius=0.045, length=0.016),
        origin=Origin(xyz=(0.085, -0.117, 0.285), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_paint,
        name="left_bearing_collar",
    )
    pan_stage.visual(
        Cylinder(radius=0.045, length=0.016),
        origin=Origin(xyz=(0.085, 0.117, 0.285), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_paint,
        name="right_bearing_collar",
    )
    _add_fastener_ring(
        pan_stage,
        radius=0.10,
        z0=0.038,
        count=8,
        shank_length=0.008,
        shank_radius=0.0045,
        head_radius=0.008,
        head_length=0.005,
        material=galvanized,
    )
    pan_stage.inertial = Inertial.from_geometry(
        Box((0.34, 0.34, 0.48)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, 0.24)),
    )

    head = model.part("spotlight_head")
    head.visual(
        head_shell_mesh,
        origin=Origin(xyz=(0.04, 0.0, 0.03)),
        material=painted_steel,
        name="head_shell",
    )
    head.visual(
        Box((0.09, 0.14, 0.09)),
        origin=Origin(xyz=(-0.005, 0.0, 0.045)),
        material=dark_paint,
        name="rear_service_box",
    )
    head.visual(
        Box((0.09, 0.12, 0.018)),
        origin=Origin(xyz=(-0.005, 0.0, 0.098)),
        material=painted_steel,
    )
    for x in (-0.055, -0.035, -0.015, 0.005):
        head.visual(
            Box((0.008, 0.13, 0.07)),
            origin=Origin(xyz=(x, 0.0, 0.05)),
            material=galvanized,
        )
    head.visual(
        Box((0.08, 0.10, 0.04)),
        origin=Origin(xyz=(0.045, 0.0, 0.0)),
        material=dark_paint,
    )
    head.visual(
        Box((0.05, 0.028, 0.10)),
        origin=Origin(xyz=(0.03, -0.084, 0.02)),
        material=painted_steel,
    )
    head.visual(
        Box((0.05, 0.028, 0.10)),
        origin=Origin(xyz=(0.03, 0.084, 0.02)),
        material=painted_steel,
    )
    head.visual(
        Cylinder(radius=0.036, length=0.020),
        origin=Origin(xyz=(0.0, -0.099, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_paint,
        name="trunnion_left",
    )
    head.visual(
        Cylinder(radius=0.036, length=0.020),
        origin=Origin(xyz=(0.0, 0.099, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_paint,
        name="trunnion_right",
    )
    head.visual(
        Cylinder(radius=0.107, length=0.038),
        origin=Origin(xyz=(0.195, 0.0, 0.03), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_paint,
        name="front_bezel",
    )
    head.visual(
        Cylinder(radius=0.095, length=0.008),
        origin=Origin(xyz=(0.210, 0.0, 0.03), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="lens",
    )
    _add_member(head, (-0.055, -0.05, 0.09), (0.015, -0.05, 0.10), 0.007, rubber)
    _add_member(head, (-0.055, 0.05, 0.09), (0.015, 0.05, 0.10), 0.007, rubber)
    _add_member(head, (-0.055, -0.05, 0.09), (-0.055, 0.05, 0.09), 0.007, rubber)
    head.inertial = Inertial.from_geometry(
        Box((0.42, 0.22, 0.22)),
        mass=22.0,
        origin=Origin(xyz=(0.02, 0.0, 0.0)),
    )

    model.articulation(
        "foundation_to_mast",
        ArticulationType.FIXED,
        parent=foundation,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.32)),
    )
    model.articulation(
        "mast_to_platform",
        ArticulationType.FIXED,
        parent=mast,
        child=platform,
        origin=Origin(xyz=(0.125, 0.0, 1.12)),
    )
    model.articulation(
        "mast_to_pan_stage",
        ArticulationType.CONTINUOUS,
        parent=mast,
        child=pan_stage,
        origin=Origin(xyz=(0.0, 0.0, 1.56)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.65),
    )
    model.articulation(
        "pan_stage_to_head",
        ArticulationType.REVOLUTE,
        parent=pan_stage,
        child=head,
        origin=Origin(xyz=(0.085, 0.0, 0.285)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.8,
            lower=-0.45,
            upper=0.85,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    foundation = object_model.get_part("foundation")
    mast = object_model.get_part("mast")
    platform = object_model.get_part("service_platform")
    pan_stage = object_model.get_part("pan_stage")
    head = object_model.get_part("spotlight_head")

    mast_to_pan = object_model.get_articulation("mast_to_pan_stage")
    head_tilt = object_model.get_articulation("pan_stage_to_head")

    pedestal_cap = foundation.get_visual("pedestal_cap")
    mast_base_flange = mast.get_visual("mast_base_flange")
    mast_hub = mast.get_visual("mast_hub")
    platform_receiver = mast.get_visual("platform_receiver")
    upper_bracket = platform.get_visual("upper_bracket")
    platform_deck = platform.get_visual("platform_deck")
    rotary_table = pan_stage.get_visual("rotary_table")
    yoke_left_arm = pan_stage.get_visual("yoke_left_arm")
    yoke_right_arm = pan_stage.get_visual("yoke_right_arm")
    lens = head.get_visual("lens")
    trunnion_left = head.get_visual("trunnion_left")
    trunnion_right = head.get_visual("trunnion_right")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=20)

    ctx.expect_gap(
        mast,
        foundation,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=mast_base_flange,
        negative_elem=pedestal_cap,
    )
    ctx.expect_gap(
        platform,
        mast,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=upper_bracket,
        negative_elem=platform_receiver,
    )
    ctx.expect_overlap(
        platform,
        mast,
        axes="yz",
        min_overlap=0.08,
        elem_a=upper_bracket,
        elem_b=platform_receiver,
    )
    ctx.expect_gap(
        pan_stage,
        mast,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=rotary_table,
        negative_elem=mast_hub,
    )
    ctx.expect_origin_distance(pan_stage, mast, axes="xy", max_dist=0.005)
    ctx.expect_contact(head, pan_stage, elem_a=trunnion_left, elem_b=yoke_left_arm)
    ctx.expect_contact(head, pan_stage, elem_a=trunnion_right, elem_b=yoke_right_arm)
    ctx.expect_origin_gap(platform, foundation, axis="z", min_gap=1.0)
    ctx.expect_gap(
        pan_stage,
        platform,
        axis="z",
        min_gap=0.30,
        positive_elem=rotary_table,
        negative_elem=platform_deck,
    )

    rest_head_pos = ctx.part_world_position(head)
    if rest_head_pos is not None:
        with ctx.pose({mast_to_pan: math.pi / 2.0}):
            pan_head_pos = ctx.part_world_position(head)
            ctx.check(
                "pan_stage_rotates_head_around_mast",
                pan_head_pos is not None
                and abs(pan_head_pos[0]) < 0.025
                and pan_head_pos[1] > 0.04,
                details=f"rest={rest_head_pos}, pan_90={pan_head_pos}",
            )
    else:
        ctx.fail("head_world_position_available", "Could not measure spotlight head world position.")

    def _aabb_center(aabb):
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    rest_lens_aabb = ctx.part_element_world_aabb(head, elem="lens")
    if rest_lens_aabb is not None:
        rest_lens_center = _aabb_center(rest_lens_aabb)
        ctx.check(
            "searchlight_projects_forward",
            rest_lens_center[0] > 0.22,
            details=f"lens_center={rest_lens_center}",
        )
        with ctx.pose({head_tilt: 0.75}):
            raised_lens_aabb = ctx.part_element_world_aabb(head, elem="lens")
            if raised_lens_aabb is not None:
                raised_lens_center = _aabb_center(raised_lens_aabb)
                ctx.check(
                    "tilt_stage_raises_beam",
                    raised_lens_center[2] > rest_lens_center[2] + 0.06,
                    details=f"rest={rest_lens_center}, raised={raised_lens_center}",
                )
            else:
                ctx.fail("tilt_pose_lens_available", "Could not measure raised lens position.")
            ctx.expect_contact(head, pan_stage, elem_a=trunnion_left, elem_b=yoke_left_arm)
            ctx.expect_contact(head, pan_stage, elem_a=trunnion_right, elem_b=yoke_right_arm)
            ctx.expect_gap(
                head,
                pan_stage,
                axis="z",
                min_gap=0.07,
                positive_elem=lens,
                negative_elem=rotary_table,
            )
        with ctx.pose({head_tilt: -0.40}):
            lowered_lens_aabb = ctx.part_element_world_aabb(head, elem="lens")
            if lowered_lens_aabb is not None:
                lowered_lens_center = _aabb_center(lowered_lens_aabb)
                ctx.check(
                    "tilt_stage_allows_down_aim",
                    lowered_lens_center[2] < rest_lens_center[2] - 0.03,
                    details=f"rest={rest_lens_center}, lowered={lowered_lens_center}",
                )
            else:
                ctx.fail("down_tilt_lens_available", "Could not measure lowered lens position.")
            ctx.expect_gap(
                head,
                pan_stage,
                axis="z",
                min_gap=0.045,
                positive_elem=lens,
                negative_elem=rotary_table,
            )
    else:
        ctx.fail("rest_lens_aabb_available", "Could not measure lens bounds in rest pose.")

    foundation_aabb = ctx.part_world_aabb(foundation)
    head_aabb = ctx.part_world_aabb(head)
    if foundation_aabb is not None and head_aabb is not None:
        total_height = head_aabb[1][2] - foundation_aabb[0][2]
        ctx.check(
            "tower_reads_tall",
            total_height > 1.95,
            details=f"total_height={total_height:.3f}",
        )
    else:
        ctx.fail("overall_height_available", "Could not measure total object height.")

    deck_aabb = ctx.part_element_world_aabb(platform, elem="platform_deck")
    if deck_aabb is not None:
        deck_center = _aabb_center(deck_aabb)
        ctx.check(
            "service_platform_projects_from_mast",
            deck_center[0] > 0.25,
            details=f"deck_center={deck_center}",
        )
    else:
        ctx.fail("platform_deck_available", "Could not measure platform deck bounds.")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
