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
    ExtrudeWithHolesGeometry,
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


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _circle_profile(radius: float, segments: int = 64) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * index / segments),
            radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]


def _gear_outline(
    outer_radius: float,
    tooth_depth: float,
    tooth_count: int,
) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    valley_radius = outer_radius - tooth_depth
    for index in range(tooth_count * 2):
        angle = math.pi * index / tooth_count
        radius = outer_radius if index % 2 == 0 else valley_radius
        points.append((radius * math.cos(angle), radius * math.sin(angle)))
    return points


def _ring_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    segments: int = 72,
):
    geom = ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius, segments=segments),
        [_circle_profile(inner_radius, segments=max(segments // 2, 24))],
        height=length,
        center=True,
    ).rotate_x(math.pi / 2.0)
    return _save_mesh(name, geom)


def _rounded_section(
    center_x: float,
    center_y: float,
    z: float,
    width: float,
    thickness: float,
) -> list[tuple[float, float, float]]:
    profile = rounded_rect_profile(
        width,
        thickness,
        radius=min(width, thickness) * 0.28,
        corner_segments=6,
    )
    return [(center_x + x, center_y + y, z) for x, y in profile]


def _arm_mesh(
    name: str,
    *,
    y_center: float,
    tip_x: float,
    tip_z: float,
):
    sections = [
        _rounded_section(0.000, y_center, 0.000, 0.036, 0.022),
        _rounded_section(tip_x * 0.20, y_center, tip_z * 0.28, 0.032, 0.019),
        _rounded_section(tip_x * 0.55, y_center, tip_z * 0.62, 0.028, 0.017),
        _rounded_section(tip_x * 0.90, y_center, tip_z * 0.92, 0.024, 0.015),
    ]
    return _save_mesh(name, section_loft(sections))


def _midpoint(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_segment(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_spider_arm(
    part,
    *,
    name: str,
    inner_radius: float,
    outer_radius: float,
    angle_deg: float,
    y_pos: float,
    width: float,
    thickness: float,
    material,
) -> None:
    angle = math.radians(angle_deg)
    a = (inner_radius * math.cos(angle), y_pos, inner_radius * math.sin(angle))
    b = (outer_radius * math.cos(angle), y_pos, outer_radius * math.sin(angle))
    part.visual(
        Box((width, thickness, _distance(a, b))),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_segment(a, b)),
        material=material,
        name=name,
    )


def _build_pedal(
    model: ArticulatedObject,
    name: str,
    *,
    side_sign: float,
    pedal_material,
    tread_material,
):
    pedal = model.part(name)
    pedal.visual(
        Box((0.072, 0.036, 0.011)),
        origin=Origin(xyz=(0.000, side_sign * 0.022, 0.000)),
        material=pedal_material,
        name="body",
    )
    pedal.visual(
        Box((0.052, 0.018, 0.003)),
        origin=Origin(xyz=(0.000, side_sign * 0.022, 0.007)),
        material=tread_material,
        name="upper_tread",
    )
    pedal.visual(
        Box((0.052, 0.018, 0.003)),
        origin=Origin(xyz=(0.000, side_sign * 0.022, -0.007)),
        material=tread_material,
        name="lower_tread",
    )
    pedal.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(
            xyz=(0.000, side_sign * 0.006, 0.000),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=pedal_material,
        name="hinge_barrel",
    )
    pedal.visual(
        Cylinder(radius=0.0055, length=0.014),
        origin=Origin(
            xyz=(0.027, side_sign * 0.022, 0.000),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=pedal_material,
        name="front_endcap",
    )
    pedal.visual(
        Cylinder(radius=0.0055, length=0.014),
        origin=Origin(
            xyz=(-0.027, side_sign * 0.022, 0.000),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=pedal_material,
        name="rear_endcap",
    )
    pedal.inertial = Inertial.from_geometry(
        Box((0.072, 0.044, 0.022)),
        mass=0.18,
        origin=Origin(xyz=(0.000, side_sign * 0.020, 0.000)),
    )
    return pedal


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_bike_compact_crankset", assets=ASSETS)

    arm_metal = model.material("arm_metal", rgba=(0.63, 0.65, 0.68, 1.0))
    spindle_steel = model.material("spindle_steel", rgba=(0.45, 0.47, 0.50, 1.0))
    bb_black = model.material("bb_black", rgba=(0.12, 0.12, 0.13, 1.0))
    chainring_black = model.material("chainring_black", rgba=(0.08, 0.08, 0.09, 1.0))
    pedal_body = model.material("pedal_body", rgba=(0.24, 0.25, 0.27, 1.0))
    tread_rubber = model.material("tread_rubber", rgba=(0.07, 0.07, 0.07, 1.0))

    bb_shell_mesh = _ring_mesh(
        "compact_bb_shell.obj",
        outer_radius=0.021,
        inner_radius=0.0125,
        length=0.066,
    )
    bb_cup_mesh = _ring_mesh(
        "compact_bb_cup.obj",
        outer_radius=0.026,
        inner_radius=0.0125,
        length=0.010,
    )
    bb_seal_mesh = _ring_mesh(
        "compact_bb_seal.obj",
        outer_radius=0.0235,
        inner_radius=0.0105,
        length=0.003,
    )

    chainring_mesh = _save_mesh(
        "compact_chainring.obj",
        ExtrudeWithHolesGeometry(
            _gear_outline(outer_radius=0.089, tooth_depth=0.006, tooth_count=34),
            [
                _circle_profile(0.024, segments=36),
                rounded_rect_profile(0.020, 0.034, radius=0.004, corner_segments=5),
                [
                    (x, y)
                    for x, y in rounded_rect_profile(
                        0.020,
                        0.034,
                        radius=0.004,
                        corner_segments=5,
                    )
                ],
                [
                    (x * math.cos(math.pi / 2.0) - y * math.sin(math.pi / 2.0),
                     x * math.sin(math.pi / 2.0) + y * math.cos(math.pi / 2.0))
                    for x, y in rounded_rect_profile(
                        0.020,
                        0.034,
                        radius=0.004,
                        corner_segments=5,
                    )
                ],
                [
                    (x * math.cos(math.pi / 4.0) - y * math.sin(math.pi / 4.0),
                     x * math.sin(math.pi / 4.0) + y * math.cos(math.pi / 4.0))
                    for x, y in rounded_rect_profile(
                        0.018,
                        0.030,
                        radius=0.004,
                        corner_segments=5,
                    )
                ],
            ],
            height=0.004,
            center=True,
        ).rotate_x(math.pi / 2.0),
    )

    right_arm_mesh = _arm_mesh(
        "right_compact_arm.obj",
        y_center=0.056,
        tip_x=0.016,
        tip_z=-0.148,
    )
    left_arm_mesh = _arm_mesh(
        "left_compact_arm.obj",
        y_center=-0.056,
        tip_x=-0.016,
        tip_z=0.148,
    )

    bottom_bracket = model.part("bottom_bracket")
    bottom_bracket.visual(bb_shell_mesh, material=bb_black, name="shell_tube")
    bottom_bracket.visual(
        bb_cup_mesh,
        origin=Origin(xyz=(0.000, 0.038, 0.000)),
        material=bb_black,
        name="right_cup",
    )
    bottom_bracket.visual(
        bb_cup_mesh,
        origin=Origin(xyz=(0.000, -0.038, 0.000)),
        material=bb_black,
        name="left_cup",
    )
    bottom_bracket.visual(
        bb_seal_mesh,
        origin=Origin(xyz=(0.000, 0.0445, 0.000)),
        material=spindle_steel,
        name="right_seal",
    )
    bottom_bracket.visual(
        bb_seal_mesh,
        origin=Origin(xyz=(0.000, -0.0445, 0.000)),
        material=spindle_steel,
        name="left_seal",
    )
    bottom_bracket.inertial = Inertial.from_geometry(
        Box((0.054, 0.094, 0.054)),
        mass=0.60,
        origin=Origin(),
    )

    crankset = model.part("crankset")
    crankset.visual(
        Cylinder(radius=0.010, length=0.118),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=spindle_steel,
        name="spindle",
    )
    crankset.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.000, 0.056, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=arm_metal,
        name="right_boss",
    )
    crankset.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.000, -0.056, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=arm_metal,
        name="left_boss",
    )
    crankset.visual(right_arm_mesh, material=arm_metal, name="right_arm")
    crankset.visual(left_arm_mesh, material=arm_metal, name="left_arm")
    crankset.visual(
        Cylinder(radius=0.013, length=0.014),
        origin=Origin(xyz=(0.016, 0.056, -0.148), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=arm_metal,
        name="right_pedal_eye",
    )
    crankset.visual(
        Cylinder(radius=0.013, length=0.014),
        origin=Origin(xyz=(-0.016, -0.056, 0.148), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=arm_metal,
        name="left_pedal_eye",
    )
    crankset.visual(
        Box((0.018, 0.011, 0.014)),
        origin=Origin(xyz=(0.016, 0.067, -0.148)),
        material=arm_metal,
        name="right_pedal_mount",
    )
    crankset.visual(
        Box((0.018, 0.011, 0.014)),
        origin=Origin(xyz=(-0.016, -0.067, 0.148)),
        material=arm_metal,
        name="left_pedal_mount",
    )
    crankset.visual(
        Cylinder(radius=0.004, length=0.020),
        origin=Origin(
            xyz=(0.016, 0.0725, -0.148),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=spindle_steel,
        name="right_hinge_spring",
    )
    crankset.visual(
        Cylinder(radius=0.004, length=0.020),
        origin=Origin(
            xyz=(-0.016, -0.0725, 0.148),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=spindle_steel,
        name="left_hinge_spring",
    )
    crankset.visual(
        Cylinder(radius=0.024, length=0.004),
        origin=Origin(xyz=(0.000, 0.041, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=arm_metal,
        name="chainring_carrier",
    )
    crankset.visual(
        chainring_mesh,
        origin=Origin(xyz=(0.000, 0.040, 0.000)),
        material=chainring_black,
        name="chainring",
    )
    for index, angle_deg in enumerate((25.0, 115.0, 205.0, 295.0)):
        _add_spider_arm(
            crankset,
            name=f"spider_arm_{index}",
            inner_radius=0.024,
            outer_radius=0.066,
            angle_deg=angle_deg,
            y_pos=0.041,
            width=0.010,
            thickness=0.006,
            material=arm_metal,
        )
    crankset.visual(
        Cylinder(radius=0.014, length=0.004),
        origin=Origin(xyz=(0.000, -0.048, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=spindle_steel,
        name="left_cap",
    )
    crankset.inertial = Inertial.from_geometry(
        Box((0.220, 0.126, 0.326)),
        mass=1.55,
        origin=Origin(),
    )

    right_pedal = _build_pedal(
        model,
        "right_pedal",
        side_sign=1.0,
        pedal_material=pedal_body,
        tread_material=tread_rubber,
    )
    left_pedal = _build_pedal(
        model,
        "left_pedal",
        side_sign=-1.0,
        pedal_material=pedal_body,
        tread_material=tread_rubber,
    )

    model.articulation(
        "bb_to_crankset",
        ArticulationType.CONTINUOUS,
        parent=bottom_bracket,
        child=crankset,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=12.0),
    )
    model.articulation(
        "right_pedal_fold",
        ArticulationType.REVOLUTE,
        parent=crankset,
        child=right_pedal,
        origin=Origin(xyz=(0.016, 0.0725, -0.148)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=0.0, upper=1.58),
    )
    model.articulation(
        "left_pedal_fold",
        ArticulationType.REVOLUTE,
        parent=crankset,
        child=left_pedal,
        origin=Origin(xyz=(-0.016, -0.0725, 0.148)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=0.0, upper=1.58),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    bottom_bracket = object_model.get_part("bottom_bracket")
    crankset = object_model.get_part("crankset")
    right_pedal = object_model.get_part("right_pedal")
    left_pedal = object_model.get_part("left_pedal")

    crank_spin = object_model.get_articulation("bb_to_crankset")
    right_fold = object_model.get_articulation("right_pedal_fold")
    left_fold = object_model.get_articulation("left_pedal_fold")

    shell_tube = bottom_bracket.get_visual("shell_tube")
    right_cup = bottom_bracket.get_visual("right_cup")
    left_cup = bottom_bracket.get_visual("left_cup")
    chainring = crankset.get_visual("chainring")
    spindle = crankset.get_visual("spindle")
    right_boss = crankset.get_visual("right_boss")
    left_boss = crankset.get_visual("left_boss")
    right_arm = crankset.get_visual("right_arm")
    left_arm = crankset.get_visual("left_arm")
    right_mount = crankset.get_visual("right_pedal_mount")
    left_mount = crankset.get_visual("left_pedal_mount")
    right_spring = crankset.get_visual("right_hinge_spring")
    left_spring = crankset.get_visual("left_hinge_spring")
    right_body = right_pedal.get_visual("body")
    left_body = left_pedal.get_visual("body")
    right_hinge_barrel = right_pedal.get_visual("hinge_barrel")
    left_hinge_barrel = left_pedal.get_visual("hinge_barrel")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    ctx.allow_overlap(
        right_pedal,
        crankset,
        reason="folding pedal knuckle nests around the crank-arm hinge lug and torsion spring hardware",
    )
    ctx.allow_overlap(
        left_pedal,
        crankset,
        reason="folding pedal knuckle nests around the crank-arm hinge lug and torsion spring hardware",
    )
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)
    ctx.expect_overlap(crankset, bottom_bracket, axes="xz", elem_a=spindle, elem_b=shell_tube, min_overlap=0.019)
    ctx.expect_gap(
        crankset,
        bottom_bracket,
        axis="y",
        positive_elem=right_boss,
        negative_elem=right_cup,
        max_gap=0.009,
        max_penetration=0.0,
    )
    ctx.expect_gap(
        bottom_bracket,
        crankset,
        axis="y",
        positive_elem=left_cup,
        negative_elem=left_boss,
        max_gap=0.009,
        max_penetration=0.0,
    )
    ctx.expect_gap(
        crankset,
        bottom_bracket,
        axis="y",
        positive_elem=chainring,
        negative_elem=shell_tube,
        min_gap=0.004,
    )

    ctx.expect_contact(right_pedal, crankset, elem_a=right_hinge_barrel, elem_b=right_mount)
    ctx.expect_contact(left_pedal, crankset, elem_a=left_hinge_barrel, elem_b=left_mount)
    ctx.expect_contact(right_pedal, crankset, elem_a=right_hinge_barrel, elem_b=right_spring)
    ctx.expect_contact(left_pedal, crankset, elem_a=left_hinge_barrel, elem_b=left_spring)
    ctx.expect_gap(
        right_pedal,
        crankset,
        axis="y",
        positive_elem=right_body,
        negative_elem=right_arm,
        min_gap=0.009,
    )
    ctx.expect_gap(
        crankset,
        left_pedal,
        axis="y",
        positive_elem=left_arm,
        negative_elem=left_body,
        min_gap=0.009,
    )

    with ctx.pose({right_fold: 1.56, left_fold: 1.56}):
        ctx.expect_overlap(right_pedal, crankset, axes="xz", elem_a=right_body, elem_b=right_arm, min_overlap=0.020)
        ctx.expect_overlap(left_pedal, crankset, axes="xz", elem_a=left_body, elem_b=left_arm, min_overlap=0.020)
        ctx.expect_gap(
            right_pedal,
            crankset,
            axis="y",
            positive_elem=right_body,
            negative_elem=right_arm,
            max_gap=0.0015,
            max_penetration=0.0005,
        )
        ctx.expect_gap(
            crankset,
            left_pedal,
            axis="y",
            positive_elem=left_arm,
            negative_elem=left_body,
            max_gap=0.0015,
            max_penetration=0.0005,
        )

    with ctx.pose({crank_spin: math.pi / 2.0}):
        ctx.expect_overlap(crankset, bottom_bracket, axes="xz", elem_a=spindle, elem_b=shell_tube, min_overlap=0.019)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
