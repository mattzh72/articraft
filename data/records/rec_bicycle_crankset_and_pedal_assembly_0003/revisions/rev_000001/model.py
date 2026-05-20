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
)

ASSETS = AssetContext.from_script(__file__)


def _radial_point(radius: float, angle: float, y: float) -> tuple[float, float, float]:
    return (radius * math.cos(angle), y, radius * math.sin(angle))


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _circle_profile(
    radius: float,
    *,
    segments: int = 40,
    center: tuple[float, float] = (0.0, 0.0),
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos((2.0 * math.pi * index) / segments),
            cy + radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _gear_profile(tip_radius: float, root_radius: float, *, teeth: int) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    tooth_step = (2.0 * math.pi) / teeth
    for tooth_index in range(teeth):
        base_angle = tooth_index * tooth_step
        points.append((root_radius * math.cos(base_angle), root_radius * math.sin(base_angle)))
        tip_lead = base_angle + 0.22 * tooth_step
        tip_trail = base_angle + 0.78 * tooth_step
        points.append((tip_radius * math.cos(tip_lead), tip_radius * math.sin(tip_lead)))
        points.append((tip_radius * math.cos(tip_trail), tip_radius * math.sin(tip_trail)))
    return points


def _ring_plate_mesh(
    *,
    mesh_name: str,
    outer_radius: float,
    root_radius: float,
    teeth: int,
    thickness: float,
    center_hole_radius: float,
    window_radius: float | None = None,
    window_ring_radius: float | None = None,
    window_phase: float = 0.0,
    bolt_circle_radius: float | None = None,
    bolt_radius: float | None = None,
) :
    hole_profiles = [_circle_profile(center_hole_radius, segments=32)]
    if window_radius is not None and window_ring_radius is not None:
        for index in range(5):
            angle = window_phase + (2.0 * math.pi * index / 5.0)
            hole_profiles.append(
                _circle_profile(
                    window_radius,
                    segments=18,
                    center=(window_ring_radius * math.cos(angle), window_ring_radius * math.sin(angle)),
                )
            )
    if bolt_circle_radius is not None and bolt_radius is not None:
        for index in range(5):
            angle = 2.0 * math.pi * index / 5.0
            hole_profiles.append(
                _circle_profile(
                    bolt_radius,
                    segments=18,
                    center=(bolt_circle_radius * math.cos(angle), bolt_circle_radius * math.sin(angle)),
                )
            )
    ring_geom = ExtrudeWithHolesGeometry(
        _gear_profile(outer_radius, root_radius, teeth=teeth),
        hole_profiles,
        height=thickness,
        center=True,
    ).rotate_x(math.pi / 2.0)
    return _save_mesh(mesh_name, ring_geom)


def _add_spider(part, *, y: float, material) -> None:
    for index in range(5):
        angle = 2.0 * math.pi * index / 5.0
        x, _, z = _radial_point(0.062, angle, y)
        part.visual(
            Box((0.070, 0.006, 0.016)),
            origin=Origin(xyz=(x, y, z), rpy=(0.0, -angle, 0.0)),
            material=material,
            name=f"spider_arm_{index}",
        )


def _add_chainring(part, *, radius: float, y: float, material, name: str) -> None:
    ring_mesh = {
        "outer_ring": _ring_plate_mesh(
            mesh_name="outer_chainring.obj",
            outer_radius=radius,
            root_radius=radius - 0.0045,
            teeth=25,
            thickness=0.0024,
            center_hole_radius=0.028,
            window_radius=0.016,
            window_ring_radius=0.068,
            window_phase=math.pi / 5.0,
            bolt_circle_radius=0.046,
            bolt_radius=0.0042,
        ),
        "middle_ring": _ring_plate_mesh(
            mesh_name="middle_chainring.obj",
            outer_radius=radius,
            root_radius=radius - 0.0040,
            teeth=21,
            thickness=0.0024,
            center_hole_radius=0.030,
            window_radius=0.013,
            window_ring_radius=0.057,
            window_phase=math.pi / 5.0,
            bolt_circle_radius=0.046,
            bolt_radius=0.0042,
        ),
        "inner_ring": _ring_plate_mesh(
            mesh_name="inner_chainring.obj",
            outer_radius=radius,
            root_radius=radius - 0.0034,
            teeth=17,
            thickness=0.0022,
            center_hole_radius=0.034,
            window_radius=0.0085,
            window_ring_radius=0.044,
            window_phase=math.pi / 5.0,
        ),
    }[name]
    part.visual(
        ring_mesh,
        origin=Origin(xyz=(0.0, y, 0.0)),
        material=material,
        name=name,
    )


def _add_crank_arm(part, *, side: float, y: float, material, prefix: str) -> None:
    part.visual(
        Box((0.050, 0.030, 0.026)),
        origin=Origin(xyz=(side * 0.028, y, 0.0)),
        material=material,
        name=f"{prefix}_root",
    )
    part.visual(
        Box((0.090, 0.018, 0.014)),
        origin=Origin(xyz=(side * 0.092, y, 0.0)),
        material=material,
        name=prefix,
    )
    part.visual(
        Box((0.050, 0.024, 0.020)),
        origin=Origin(xyz=(side * 0.158, y, 0.0)),
        material=material,
        name=f"{prefix}_end",
    )


def _add_pedal(part, *, side_sign: float, body_material, metal_material) -> None:
    axle_center_y = side_sign * 0.017
    body_center_y = side_sign * 0.030
    rail_offset_y = side_sign * 0.0115
    part.visual(
        Cylinder(radius=0.0045, length=0.034),
        origin=Origin(xyz=(0.0, axle_center_y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_material,
        name="axle",
    )
    part.visual(
        Box((0.018, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, side_sign * 0.010, 0.0)),
        material=metal_material,
        name="axle_block",
    )
    part.visual(
        Box((0.078, 0.028, 0.004)),
        origin=Origin(xyz=(0.0, body_center_y, 0.0)),
        material=body_material,
        name="platform_body",
    )
    part.visual(
        Box((0.042, 0.016, 0.002)),
        origin=Origin(xyz=(0.0, body_center_y, -0.001)),
        material=metal_material,
        name="recess_floor",
    )
    part.visual(
        Box((0.078, 0.005, 0.004)),
        origin=Origin(xyz=(0.0, body_center_y - rail_offset_y, 0.004)),
        material=body_material,
        name="left_top_rail",
    )
    part.visual(
        Box((0.078, 0.005, 0.004)),
        origin=Origin(xyz=(0.0, body_center_y + rail_offset_y, 0.004)),
        material=body_material,
        name="right_top_rail",
    )
    part.visual(
        Box((0.012, 0.020, 0.004)),
        origin=Origin(xyz=(-0.031, body_center_y, 0.004)),
        material=body_material,
        name="rear_binding",
    )
    part.visual(
        Box((0.012, 0.020, 0.004)),
        origin=Origin(xyz=(0.031, body_center_y, 0.004)),
        material=body_material,
        name="front_binding",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="road_bike_triple_crankset", assets=ASSETS)

    alloy = model.material("forged_alloy", rgba=(0.78, 0.80, 0.83, 1.0))
    chainring_steel = model.material("chainring_steel", rgba=(0.62, 0.65, 0.69, 1.0))
    black_anodized = model.material("black_anodized", rgba=(0.14, 0.15, 0.16, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.27, 0.29, 0.31, 1.0))
    pedal_body = model.material("pedal_body", rgba=(0.10, 0.11, 0.12, 1.0))

    bearing_set = model.part("bearing_set")
    bearing_set.visual(
        Cylinder(radius=0.020, length=0.036),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_anodized,
        name="shell_tube",
    )
    bearing_set.visual(
        Cylinder(radius=0.028, length=0.016),
        origin=Origin(xyz=(0.0, 0.026, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_cup",
    )
    bearing_set.visual(
        Cylinder(radius=0.028, length=0.016),
        origin=Origin(xyz=(0.0, -0.026, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_cup",
    )
    bearing_set.visual(
        Cylinder(radius=0.024, length=0.008),
        origin=Origin(xyz=(0.0, 0.018, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_anodized,
        name="right_seal",
    )
    bearing_set.visual(
        Cylinder(radius=0.024, length=0.008),
        origin=Origin(xyz=(0.0, -0.018, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_anodized,
        name="left_seal",
    )
    bearing_set.inertial = Inertial.from_geometry(Box((0.080, 0.080, 0.060)), mass=0.45, origin=Origin())

    drive_side = model.part("drive_side_assembly")
    drive_side.visual(
        Cylinder(radius=0.0135, length=0.118),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="spindle_tube",
    )
    drive_side.visual(
        Cylinder(radius=0.030, length=0.008),
        origin=Origin(xyz=(0.0, 0.024, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=alloy,
        name="spider_hub",
    )
    drive_side.visual(
        Cylinder(radius=0.022, length=0.020),
        origin=Origin(xyz=(0.0, 0.047, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=alloy,
        name="right_boss",
    )
    _add_spider(drive_side, y=0.024, material=alloy)
    _add_chainring(drive_side, radius=0.111, y=0.029, material=chainring_steel, name="outer_ring")
    _add_chainring(drive_side, radius=0.094, y=0.024, material=chainring_steel, name="middle_ring")
    _add_chainring(drive_side, radius=0.075, y=0.019, material=chainring_steel, name="inner_ring")
    for index in range(5):
        angle = 2.0 * math.pi * index / 5.0
        x, _, z = _radial_point(0.046, angle, 0.024)
        drive_side.visual(
            Cylinder(radius=0.0038, length=0.014),
            origin=Origin(xyz=(x, 0.024, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"chainring_bolt_{index}",
        )
    _add_crank_arm(drive_side, side=1.0, y=0.047, material=alloy, prefix="right_arm")
    drive_side.visual(
        Cylinder(radius=0.015, length=0.016),
        origin=Origin(xyz=(0.175, 0.047, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=alloy,
        name="right_pedal_eye",
    )
    drive_side.inertial = Inertial.from_geometry(
        Box((0.360, 0.120, 0.240)),
        mass=1.75,
        origin=Origin(xyz=(0.030, 0.020, 0.0)),
    )

    left_crank = model.part("left_crank")
    left_crank.visual(
        Cylinder(radius=0.0215, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=alloy,
        name="clamp_boss",
    )
    _add_crank_arm(left_crank, side=-1.0, y=0.003, material=alloy, prefix="left_arm")
    left_crank.visual(
        Cylinder(radius=0.015, length=0.016),
        origin=Origin(xyz=(-0.175, 0.003, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=alloy,
        name="left_pedal_eye",
    )
    left_crank.visual(
        Cylinder(radius=0.011, length=0.006),
        origin=Origin(xyz=(0.0, -0.015, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_anodized,
        name="preload_cap",
    )
    left_crank.visual(
        Box((0.007, 0.004, 0.018)),
        origin=Origin(xyz=(0.010, -0.006, 0.010)),
        material=dark_steel,
        name="upper_pinch_bolt",
    )
    left_crank.visual(
        Box((0.007, 0.004, 0.018)),
        origin=Origin(xyz=(0.010, -0.006, -0.010)),
        material=dark_steel,
        name="lower_pinch_bolt",
    )
    left_crank.visual(
        Box((0.002, 0.012, 0.030)),
        origin=Origin(xyz=(-0.010, -0.011, 0.0)),
        material=black_anodized,
        name="clamp_slot",
    )
    left_crank.inertial = Inertial.from_geometry(
        Box((0.360, 0.120, 0.110)),
        mass=0.62,
        origin=Origin(xyz=(-0.070, 0.015, 0.0)),
    )

    right_pedal = model.part("right_pedal")
    _add_pedal(right_pedal, side_sign=1.0, body_material=pedal_body, metal_material=dark_steel)
    right_pedal.inertial = Inertial.from_geometry(
        Box((0.090, 0.048, 0.016)),
        mass=0.17,
        origin=Origin(xyz=(0.0, 0.020, 0.002)),
    )

    left_pedal = model.part("left_pedal")
    _add_pedal(left_pedal, side_sign=-1.0, body_material=pedal_body, metal_material=dark_steel)
    left_pedal.inertial = Inertial.from_geometry(
        Box((0.090, 0.048, 0.016)),
        mass=0.17,
        origin=Origin(xyz=(0.0, -0.020, 0.002)),
    )

    model.articulation(
        "crank_rotation",
        ArticulationType.CONTINUOUS,
        parent=bearing_set,
        child=drive_side,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=16.0),
    )
    model.articulation(
        "left_crank_mount",
        ArticulationType.FIXED,
        parent=drive_side,
        child=left_crank,
        origin=Origin(xyz=(0.0, -0.050, 0.0)),
    )
    model.articulation(
        "right_pedal_spin",
        ArticulationType.CONTINUOUS,
        parent=drive_side,
        child=right_pedal,
        origin=Origin(xyz=(0.175, 0.055, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=25.0),
    )
    model.articulation(
        "left_pedal_spin",
        ArticulationType.CONTINUOUS,
        parent=left_crank,
        child=left_pedal,
        origin=Origin(xyz=(-0.175, -0.005, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=25.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    bearing_set = object_model.get_part("bearing_set")
    drive_side = object_model.get_part("drive_side_assembly")
    left_crank = object_model.get_part("left_crank")
    right_pedal = object_model.get_part("right_pedal")
    left_pedal = object_model.get_part("left_pedal")

    crank_rotation = object_model.get_articulation("crank_rotation")
    right_pedal_spin = object_model.get_articulation("right_pedal_spin")
    left_pedal_spin = object_model.get_articulation("left_pedal_spin")

    shell_tube = bearing_set.get_visual("shell_tube")
    right_cup = bearing_set.get_visual("right_cup")
    left_cup = bearing_set.get_visual("left_cup")
    outer_ring = drive_side.get_visual("outer_ring")
    middle_ring = drive_side.get_visual("middle_ring")
    inner_ring = drive_side.get_visual("inner_ring")
    spindle_tube = drive_side.get_visual("spindle_tube")
    left_boss = left_crank.get_visual("clamp_boss")
    preload_cap = left_crank.get_visual("preload_cap")
    right_arm = drive_side.get_visual("right_arm")
    left_arm = left_crank.get_visual("left_arm")
    right_platform = right_pedal.get_visual("platform_body")
    right_recess = right_pedal.get_visual("recess_floor")
    right_rail = right_pedal.get_visual("left_top_rail")
    left_platform = left_pedal.get_visual("platform_body")
    left_recess = left_pedal.get_visual("recess_floor")
    left_rail = left_pedal.get_visual("left_top_rail")
    right_axle = right_pedal.get_visual("axle")
    left_axle = left_pedal.get_visual("axle")
    right_eye = drive_side.get_visual("right_pedal_eye")
    left_eye = left_crank.get_visual("left_pedal_eye")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_disconnected()
    ctx.allow_overlap(drive_side, bearing_set, reason="hollowtech spindle runs through the outboard-bearing assembly")
    ctx.check_articulation_overlaps(max_pose_samples=128)
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_origin_distance(drive_side, bearing_set, axes="xz", max_dist=0.001)
    ctx.expect_overlap(bearing_set, bearing_set, axes="xz", elem_a=right_cup, elem_b=shell_tube, min_overlap=0.04)
    ctx.expect_overlap(bearing_set, bearing_set, axes="xz", elem_a=left_cup, elem_b=shell_tube, min_overlap=0.04)
    ctx.expect_gap(
        bearing_set,
        bearing_set,
        axis="y",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem=right_cup,
        negative_elem=shell_tube,
    )
    ctx.expect_gap(
        bearing_set,
        bearing_set,
        axis="y",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem=shell_tube,
        negative_elem=left_cup,
    )
    ctx.expect_within(drive_side, drive_side, axes="xz", inner_elem=middle_ring, outer_elem=outer_ring)
    ctx.expect_within(drive_side, drive_side, axes="xz", inner_elem=inner_ring, outer_elem=middle_ring)
    ctx.expect_gap(
        drive_side,
        drive_side,
        axis="y",
        min_gap=0.002,
        max_gap=0.004,
        positive_elem=outer_ring,
        negative_elem=middle_ring,
    )
    ctx.expect_gap(
        drive_side,
        drive_side,
        axis="y",
        min_gap=0.002,
        max_gap=0.004,
        positive_elem=middle_ring,
        negative_elem=inner_ring,
    )
    ctx.expect_overlap(
        left_crank,
        drive_side,
        axes="xz",
        min_overlap=0.02,
        elem_a=left_boss,
        elem_b=spindle_tube,
    )
    ctx.expect_gap(
        left_crank,
        left_crank,
        axis="y",
        max_gap=0.0015,
        max_penetration=0.0,
        positive_elem=left_boss,
        negative_elem=preload_cap,
    )
    ctx.expect_gap(
        right_pedal,
        drive_side,
        axis="y",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem=right_axle,
        negative_elem=right_eye,
    )
    ctx.expect_gap(
        left_crank,
        left_pedal,
        axis="y",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem=left_eye,
        negative_elem=left_axle,
    )
    ctx.expect_gap(
        right_pedal,
        left_pedal,
        axis="x",
        min_gap=0.26,
        positive_elem=right_platform,
        negative_elem=left_platform,
    )
    ctx.expect_within(right_pedal, right_pedal, axes="xy", inner_elem=right_recess, outer_elem=right_platform)
    ctx.expect_within(left_pedal, left_pedal, axes="xy", inner_elem=left_recess, outer_elem=left_platform)
    ctx.expect_gap(
        right_pedal,
        right_pedal,
        axis="z",
        min_gap=0.0015,
        max_gap=0.0025,
        positive_elem=right_rail,
        negative_elem=right_recess,
    )
    ctx.expect_gap(
        left_pedal,
        left_pedal,
        axis="z",
        min_gap=0.0015,
        max_gap=0.0025,
        positive_elem=left_rail,
        negative_elem=left_recess,
    )

    with ctx.pose({crank_rotation: math.pi / 2.0}):
        ctx.expect_gap(
            left_crank,
            drive_side,
            axis="z",
            min_gap=0.09,
            positive_elem=left_arm,
            negative_elem=right_arm,
        )
        ctx.expect_gap(
            left_pedal,
            right_pedal,
            axis="z",
            min_gap=0.26,
            positive_elem=left_platform,
            negative_elem=right_platform,
        )

    with ctx.pose({right_pedal_spin: 1.2, left_pedal_spin: -1.2}):
        ctx.expect_gap(
            right_pedal,
            drive_side,
            axis="y",
            max_gap=0.0005,
            max_penetration=0.0,
            positive_elem=right_axle,
            negative_elem=right_eye,
        )
        ctx.expect_gap(
            left_crank,
            left_pedal,
            axis="y",
            max_gap=0.0005,
            max_penetration=0.0,
            positive_elem=left_eye,
            negative_elem=left_axle,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
