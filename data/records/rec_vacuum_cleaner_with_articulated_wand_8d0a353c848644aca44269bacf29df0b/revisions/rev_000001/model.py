from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


UPPER_WAND_LENGTH = 0.22
LOWER_WAND_LENGTH = 0.44


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _add_wand_segment(
    part,
    *,
    prefix: str,
    length: float,
    tube_radius: float,
    rail_radius: float,
    frame_material,
    accent_material,
) -> None:
    part.visual(
        Cylinder(radius=0.017, length=0.084),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=accent_material,
        name=f"{prefix}_proximal_barrel",
    )
    tube_start = 0.018
    tube_end = length - 0.050
    part.visual(
        Cylinder(radius=tube_radius, length=tube_end - tube_start),
        origin=Origin(xyz=((tube_start + tube_end) * 0.5, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_material,
        name=f"{prefix}_tube",
    )
    part.visual(
        Box((0.050, 0.100, 0.016)),
        origin=Origin(xyz=(0.060, 0.0, 0.026)),
        material=accent_material,
        name=f"{prefix}_proximal_collar",
    )
    part.visual(
        Box((0.048, 0.100, 0.016)),
        origin=Origin(xyz=(length - 0.042, 0.0, 0.026)),
        material=accent_material,
        name=f"{prefix}_distal_collar",
    )
    rail_length = max(length - 0.090, 0.050)
    rail_center_x = length * 0.5
    for side, y_pos in (("left", 0.032), ("right", -0.032)):
        part.visual(
            Cylinder(radius=rail_radius, length=rail_length),
            origin=Origin(xyz=(rail_center_x, y_pos, 0.026), rpy=(0.0, pi / 2.0, 0.0)),
            material=accent_material,
            name=f"{prefix}_{side}_rail",
        )
    for side, y_pos in (("left", 0.047), ("right", -0.047)):
        part.visual(
            Box((0.036, 0.010, 0.044)),
            origin=Origin(xyz=(length - 0.014, y_pos, 0.0)),
            material=frame_material,
            name=f"{prefix}_{side}_fork",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="articulated_wand_vacuum")

    graphite = model.material("graphite", rgba=(0.17, 0.18, 0.20, 1.0))
    warm_gray = model.material("warm_gray", rgba=(0.56, 0.58, 0.60, 1.0))
    crimson = model.material("crimson", rgba=(0.77, 0.16, 0.14, 1.0))
    clear_smoke = model.material("clear_smoke", rgba=(0.58, 0.66, 0.72, 0.34))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    main_body = model.part("main_body")
    main_body.inertial = Inertial.from_geometry(
        Box((0.36, 0.18, 0.34)),
        mass=3.8,
        origin=Origin(xyz=(-0.11, 0.0, 0.16)),
    )
    main_body.visual(
        Cylinder(radius=0.050, length=0.165),
        origin=Origin(xyz=(-0.125, 0.0, 0.155), rpy=(0.0, pi / 2.0, 0.0)),
        material=graphite,
        name="motor_barrel",
    )
    main_body.visual(
        Cylinder(radius=0.042, length=0.095),
        origin=Origin(xyz=(-0.215, 0.0, 0.155), rpy=(0.0, pi / 2.0, 0.0)),
        material=warm_gray,
        name="rear_cap",
    )
    dust_cup = LatheGeometry.from_shell_profiles(
        [
            (0.014, -0.085),
            (0.046, -0.062),
            (0.056, -0.020),
            (0.060, 0.025),
            (0.054, 0.070),
            (0.032, 0.098),
        ],
        [
            (0.008, -0.080),
            (0.038, -0.058),
            (0.047, -0.018),
            (0.050, 0.022),
            (0.044, 0.064),
            (0.025, 0.092),
        ],
        segments=54,
        start_cap="flat",
        end_cap="flat",
    ).rotate_y(pi / 2.0)
    main_body.visual(
        _mesh("vacuum_dust_cup", dust_cup),
        origin=Origin(xyz=(-0.045, 0.0, 0.092)),
        material=clear_smoke,
        name="dust_cup",
    )
    handle_arch = tube_from_spline_points(
        [
            (-0.185, 0.0, 0.118),
            (-0.205, 0.0, 0.210),
            (-0.140, 0.0, 0.292),
            (-0.072, 0.0, 0.258),
            (-0.018, 0.0, 0.108),
        ],
        radius=0.017,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    main_body.visual(_mesh("vacuum_handle_arch", handle_arch), material=graphite, name="handle_arch")
    main_body.visual(
        Cylinder(radius=0.015, length=0.120),
        origin=Origin(xyz=(-0.130, 0.0, 0.272), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="grip_bar",
    )
    main_body.visual(
        Box((0.110, 0.082, 0.065)),
        origin=Origin(xyz=(-0.160, 0.0, 0.070)),
        material=crimson,
        name="battery_pack",
    )
    main_body.visual(
        Box((0.055, 0.100, 0.030)),
        origin=Origin(xyz=(-0.030, 0.0, 0.022)),
        material=warm_gray,
        name="front_spine",
    )
    for side, y_pos in (("left", 0.047), ("right", -0.047)):
        main_body.visual(
            Box((0.040, 0.010, 0.046)),
            origin=Origin(xyz=(0.010, y_pos, 0.0)),
            material=warm_gray,
            name=f"body_{side}_fork",
        )

    upper_wand = model.part("upper_wand")
    upper_wand.inertial = Inertial.from_geometry(
        Box((0.24, 0.11, 0.08)),
        mass=0.85,
        origin=Origin(xyz=(UPPER_WAND_LENGTH * 0.5, 0.0, 0.015)),
    )
    _add_wand_segment(
        upper_wand,
        prefix="upper",
        length=UPPER_WAND_LENGTH,
        tube_radius=0.019,
        rail_radius=0.0065,
        frame_material=graphite,
        accent_material=warm_gray,
    )

    lower_wand = model.part("lower_wand")
    lower_wand.inertial = Inertial.from_geometry(
        Box((0.48, 0.11, 0.08)),
        mass=1.05,
        origin=Origin(xyz=(LOWER_WAND_LENGTH * 0.5, 0.0, 0.015)),
    )
    _add_wand_segment(
        lower_wand,
        prefix="lower",
        length=LOWER_WAND_LENGTH,
        tube_radius=0.020,
        rail_radius=0.006,
        frame_material=graphite,
        accent_material=warm_gray,
    )

    floor_nozzle = model.part("floor_nozzle")
    floor_nozzle.inertial = Inertial.from_geometry(
        Box((0.32, 0.30, 0.08)),
        mass=0.75,
        origin=Origin(xyz=(0.140, 0.0, -0.018)),
    )
    floor_nozzle.visual(
        Cylinder(radius=0.016, length=0.084),
        origin=Origin(xyz=(0.000, 0.0, 0.010), rpy=(pi / 2.0, 0.0, 0.0)),
        material=warm_gray,
        name="nozzle_hinge_barrel",
    )
    for side, y_pos in (("left", 0.034), ("right", -0.034)):
        floor_nozzle.visual(
            Box((0.060, 0.014, 0.070)),
            origin=Origin(xyz=(0.038, y_pos, -0.020)),
            material=warm_gray,
            name=f"nozzle_{side}_yoke",
        )
    nozzle_head_geom = ExtrudeGeometry(
        rounded_rect_profile(0.270, 0.300, 0.028),
        0.028,
        center=True,
    )
    floor_nozzle.visual(
        _mesh("vacuum_nozzle_head", nozzle_head_geom),
        origin=Origin(xyz=(0.140, 0.0, -0.032)),
        material=graphite,
        name="nozzle_head",
    )
    floor_nozzle.visual(
        Box((0.030, 0.270, 0.010)),
        origin=Origin(xyz=(0.252, 0.0, -0.043)),
        material=crimson,
        name="front_bumper",
    )
    floor_nozzle.visual(
        Box((0.060, 0.240, 0.010)),
        origin=Origin(xyz=(0.058, 0.0, -0.033)),
        material=warm_gray,
        name="rear_axle_bar",
    )
    for side, y_pos in (("left", 0.115), ("right", -0.115)):
        floor_nozzle.visual(
            Cylinder(radius=0.017, length=0.014),
            origin=Origin(xyz=(0.058, y_pos, -0.033), rpy=(pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=f"rear_wheel_{side}",
        )

    model.articulation(
        "body_to_upper",
        ArticulationType.REVOLUTE,
        parent=main_body,
        child=upper_wand,
        origin=Origin(rpy=(0.0, 0.92, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=-0.45, upper=0.35),
    )
    model.articulation(
        "upper_to_lower",
        ArticulationType.REVOLUTE,
        parent=upper_wand,
        child=lower_wand,
        origin=Origin(xyz=(UPPER_WAND_LENGTH, 0.0, 0.0), rpy=(0.0, 0.36, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=-0.55, upper=0.50),
    )
    model.articulation(
        "lower_to_nozzle",
        ArticulationType.REVOLUTE,
        parent=lower_wand,
        child=floor_nozzle,
        origin=Origin(xyz=(LOWER_WAND_LENGTH, 0.0, 0.0), rpy=(0.0, -1.28, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=3.0, lower=-0.25, upper=0.70),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    main_body = object_model.get_part("main_body")
    lower_wand = object_model.get_part("lower_wand")
    floor_nozzle = object_model.get_part("floor_nozzle")
    body_to_upper = object_model.get_articulation("body_to_upper")
    upper_to_lower = object_model.get_articulation("upper_to_lower")
    lower_to_nozzle = object_model.get_articulation("lower_to_nozzle")

    ctx.expect_gap(
        main_body,
        floor_nozzle,
        axis="z",
        positive_elem="motor_barrel",
        negative_elem="nozzle_head",
        min_gap=0.55,
        name="main body rides well above the floor nozzle",
    )

    rest_lower = ctx.part_world_position(lower_wand)
    rest_nozzle = ctx.part_world_position(floor_nozzle)
    with ctx.pose({body_to_upper: -0.45, upper_to_lower: -0.55}):
        folded_lower = ctx.part_world_position(lower_wand)
        folded_nozzle = ctx.part_world_position(floor_nozzle)
    ctx.check(
        "wand elbows lift the cleaning head for storage",
        rest_lower is not None
        and folded_lower is not None
        and rest_nozzle is not None
        and folded_nozzle is not None
        and folded_lower[2] > rest_lower[2] + 0.06
        and folded_nozzle[2] > rest_nozzle[2] + 0.30,
        details=(
            f"rest_lower={rest_lower}, folded_lower={folded_lower}, "
            f"rest_nozzle={rest_nozzle}, folded_nozzle={folded_nozzle}"
        ),
    )

    rest_head = ctx.part_element_world_aabb(floor_nozzle, elem="nozzle_head")
    with ctx.pose({lower_to_nozzle: 0.45}):
        pitched_head = ctx.part_element_world_aabb(floor_nozzle, elem="nozzle_head")
    ctx.check(
        "floor nozzle pitches upward at the hinge",
        rest_head is not None and pitched_head is not None and pitched_head[1][2] > rest_head[1][2] + 0.04,
        details=f"rest={rest_head}, pitched={pitched_head}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
