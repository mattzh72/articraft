from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

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
ARM_LENGTH = 0.170
RIGHT_ARM_ANGLE = -1.0


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _circle_profile(radius: float, segments: int = 56) -> list[tuple[float, float]]:
    return [
        (radius * cos(2.0 * pi * i / segments), radius * sin(2.0 * pi * i / segments))
        for i in range(segments)
    ]


def _gear_profile(radius_tip: float, radius_root: float, teeth: int) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    step = 2.0 * pi / teeth
    for tooth in range(teeth):
        angle = tooth * step
        points.extend(
            [
                (radius_root * cos(angle), radius_root * sin(angle)),
                (
                    radius_tip * cos(angle + step * 0.20),
                    radius_tip * sin(angle + step * 0.20),
                ),
                (
                    radius_tip * cos(angle + step * 0.54),
                    radius_tip * sin(angle + step * 0.54),
                ),
                (
                    radius_root * cos(angle + step * 0.78),
                    radius_root * sin(angle + step * 0.78),
                ),
            ]
        )
    return points


def _rounded_section(width: float, thickness: float, y_pos: float) -> list[tuple[float, float, float]]:
    corner = max(min(width, thickness) * 0.22, 0.0012)
    return [(x, y_pos, z) for x, z in rounded_rect_profile(width, thickness, corner, corner_segments=6)]


def _arm_mesh(
    *,
    name: str,
    length: float,
    widths: tuple[float, float, float, float],
    thicknesses: tuple[float, float, float, float],
):
    sections = [
        _rounded_section(widths[0], thicknesses[0], 0.0),
        _rounded_section(widths[1], thicknesses[1], length * 0.18),
        _rounded_section(widths[2], thicknesses[2], length * 0.60),
        _rounded_section(widths[3], thicknesses[3], length),
    ]
    return _save_mesh(name, section_loft(sections))


def _chainring_mesh(
    name: str,
    *,
    tip_radius: float,
    root_radius: float,
    inner_radius: float,
    thickness: float,
    teeth: int,
):
    return _save_mesh(
        name,
        ExtrudeWithHolesGeometry(
            _gear_profile(tip_radius, root_radius, teeth),
            [_circle_profile(inner_radius, 44)],
            height=thickness,
            center=True,
        ).rotate_y(pi / 2.0),
    )


def _pedal_visuals(part, *, side_sign: float, body_material, clip_material) -> None:
    part.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(side_sign * 0.013, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=clip_material,
        name="inner_collar",
    )
    part.visual(
        Box((0.060, 0.026, 0.012)),
        origin=Origin(xyz=(side_sign * 0.043, 0.0, 0.0)),
        material=body_material,
        name="body_core",
    )
    part.visual(
        Box((0.036, 0.014, 0.009)),
        origin=Origin(xyz=(side_sign * 0.045, 0.018, 0.004)),
        material=clip_material,
        name="front_binding",
    )
    part.visual(
        Box((0.036, 0.014, 0.009)),
        origin=Origin(xyz=(side_sign * 0.045, -0.018, 0.004)),
        material=clip_material,
        name="rear_binding",
    )
    part.visual(
        Box((0.046, 0.018, 0.008)),
        origin=Origin(xyz=(side_sign * 0.046, 0.0, -0.008)),
        material=body_material,
        name="lower_keel",
    )
    part.visual(
        Cylinder(radius=0.004, length=0.038),
        origin=Origin(xyz=(side_sign * 0.044, 0.0, 0.004), rpy=(0.0, pi / 2.0, 0.0)),
        material=clip_material,
        name="spring_bar",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mountain_bike_crankset", assets=ASSETS)

    alloy = model.material("alloy", rgba=(0.72, 0.74, 0.77, 1.0))
    dark_alloy = model.material("dark_alloy", rgba=(0.34, 0.36, 0.39, 1.0))
    anodized_black = model.material("anodized_black", rgba=(0.12, 0.12, 0.13, 1.0))
    steel = model.material("steel", rgba=(0.55, 0.57, 0.60, 1.0))
    pedal_composite = model.material("pedal_composite", rgba=(0.10, 0.10, 0.11, 1.0))

    bottom_bracket = model.part("bottom_bracket")
    bottom_bracket.inertial = Inertial.from_geometry(
        Cylinder(radius=0.026, length=0.083),
        mass=0.45,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    bb_shell = _save_mesh(
        "crankset_bottom_bracket_shell.obj",
        ExtrudeWithHolesGeometry(
            _circle_profile(0.026, 64),
            [_circle_profile(0.0185, 56)],
            height=0.083,
            center=True,
        ).rotate_y(pi / 2.0),
    )
    bottom_bracket.visual(bb_shell, material=anodized_black, name="shell")

    crankset = model.part("crankset")
    crankset.inertial = Inertial.from_geometry(
        Box((0.36, 0.28, 0.10)),
        mass=1.65,
        origin=Origin(),
    )

    right_pedal_y = ARM_LENGTH * cos(RIGHT_ARM_ANGLE)
    right_pedal_z = ARM_LENGTH * sin(RIGHT_ARM_ANGLE)
    left_pedal_y = ARM_LENGTH * cos(RIGHT_ARM_ANGLE + pi)
    left_pedal_z = ARM_LENGTH * sin(RIGHT_ARM_ANGLE + pi)

    crankset.visual(
        Cylinder(radius=0.015, length=0.170),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="spindle",
    )
    crankset.visual(
        Cylinder(radius=0.022, length=0.010),
        origin=Origin(xyz=(0.0465, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_alloy,
        name="right_bearing_collar",
    )
    crankset.visual(
        Cylinder(radius=0.022, length=0.010),
        origin=Origin(xyz=(-0.0465, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_alloy,
        name="left_bearing_collar",
    )
    crankset.visual(
        Cylinder(radius=0.029, length=0.014),
        origin=Origin(xyz=(0.053, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=alloy,
        name="spider_hub",
    )
    crankset.visual(
        _chainring_mesh(
            "crankset_outer_chainring.obj",
            tip_radius=0.114,
            root_radius=0.109,
            inner_radius=0.072,
            thickness=0.004,
            teeth=18,
        ),
        origin=Origin(xyz=(0.062, 0.0, 0.0)),
        material=anodized_black,
        name="outer_ring",
    )
    crankset.visual(
        _chainring_mesh(
            "crankset_inner_chainring.obj",
            tip_radius=0.093,
            root_radius=0.089,
            inner_radius=0.050,
            thickness=0.004,
            teeth=14,
        ),
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        material=dark_alloy,
        name="inner_ring",
    )

    spider_arm_mesh = _arm_mesh(
        name="crankset_spider_arm.obj",
        length=0.075,
        widths=(0.020, 0.018, 0.015, 0.012),
        thicknesses=(0.010, 0.010, 0.009, 0.008),
    )
    for idx, angle in enumerate((0.0, pi / 2.0, pi, 3.0 * pi / 2.0)):
        crankset.visual(
            spider_arm_mesh,
            origin=Origin(xyz=(0.056, 0.0, 0.0), rpy=(angle, 0.0, 0.0)),
            material=alloy,
            name=f"spider_arm_{idx}",
        )

    crank_arm_mesh = _arm_mesh(
        name="crankset_arm.obj",
        length=ARM_LENGTH,
        widths=(0.036, 0.044, 0.034, 0.024),
        thicknesses=(0.020, 0.017, 0.014, 0.012),
    )
    crankset.visual(
        Cylinder(radius=0.024, length=0.024),
        origin=Origin(xyz=(0.056, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=alloy,
        name="right_root_hub",
    )
    crankset.visual(
        Cylinder(radius=0.024, length=0.024),
        origin=Origin(xyz=(-0.056, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=alloy,
        name="left_root_hub",
    )
    crankset.visual(
        crank_arm_mesh,
        origin=Origin(xyz=(0.069, 0.0, 0.0), rpy=(RIGHT_ARM_ANGLE, 0.0, 0.0)),
        material=alloy,
        name="right_arm",
    )
    crankset.visual(
        crank_arm_mesh,
        origin=Origin(xyz=(-0.069, 0.0, 0.0), rpy=(RIGHT_ARM_ANGLE + pi, 0.0, 0.0)),
        material=alloy,
        name="left_arm",
    )
    crankset.visual(
        Cylinder(radius=0.017, length=0.020),
        origin=Origin(xyz=(0.069, right_pedal_y, right_pedal_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=alloy,
        name="right_pedal_boss",
    )
    crankset.visual(
        Cylinder(radius=0.006, length=0.014),
        origin=Origin(xyz=(0.084, right_pedal_y, right_pedal_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="right_stub_axle",
    )
    crankset.visual(
        Cylinder(radius=0.017, length=0.020),
        origin=Origin(xyz=(-0.069, left_pedal_y, left_pedal_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=alloy,
        name="left_pedal_boss",
    )
    crankset.visual(
        Cylinder(radius=0.006, length=0.014),
        origin=Origin(xyz=(-0.084, left_pedal_y, left_pedal_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="left_stub_axle",
    )

    right_pedal = model.part("right_pedal")
    right_pedal.inertial = Inertial.from_geometry(
        Box((0.085, 0.080, 0.028)),
        mass=0.18,
        origin=Origin(xyz=(0.043, 0.0, 0.0)),
    )
    _pedal_visuals(right_pedal, side_sign=1.0, body_material=pedal_composite, clip_material=steel)

    left_pedal = model.part("left_pedal")
    left_pedal.inertial = Inertial.from_geometry(
        Box((0.085, 0.080, 0.028)),
        mass=0.18,
        origin=Origin(xyz=(-0.043, 0.0, 0.0)),
    )
    _pedal_visuals(left_pedal, side_sign=-1.0, body_material=pedal_composite, clip_material=steel)

    model.articulation(
        "bb_spin",
        ArticulationType.REVOLUTE,
        parent=bottom_bracket,
        child=crankset,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=8.0, lower=-pi, upper=pi),
    )
    model.articulation(
        "right_pedal_spin",
        ArticulationType.REVOLUTE,
        parent=crankset,
        child=right_pedal,
        origin=Origin(xyz=(0.084, right_pedal_y, right_pedal_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=18.0, lower=-pi, upper=pi),
    )
    model.articulation(
        "left_pedal_spin",
        ArticulationType.REVOLUTE,
        parent=crankset,
        child=left_pedal,
        origin=Origin(xyz=(-0.084, left_pedal_y, left_pedal_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=18.0, lower=-pi, upper=pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    bottom_bracket = object_model.get_part("bottom_bracket")
    crankset = object_model.get_part("crankset")
    right_pedal = object_model.get_part("right_pedal")
    left_pedal = object_model.get_part("left_pedal")
    bb_spin = object_model.get_articulation("bb_spin")
    right_pedal_spin = object_model.get_articulation("right_pedal_spin")
    left_pedal_spin = object_model.get_articulation("left_pedal_spin")

    bb_shell = bottom_bracket.get_visual("shell")
    inner_ring = crankset.get_visual("inner_ring")
    outer_ring = crankset.get_visual("outer_ring")
    right_bearing_collar = crankset.get_visual("right_bearing_collar")
    left_bearing_collar = crankset.get_visual("left_bearing_collar")
    right_stub_axle = crankset.get_visual("right_stub_axle")
    left_stub_axle = crankset.get_visual("left_stub_axle")
    right_pedal_boss = crankset.get_visual("right_pedal_boss")
    left_pedal_boss = crankset.get_visual("left_pedal_boss")
    right_inner_collar = right_pedal.get_visual("inner_collar")
    left_inner_collar = left_pedal.get_visual("inner_collar")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.019)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128, overlap_tol=0.001, overlap_volume_tol=0.0)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_origin_distance(crankset, bottom_bracket, axes="yz", max_dist=0.001, name="spindle_axis_concentric")
    ctx.expect_contact(crankset, bottom_bracket, elem_a=right_bearing_collar, elem_b=bb_shell)
    ctx.expect_contact(crankset, bottom_bracket, elem_a=left_bearing_collar, elem_b=bb_shell)
    ctx.expect_gap(
        crankset,
        bottom_bracket,
        axis="x",
        min_gap=0.010,
        positive_elem=inner_ring,
        negative_elem=bb_shell,
        name="inner_ring_outboard_of_bb_shell",
    )
    ctx.expect_gap(
        crankset,
        bottom_bracket,
        axis="x",
        min_gap=0.017,
        positive_elem=outer_ring,
        negative_elem=bb_shell,
        name="outer_ring_outboard_of_bb_shell",
    )
    ctx.expect_contact(right_pedal, crankset, elem_a=right_inner_collar, elem_b=right_stub_axle)
    ctx.expect_contact(left_pedal, crankset, elem_a=left_inner_collar, elem_b=left_stub_axle)
    ctx.expect_within(
        right_pedal,
        crankset,
        axes="yz",
        inner_elem=right_inner_collar,
        outer_elem=right_pedal_boss,
    )
    ctx.expect_within(
        left_pedal,
        crankset,
        axes="yz",
        inner_elem=left_inner_collar,
        outer_elem=left_pedal_boss,
    )
    ctx.expect_gap(left_pedal, bottom_bracket, axis="z", min_gap=0.080, name="left_pedal_above_shell_rest")
    ctx.expect_gap(bottom_bracket, right_pedal, axis="z", min_gap=0.080, name="right_pedal_below_shell_rest")

    with ctx.pose({bb_spin: 2.2, right_pedal_spin: 0.7, left_pedal_spin: -0.6}):
        ctx.expect_contact(right_pedal, crankset, elem_a=right_inner_collar, elem_b=right_stub_axle)
        ctx.expect_contact(left_pedal, crankset, elem_a=left_inner_collar, elem_b=left_stub_axle)
        ctx.expect_gap(right_pedal, bottom_bracket, axis="z", min_gap=0.080, name="right_pedal_above_shell_rotated")
        ctx.expect_gap(bottom_bracket, left_pedal, axis="z", min_gap=0.080, name="left_pedal_below_shell_rotated")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
