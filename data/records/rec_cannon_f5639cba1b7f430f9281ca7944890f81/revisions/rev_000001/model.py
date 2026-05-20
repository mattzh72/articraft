from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _add_spoked_wheel(
    part,
    mesh_prefix: str,
    *,
    tire_major_radius: float,
    tire_tube_radius: float,
    rim_major_radius: float,
    rim_tube_radius: float,
    hub_radius: float,
    hub_width: float,
    spoke_count: int,
    tire_material,
    wheel_material,
    metal_material,
) -> None:
    tire_mesh = _mesh(
        f"{mesh_prefix}_tire",
        TorusGeometry(
            radius=tire_major_radius,
            tube=tire_tube_radius,
            radial_segments=18,
            tubular_segments=72,
        ).rotate_y(pi / 2.0),
    )
    rim_mesh = _mesh(
        f"{mesh_prefix}_rim",
        TorusGeometry(
            radius=rim_major_radius,
            tube=rim_tube_radius,
            radial_segments=16,
            tubular_segments=72,
        ).rotate_y(pi / 2.0),
    )

    part.visual(tire_mesh, material=tire_material, name="tire")
    part.visual(rim_mesh, material=wheel_material, name="rim")
    part.visual(
        Cylinder(radius=hub_radius, length=hub_width),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=wheel_material,
        name="hub",
    )
    part.visual(
        Cylinder(radius=hub_radius * 1.45, length=0.018),
        origin=Origin(xyz=(hub_width * 0.18, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=wheel_material,
        name="inner_flange",
    )
    part.visual(
        Cylinder(radius=hub_radius * 1.45, length=0.018),
        origin=Origin(xyz=(-hub_width * 0.18, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=wheel_material,
        name="outer_flange",
    )
    part.visual(
        Cylinder(radius=hub_radius * 0.32, length=hub_width * 0.92),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=metal_material,
        name="axle_bore_cap",
    )

    spoke_length = rim_major_radius - hub_radius * 1.20
    spoke_mid_radius = hub_radius * 1.20 + spoke_length * 0.5
    for spoke_index in range(spoke_count):
        angle = 2.0 * pi * spoke_index / spoke_count
        part.visual(
            Box((hub_width * 0.46, 0.022, spoke_length)),
            origin=Origin(
                xyz=(0.0, spoke_mid_radius * cos(angle), spoke_mid_radius * sin(angle)),
                rpy=(angle - pi / 2.0, 0.0, 0.0),
            ),
            material=wheel_material,
            name=f"spoke_{spoke_index:02d}",
        )


def _add_trail_leg(
    part,
    *,
    side_sign: float,
    trail_material,
    metal_material,
) -> None:
    part.visual(
        Cylinder(radius=0.055, length=0.12),
        origin=Origin(),
        material=metal_material,
        name="hinge_knuckle",
    )
    part.visual(
        Box((0.10, 0.18, 0.12)),
        origin=Origin(xyz=(0.0, -0.11, -0.01)),
        material=trail_material,
        name="junction_block",
    )
    part.visual(
        Box((0.09, 1.00, 0.08)),
        origin=Origin(xyz=(0.0, -0.62, -0.10), rpy=(0.10, 0.0, 0.0)),
        material=trail_material,
        name="main_beam",
    )
    part.visual(
        Box((0.06, 0.42, 0.05)),
        origin=Origin(xyz=(0.0, -0.34, 0.03), rpy=(0.18, 0.0, 0.0)),
        material=trail_material,
        name="upper_brace",
    )
    part.visual(
        Box((0.16, 0.10, 0.16)),
        origin=Origin(xyz=(0.0, -1.12, -0.22), rpy=(0.22, 0.0, 0.0)),
        material=metal_material,
        name="spade",
    )
    part.visual(
        Cylinder(radius=0.016, length=0.18),
        origin=Origin(
            xyz=(0.0, -1.02, -0.12),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=metal_material,
        name="handspike",
    )
    part.visual(
        Box((0.08, 0.24, 0.08)),
        origin=Origin(xyz=(0.0, -0.98, -0.17), rpy=(0.20, 0.0, 0.0)),
        material=trail_material,
        name="tail_bridge",
    )
    part.visual(
        Box((0.02, 0.16, 0.08)),
        origin=Origin(xyz=(0.018 * side_sign, -0.22, 0.01), rpy=(0.18, 0.0, 0.0)),
        material=metal_material,
        name="brace_lug",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mountain_pack_howitzer")

    olive_drab = model.material("olive_drab", rgba=(0.37, 0.40, 0.27, 1.0))
    dark_olive = model.material("dark_olive", rgba=(0.24, 0.27, 0.18, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.34, 0.36, 0.39, 1.0))
    steel = model.material("steel", rgba=(0.60, 0.62, 0.65, 1.0))
    tire_black = model.material("tire_black", rgba=(0.07, 0.07, 0.07, 1.0))

    carriage = model.part("carriage")
    carriage.inertial = Inertial.from_geometry(
        Box((1.55, 0.95, 0.90)),
        mass=540.0,
        origin=Origin(xyz=(0.0, -0.10, 0.45)),
    )
    carriage.visual(
        Cylinder(radius=0.050, length=1.26),
        origin=Origin(xyz=(0.0, 0.0, 0.46), rpy=(0.0, pi / 2.0, 0.0)),
        material=gunmetal,
        name="axle_beam",
    )
    carriage.visual(
        Cylinder(radius=0.062, length=0.042),
        origin=Origin(xyz=(0.63, 0.0, 0.46), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="left_spindle_stub",
    )
    carriage.visual(
        Cylinder(radius=0.062, length=0.042),
        origin=Origin(xyz=(-0.63, 0.0, 0.46), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="right_spindle_stub",
    )
    carriage.visual(
        Box((0.38, 0.24, 0.16)),
        origin=Origin(xyz=(0.0, -0.01, 0.44)),
        material=olive_drab,
        name="center_bed",
    )
    carriage.visual(
        Box((0.20, 0.12, 0.08)),
        origin=Origin(xyz=(0.0, 0.05, 0.56)),
        material=olive_drab,
        name="center_arch",
    )
    carriage.visual(
        Box((0.06, 0.24, 0.18)),
        origin=Origin(xyz=(0.0, -0.29, 0.31)),
        material=olive_drab,
        name="trail_junction",
    )
    carriage.visual(
        Box((0.14, 0.22, 0.08)),
        origin=Origin(xyz=(0.0, -0.19, 0.34), rpy=(-0.18, 0.0, 0.0)),
        material=olive_drab,
        name="drag_beam",
    )
    carriage.visual(
        Box((0.045, 0.28, 0.11)),
        origin=Origin(xyz=(0.195, 0.11, 0.57), rpy=(0.42, 0.0, 0.0)),
        material=olive_drab,
        name="left_cradle_brace",
    )
    carriage.visual(
        Box((0.045, 0.28, 0.11)),
        origin=Origin(xyz=(-0.195, 0.11, 0.57), rpy=(0.42, 0.0, 0.0)),
        material=olive_drab,
        name="right_cradle_brace",
    )
    carriage.visual(
        Box((0.06, 0.24, 0.18)),
        origin=Origin(xyz=(0.17, 0.22, 0.72)),
        material=olive_drab,
        name="left_cradle_cheek",
    )
    carriage.visual(
        Box((0.06, 0.24, 0.18)),
        origin=Origin(xyz=(-0.17, 0.22, 0.72)),
        material=olive_drab,
        name="right_cradle_cheek",
    )
    carriage.visual(
        Box((0.031, 0.22, 0.14)),
        origin=Origin(xyz=(0.0195, -0.31, 0.28)),
        material=gunmetal,
        name="left_trail_inner_cheek",
    )
    carriage.visual(
        Box((0.031, 0.22, 0.14)),
        origin=Origin(xyz=(0.1605, -0.31, 0.28)),
        material=gunmetal,
        name="left_trail_outer_cheek",
    )
    carriage.visual(
        Box((0.126, 0.10, 0.04)),
        origin=Origin(xyz=(0.090, -0.24, 0.35)),
        material=gunmetal,
        name="left_trail_bridge",
    )
    carriage.visual(
        Box((0.031, 0.22, 0.14)),
        origin=Origin(xyz=(-0.0195, -0.31, 0.28)),
        material=gunmetal,
        name="right_trail_inner_cheek",
    )
    carriage.visual(
        Box((0.031, 0.22, 0.14)),
        origin=Origin(xyz=(-0.1605, -0.31, 0.28)),
        material=gunmetal,
        name="right_trail_outer_cheek",
    )
    carriage.visual(
        Box((0.126, 0.10, 0.04)),
        origin=Origin(xyz=(-0.090, -0.24, 0.35)),
        material=gunmetal,
        name="right_trail_bridge",
    )
    barrel = model.part("barrel")
    barrel.inertial = Inertial.from_geometry(
        Box((0.28, 1.38, 0.28)),
        mass=235.0,
        origin=Origin(xyz=(0.0, 0.40, 0.0)),
    )
    barrel.visual(
        Cylinder(radius=0.055, length=0.28),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=gunmetal,
        name="trunnion_shaft",
    )
    barrel.visual(
        Box((0.20, 0.20, 0.18)),
        origin=Origin(xyz=(0.0, -0.14, 0.0)),
        material=dark_olive,
        name="breech_block",
    )
    barrel.visual(
        Box((0.14, 0.08, 0.16)),
        origin=Origin(xyz=(0.0, -0.26, 0.0)),
        material=gunmetal,
        name="breech_wedge",
    )
    barrel.visual(
        Cylinder(radius=0.095, length=0.40),
        origin=Origin(xyz=(0.0, 0.14, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=olive_drab,
        name="recoil_sleeve",
    )
    barrel.visual(
        Cylinder(radius=0.050, length=0.84),
        origin=Origin(xyz=(0.0, 0.56, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="tube",
    )
    barrel.visual(
        Cylinder(radius=0.060, length=0.12),
        origin=Origin(xyz=(0.0, 1.04, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="muzzle_reinforce",
    )
    barrel.visual(
        Cylinder(radius=0.040, length=0.36),
        origin=Origin(xyz=(0.0, 0.18, -0.10), rpy=(pi / 2.0, 0.0, 0.0)),
        material=olive_drab,
        name="recuperator",
    )
    barrel.visual(
        Box((0.05, 0.12, 0.10)),
        origin=Origin(xyz=(0.10, -0.02, 0.10)),
        material=dark_olive,
        name="sight_block",
    )

    left_trail = model.part("left_trail")
    left_trail.inertial = Inertial.from_geometry(
        Box((0.18, 1.34, 0.34)),
        mass=78.0,
        origin=Origin(xyz=(0.0, -0.64, -0.08)),
    )
    _add_trail_leg(
        left_trail,
        side_sign=1.0,
        trail_material=olive_drab,
        metal_material=gunmetal,
    )

    right_trail = model.part("right_trail")
    right_trail.inertial = Inertial.from_geometry(
        Box((0.18, 1.34, 0.34)),
        mass=78.0,
        origin=Origin(xyz=(0.0, -0.64, -0.08)),
    )
    _add_trail_leg(
        right_trail,
        side_sign=-1.0,
        trail_material=olive_drab,
        metal_material=gunmetal,
    )

    left_wheel = model.part("left_wheel")
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.46, length=0.11),
        mass=48.0,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_spoked_wheel(
        left_wheel,
        "left_wheel",
        tire_major_radius=0.38,
        tire_tube_radius=0.050,
        rim_major_radius=0.33,
        rim_tube_radius=0.020,
        hub_radius=0.072,
        hub_width=0.11,
        spoke_count=12,
        tire_material=tire_black,
        wheel_material=olive_drab,
        metal_material=steel,
    )

    right_wheel = model.part("right_wheel")
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.46, length=0.11),
        mass=48.0,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_spoked_wheel(
        right_wheel,
        "right_wheel",
        tire_major_radius=0.38,
        tire_tube_radius=0.050,
        rim_major_radius=0.33,
        rim_tube_radius=0.020,
        hub_radius=0.072,
        hub_width=0.11,
        spoke_count=12,
        tire_material=tire_black,
        wheel_material=olive_drab,
        metal_material=steel,
    )

    model.articulation(
        "barrel_elevation",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=barrel,
        origin=Origin(xyz=(0.0, 0.24, 0.71)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2200.0,
            velocity=0.9,
            lower=-0.10,
            upper=0.92,
        ),
    )
    model.articulation(
        "left_trail_spread",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=left_trail,
        origin=Origin(xyz=(0.09, -0.40, 0.28)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1600.0,
            velocity=0.8,
            lower=0.0,
            upper=0.62,
        ),
    )
    model.articulation(
        "right_trail_spread",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=right_trail,
        origin=Origin(xyz=(-0.09, -0.40, 0.28)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=1600.0,
            velocity=0.8,
            lower=0.0,
            upper=0.62,
        ),
    )
    model.articulation(
        "left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=left_wheel,
        origin=Origin(xyz=(0.706, 0.0, 0.46)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=320.0, velocity=18.0),
    )
    model.articulation(
        "right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=right_wheel,
        origin=Origin(xyz=(-0.706, 0.0, 0.46)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=320.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    barrel_joint = object_model.get_articulation("barrel_elevation")
    left_trail_joint = object_model.get_articulation("left_trail_spread")
    right_trail_joint = object_model.get_articulation("right_trail_spread")
    left_wheel_joint = object_model.get_articulation("left_wheel_spin")
    right_wheel_joint = object_model.get_articulation("right_wheel_spin")

    ctx.check(
        "barrel uses revolute elevation joint",
        barrel_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={barrel_joint.articulation_type}",
    )
    ctx.check(
        "split trails use revolute hinges",
        left_trail_joint.articulation_type == ArticulationType.REVOLUTE
        and right_trail_joint.articulation_type == ArticulationType.REVOLUTE,
        details=(
            f"left={left_trail_joint.articulation_type}, "
            f"right={right_trail_joint.articulation_type}"
        ),
    )
    ctx.check(
        "wheels spin continuously",
        left_wheel_joint.articulation_type == ArticulationType.CONTINUOUS
        and right_wheel_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=(
            f"left={left_wheel_joint.articulation_type}, "
            f"right={right_wheel_joint.articulation_type}"
        ),
    )

    tube_rest = ctx.part_element_world_aabb("barrel", elem="tube")
    left_spade_rest = ctx.part_element_world_aabb("left_trail", elem="spade")
    right_spade_rest = ctx.part_element_world_aabb("right_trail", elem="spade")

    with ctx.pose({barrel_joint: 0.80, left_trail_joint: 0.56, right_trail_joint: 0.56}):
        tube_high = ctx.part_element_world_aabb("barrel", elem="tube")
        left_spade_open = ctx.part_element_world_aabb("left_trail", elem="spade")
        right_spade_open = ctx.part_element_world_aabb("right_trail", elem="spade")

    tube_rest_max_z = tube_rest[1][2] if tube_rest else None
    tube_high_max_z = tube_high[1][2] if tube_high else None
    left_spade_rest_x = ((left_spade_rest[0][0] + left_spade_rest[1][0]) * 0.5) if left_spade_rest else None
    left_spade_open_x = ((left_spade_open[0][0] + left_spade_open[1][0]) * 0.5) if left_spade_open else None
    right_spade_rest_x = ((right_spade_rest[0][0] + right_spade_rest[1][0]) * 0.5) if right_spade_rest else None
    right_spade_open_x = ((right_spade_open[0][0] + right_spade_open[1][0]) * 0.5) if right_spade_open else None

    ctx.check(
        "barrel elevates upward",
        tube_rest_max_z is not None
        and tube_high_max_z is not None
        and tube_high_max_z > tube_rest_max_z + 0.35,
        details=f"rest={tube_rest_max_z}, elevated={tube_high_max_z}",
    )
    ctx.check(
        "left trail swings outward",
        left_spade_rest_x is not None
        and left_spade_open_x is not None
        and left_spade_open_x > left_spade_rest_x + 0.35,
        details=f"rest={left_spade_rest_x}, open={left_spade_open_x}",
    )
    ctx.check(
        "right trail swings outward",
        right_spade_rest_x is not None
        and right_spade_open_x is not None
        and right_spade_open_x < right_spade_rest_x - 0.35,
        details=f"rest={right_spade_rest_x}, open={right_spade_open_x}",
    )

    ctx.expect_gap(
        "barrel",
        "carriage",
        axis="z",
        positive_elem="breech_block",
        negative_elem="center_arch",
        min_gap=0.005,
        max_gap=0.20,
        name="breech clears carriage arch at rest",
    )
    ctx.expect_gap(
        "left_wheel",
        "carriage",
        axis="x",
        positive_elem="hub",
        negative_elem="left_spindle_stub",
        max_gap=0.05,
        max_penetration=1e-6,
        name="left wheel mounts close to spindle",
    )
    ctx.expect_gap(
        "carriage",
        "right_wheel",
        axis="x",
        positive_elem="right_spindle_stub",
        negative_elem="hub",
        max_gap=0.05,
        max_penetration=1e-6,
        name="right wheel mounts close to spindle",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
