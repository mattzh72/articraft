from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


ARM_BARREL_LENGTH = 0.030
ARM_BARREL_RADIUS = 0.010
SHADE_BARREL_LENGTH = 0.022
SHADE_BARREL_RADIUS = 0.008


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _rotate_xy(x: float, y: float, yaw: float) -> tuple[float, float]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (c * x - s * y, s * x + c * y)


def _hinge_frame_origin(
    radial_x: float,
    tangential_y: float,
    z: float,
    yaw: float,
    *,
    rpy: tuple[float, float, float] | None = None,
) -> Origin:
    x, y = _rotate_xy(radial_x, tangential_y, yaw)
    return Origin(xyz=(x, y, z), rpy=rpy or (0.0, 0.0, yaw))


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt(
        ((b[0] - a[0]) ** 2) + ((b[1] - a[1]) ** 2) + ((b[2] - a[2]) ** 2)
    )


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


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


def _add_spanning_cylinder(
    part,
    *,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    material,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(start, end)),
        origin=Origin(
            xyz=_midpoint(start, end),
            rpy=_rpy_for_cylinder(start, end),
        ),
        material=material,
        name=name,
    )


def _build_arm_tube_mesh():
    points = [
        (0.022, 0.0, 0.002),
        (0.120, 0.0, 0.028),
        (0.285, 0.0, 0.082),
        (0.430, 0.0, 0.132),
        (0.474, 0.0, 0.150),
    ]
    return _mesh(
        "lamp_arm_tube",
        tube_from_spline_points(
            points,
            radius=0.0115,
            samples_per_segment=16,
            radial_segments=18,
            cap_ends=True,
        ),
    )


def _build_shade_shell_mesh():
    return _mesh(
        "lamp_shade_can",
        CylinderGeometry(
            radius=0.055,
            height=0.110,
            radial_segments=40,
            closed=False,
        ),
    )


def _build_shade_rim_mesh():
    return _mesh(
        "lamp_shade_rim",
        TorusGeometry(
            radius=0.052,
            tube=0.003,
            radial_segments=14,
            tubular_segments=44,
        ),
    )


def _add_crown_clevis(
    stand,
    *,
    yaw: float,
    crown_z: float,
    radial_offset: float,
    material,
) -> None:
    cheek_offset = (ARM_BARREL_LENGTH * 0.5) + 0.0025
    _ = (stand, yaw, crown_z, radial_offset, material, cheek_offset)
    return None


def _add_arm_geometry(arm_part, arm_tube_mesh, arm_material, hardware_material) -> None:
    arm_part.visual(
        Cylinder(radius=ARM_BARREL_RADIUS, length=ARM_BARREL_LENGTH),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_material,
        name="root_barrel",
    )
    arm_part.visual(
        Box((0.032, 0.018, 0.018)),
        origin=Origin(xyz=(0.016, 0.0, 0.002)),
        material=hardware_material,
        name="root_block",
    )
    arm_part.visual(
        arm_tube_mesh,
        material=arm_material,
        name="arm_tube",
    )
    _add_spanning_cylinder(
        arm_part,
        start=(0.462, 0.0, 0.146),
        end=(0.497, 0.0, 0.156),
        radius=0.0085,
        material=hardware_material,
        name="tip_spindle",
    )
def _add_shade_geometry(
    shade_part,
    shade_shell_mesh,
    shade_material,
    hardware_material,
    bulb_material,
) -> None:
    shade_part.visual(
        Cylinder(radius=SHADE_BARREL_RADIUS, length=SHADE_BARREL_LENGTH),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_material,
        name="shade_barrel",
    )
    shade_part.visual(
        Cylinder(radius=0.008, length=0.040),
        origin=Origin(xyz=(0.024, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware_material,
        name="shade_neck",
    )
    shade_part.visual(
        shade_shell_mesh,
        origin=Origin(xyz=(0.086, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shade_material,
        name="shade_can",
    )
    shade_part.visual(
        Cylinder(radius=0.054, length=0.020),
        origin=Origin(xyz=(0.041, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shade_material,
        name="rear_cap",
    )
    shade_part.visual(
        Cylinder(radius=0.014, length=0.040),
        origin=Origin(xyz=(0.056, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware_material,
        name="socket",
    )
    shade_part.visual(
        Sphere(radius=0.019),
        origin=Origin(xyz=(0.087, 0.0, 0.0)),
        material=bulb_material,
        name="bulb",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="multi_head_floor_lamp")

    base_finish = model.material("base_finish", rgba=(0.11, 0.11, 0.12, 1.0))
    stem_finish = model.material("stem_finish", rgba=(0.19, 0.19, 0.20, 1.0))
    arm_finish = model.material("arm_finish", rgba=(0.60, 0.48, 0.28, 1.0))
    hardware_finish = model.material("hardware_finish", rgba=(0.25, 0.25, 0.26, 1.0))
    shade_finish = model.material("shade_finish", rgba=(0.83, 0.80, 0.72, 1.0))
    shade_inner = model.material("shade_inner", rgba=(0.95, 0.93, 0.88, 1.0))
    bulb_frosted = model.material("bulb_frosted", rgba=(0.98, 0.96, 0.88, 0.72))

    arm_tube_mesh = _build_arm_tube_mesh()
    shade_shell_mesh = _build_shade_shell_mesh()

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.160, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=base_finish,
        name="base_disk",
    )
    stand.visual(
        Cylinder(radius=0.072, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=base_finish,
        name="base_cap",
    )
    stand.visual(
        Cylinder(radius=0.017, length=1.455),
        origin=Origin(xyz=(0.0, 0.0, 0.7595)),
        material=stem_finish,
        name="main_column",
    )
    stand.visual(
        Cylinder(radius=0.026, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 1.542)),
        material=stem_finish,
        name="crown_sleeve",
    )
    stand.visual(
        Cylinder(radius=0.043, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 1.598)),
        material=hardware_finish,
        name="crown_hub",
    )

    crown_z = 1.598
    crown_radial_offset = 0.050
    branch_yaws = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    for yaw in branch_yaws:
        _add_crown_clevis(
            stand,
            yaw=yaw,
            crown_z=crown_z,
            radial_offset=crown_radial_offset,
            material=hardware_finish,
        )
    stand.inertial = Inertial.from_geometry(
        Box((0.34, 0.34, 1.62)),
        mass=13.5,
        origin=Origin(xyz=(0.0, 0.0, 0.81)),
    )

    arm_tip_origin = Origin(xyz=(0.505, 0.0, 0.156))
    for index, yaw in enumerate(branch_yaws):
        arm = model.part(f"arm_{index}")
        _add_arm_geometry(arm, arm_tube_mesh, arm_finish, hardware_finish)
        arm.inertial = Inertial.from_geometry(
            Box((0.54, 0.06, 0.22)),
            mass=0.85,
            origin=Origin(xyz=(0.270, 0.0, 0.085)),
        )

        shade = model.part(f"shade_{index}")
        _add_shade_geometry(
            shade,
            shade_shell_mesh,
            shade_finish,
            hardware_finish,
            bulb_frosted,
        )
        shade.inertial = Inertial.from_geometry(
            Box((0.17, 0.13, 0.13)),
            mass=0.42,
            origin=Origin(xyz=(0.085, 0.0, 0.0)),
        )

        model.articulation(
            f"stand_to_arm_{index}",
            ArticulationType.REVOLUTE,
            parent=stand,
            child=arm,
            origin=Origin(
                xyz=(
                    crown_radial_offset * math.cos(yaw),
                    crown_radial_offset * math.sin(yaw),
                    crown_z,
                ),
                rpy=(0.0, 0.0, yaw),
            ),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=16.0,
                velocity=1.0,
                lower=-0.45,
                upper=0.60,
            ),
        )
        model.articulation(
            f"arm_{index}_to_shade_{index}",
            ArticulationType.REVOLUTE,
            parent=arm,
            child=shade,
            origin=arm_tip_origin,
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=1.4,
                lower=-0.95,
                upper=0.55,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    stand = object_model.get_part("stand")

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
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
    for index in range(3):
        arm = object_model.get_part(f"arm_{index}")
        shade = object_model.get_part(f"shade_{index}")
        ctx.allow_overlap(
            arm,
            shade,
            elem_a="tip_spindle",
            elem_b="shade_barrel",
            reason="Shade hinge pin is captured within the cylindrical tilt barrel.",
        )
        ctx.allow_overlap(
            arm,
            stand,
            elem_a="root_barrel",
            elem_b="crown_hub",
            reason="Arm root hinge barrel is seated into the crown hub pivot seat.",
        )
    ctx.fail_if_parts_overlap_in_current_pose()

    arms = [object_model.get_part(f"arm_{index}") for index in range(3)]
    shades = [object_model.get_part(f"shade_{index}") for index in range(3)]
    arm_joints = [object_model.get_articulation(f"stand_to_arm_{index}") for index in range(3)]
    shade_joints = [
        object_model.get_articulation(f"arm_{index}_to_shade_{index}") for index in range(3)
    ]

    for index, arm in enumerate(arms):
        ctx.expect_contact(arm, stand, name=f"arm_{index}_mounted_to_crown")
        ctx.expect_origin_gap(
            arm,
            stand,
            axis="z",
            min_gap=1.50,
            max_gap=1.70,
            name=f"arm_{index}_hinge_height",
        )

    for index, (arm, shade) in enumerate(zip(arms, shades)):
        ctx.expect_contact(shade, arm, name=f"shade_{index}_mounted_to_arm")

    arm_tube_rest = ctx.part_element_world_aabb(arms[0], elem="arm_tube")
    assert arm_tube_rest is not None
    with ctx.pose({arm_joints[0]: 0.35}):
        arm_tube_up = ctx.part_element_world_aabb(arms[0], elem="arm_tube")
        assert arm_tube_up is not None
        ctx.expect_contact(arms[0], stand, name="arm_0_remains_pinned_when_raised")
        assert arm_tube_up[1][2] > arm_tube_rest[1][2] + 0.06

    shade_can_rest = ctx.part_element_world_aabb(shades[0], elem="shade_can")
    assert shade_can_rest is not None
    with ctx.pose({shade_joints[0]: 0.40}):
        shade_can_up = ctx.part_element_world_aabb(shades[0], elem="shade_can")
        assert shade_can_up is not None
        ctx.expect_contact(shades[0], arms[0], name="shade_0_remains_pinned_when_tilted")
        assert shade_can_up[1][2] > shade_can_rest[1][2] + 0.02

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
