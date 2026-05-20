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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _xy_section(
    width_x: float,
    width_y: float,
    z: float,
    *,
    x_offset: float = 0.0,
    y_offset: float = 0.0,
    radius: float | None = None,
):
    fillet = radius if radius is not None else min(width_x, width_y) * 0.24
    return tuple(
        (x + x_offset, y + y_offset, z)
        for x, y in rounded_rect_profile(width_x, width_y, fillet, corner_segments=8)
    )


def _yz_section(
    width_y: float,
    height_z: float,
    x: float,
    *,
    y_offset: float = 0.0,
    z_offset: float = 0.0,
    radius: float | None = None,
):
    fillet = radius if radius is not None else min(width_y, height_z) * 0.28
    return tuple(
        (x, y + y_offset, z + z_offset)
        for y, z in rounded_rect_profile(width_y, height_z, fillet, corner_segments=8)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="robotic_leg_module")

    shell_grey = model.material("shell_grey", rgba=(0.63, 0.66, 0.70, 1.0))
    graphite = model.material("graphite", rgba=(0.16, 0.17, 0.19, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.28, 0.30, 0.33, 1.0))
    actuator_black = model.material("actuator_black", rgba=(0.08, 0.09, 0.10, 1.0))
    sole_rubber = model.material("sole_rubber", rgba=(0.10, 0.11, 0.12, 1.0))

    upper_housing = model.part("upper_housing")
    housing_shell = section_loft(
        [
            _xy_section(0.14, 0.12, 0.03, x_offset=-0.006),
            _xy_section(0.19, 0.13, 0.14, x_offset=-0.002),
            _xy_section(0.16, 0.11, 0.25, x_offset=-0.012),
        ]
    )
    upper_housing.visual(
        _mesh("upper_housing_shell", housing_shell),
        material=shell_grey,
        name="housing_shell",
    )
    upper_housing.visual(
        Box((0.11, 0.08, 0.028)),
        origin=Origin(xyz=(-0.010, 0.0, 0.034)),
        material=graphite,
        name="hip_mount_block",
    )
    for side in (-1.0, 1.0):
        upper_housing.visual(
            Box((0.030, 0.024, 0.092)),
            origin=Origin(xyz=(0.0, side * 0.053, 0.010)),
            material=dark_steel,
            name=f"hip_cheek_{'left' if side > 0 else 'right'}",
        )
        upper_housing.visual(
            Cylinder(radius=0.024, length=0.024),
            origin=Origin(
                xyz=(0.0, side * 0.053, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_steel,
            name=f"hip_lug_{'left' if side > 0 else 'right'}",
        )
    upper_housing.visual(
        Box((0.072, 0.070, 0.072)),
        origin=Origin(xyz=(-0.088, 0.0, 0.182)),
        material=actuator_black,
        name="actuator_pack",
    )
    upper_housing.inertial = Inertial.from_geometry(
        Box((0.20, 0.14, 0.28)),
        mass=16.0,
        origin=Origin(xyz=(-0.005, 0.0, 0.14)),
    )

    thigh_link = model.part("thigh_link")
    thigh_shell = section_loft(
        [
            _xy_section(0.080, 0.066, -0.014, x_offset=0.0),
            _xy_section(0.056, 0.046, -0.205, x_offset=0.018),
            _xy_section(0.064, 0.056, -0.398, x_offset=0.008),
        ]
    )
    thigh_link.visual(
        Cylinder(radius=0.020, length=0.082),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="hip_barrel",
    )
    thigh_link.visual(
        _mesh("thigh_shell", thigh_shell),
        material=shell_grey,
        name="thigh_shell",
    )
    for side in (-1.0, 1.0):
        thigh_link.visual(
            Box((0.032, 0.018, 0.072)),
            origin=Origin(xyz=(0.008, side * 0.045, -0.384)),
            material=dark_steel,
            name=f"knee_cheek_{'left' if side > 0 else 'right'}",
        )
        thigh_link.visual(
            Cylinder(radius=0.018, length=0.022),
            origin=Origin(
                xyz=(0.008, side * 0.045, -0.420),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_steel,
            name=f"knee_lug_{'left' if side > 0 else 'right'}",
        )
    thigh_link.visual(
        Box((0.044, 0.074, 0.060)),
        origin=Origin(xyz=(0.010, 0.0, -0.364)),
        material=dark_steel,
        name="knee_carrier",
    )
    thigh_link.visual(
        Box((0.038, 0.032, 0.070)),
        origin=Origin(xyz=(0.034, 0.0, -0.290)),
        material=actuator_black,
        name="thigh_motor_cover",
    )
    thigh_link.inertial = Inertial.from_geometry(
        Box((0.10, 0.08, 0.44)),
        mass=7.0,
        origin=Origin(xyz=(0.010, 0.0, -0.210)),
    )

    shank_link = model.part("shank_link")
    shank_shell = section_loft(
        [
            _xy_section(0.066, 0.054, -0.014, x_offset=-0.004),
            _xy_section(0.046, 0.040, -0.190, x_offset=-0.020),
            _xy_section(0.058, 0.050, -0.376, x_offset=-0.030),
        ]
    )
    shank_link.visual(
        Cylinder(radius=0.017, length=0.068),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="knee_barrel",
    )
    shank_link.visual(
        _mesh("shank_shell", shank_shell),
        material=shell_grey,
        name="shank_shell",
    )
    for side in (-1.0, 1.0):
        shank_link.visual(
            Box((0.028, 0.016, 0.060)),
            origin=Origin(xyz=(-0.010, side * 0.039, -0.370)),
            material=dark_steel,
            name=f"ankle_cheek_{'left' if side > 0 else 'right'}",
        )
        shank_link.visual(
            Cylinder(radius=0.016, length=0.020),
            origin=Origin(
                xyz=(-0.010, side * 0.039, -0.400),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_steel,
            name=f"ankle_lug_{'left' if side > 0 else 'right'}",
        )
    shank_link.visual(
        Box((0.040, 0.066, 0.054)),
        origin=Origin(xyz=(-0.014, 0.0, -0.350)),
        material=dark_steel,
        name="ankle_carrier",
    )
    shank_link.visual(
        Box((0.034, 0.030, 0.064)),
        origin=Origin(xyz=(-0.034, 0.0, -0.246)),
        material=actuator_black,
        name="shank_motor_cover",
    )
    shank_link.inertial = Inertial.from_geometry(
        Box((0.09, 0.07, 0.42)),
        mass=5.0,
        origin=Origin(xyz=(-0.016, 0.0, -0.200)),
    )

    foot_section = model.part("foot_section")
    foot_shell = section_loft(
        [
            _yz_section(0.050, 0.040, -0.030, z_offset=-0.022),
            _yz_section(0.058, 0.050, 0.000, z_offset=-0.030),
            _yz_section(0.054, 0.038, 0.115, z_offset=-0.040),
            _yz_section(0.040, 0.024, 0.195, z_offset=-0.032),
        ]
    )
    foot_section.visual(
        Cylinder(radius=0.015, length=0.058),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="ankle_barrel",
    )
    foot_section.visual(
        _mesh("foot_shell", foot_shell),
        material=shell_grey,
        name="foot_shell",
    )
    foot_section.visual(
        Box((0.070, 0.060, 0.014)),
        origin=Origin(xyz=(0.145, 0.0, -0.050)),
        material=sole_rubber,
        name="toe_pad",
    )
    foot_section.visual(
        Box((0.045, 0.052, 0.012)),
        origin=Origin(xyz=(-0.026, 0.0, -0.044)),
        material=sole_rubber,
        name="heel_pad",
    )
    foot_section.inertial = Inertial.from_geometry(
        Box((0.25, 0.08, 0.07)),
        mass=2.2,
        origin=Origin(xyz=(0.085, 0.0, -0.032)),
    )

    model.articulation(
        "hip_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_housing,
        child=thigh_link,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=2.2,
            lower=-0.65,
            upper=1.00,
        ),
    )
    model.articulation(
        "knee_pitch",
        ArticulationType.REVOLUTE,
        parent=thigh_link,
        child=shank_link,
        origin=Origin(xyz=(0.008, 0.0, -0.420)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=160.0,
            velocity=2.8,
            lower=0.0,
            upper=1.75,
        ),
    )
    model.articulation(
        "ankle_pitch",
        ArticulationType.REVOLUTE,
        parent=shank_link,
        child=foot_section,
        origin=Origin(xyz=(-0.010, 0.0, -0.400)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=70.0,
            velocity=3.0,
            lower=-0.60,
            upper=0.55,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    upper_housing = object_model.get_part("upper_housing")
    thigh_link = object_model.get_part("thigh_link")
    shank_link = object_model.get_part("shank_link")
    foot_section = object_model.get_part("foot_section")

    hip_pitch = object_model.get_articulation("hip_pitch")
    knee_pitch = object_model.get_articulation("knee_pitch")
    ankle_pitch = object_model.get_articulation("ankle_pitch")

    def _aabb_dims(aabb):
        if aabb is None:
            return None
        return tuple(aabb[1][axis] - aabb[0][axis] for axis in range(3))

    def _aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    housing_pos = ctx.part_world_position(upper_housing)
    thigh_pos = ctx.part_world_position(thigh_link)
    shank_pos = ctx.part_world_position(shank_link)
    foot_pos = ctx.part_world_position(foot_section)

    ctx.check(
        "joint spacing keeps the chain long",
        housing_pos is not None
        and thigh_pos is not None
        and shank_pos is not None
        and foot_pos is not None
        and shank_pos[2] < thigh_pos[2] - 0.35
        and foot_pos[2] < shank_pos[2] - 0.33,
        details=(
            f"housing={housing_pos}, thigh={thigh_pos}, "
            f"shank={shank_pos}, foot={foot_pos}"
        ),
    )

    thigh_aabb = ctx.part_world_aabb(thigh_link)
    shank_aabb = ctx.part_world_aabb(shank_link)
    foot_aabb = ctx.part_world_aabb(foot_section)
    thigh_dims = _aabb_dims(thigh_aabb)
    shank_dims = _aabb_dims(shank_aabb)
    foot_dims = _aabb_dims(foot_aabb)

    ctx.check(
        "thigh link reads as slender",
        thigh_dims is not None
        and thigh_dims[2] > 0.43
        and thigh_dims[2] > 3.8 * max(thigh_dims[0], thigh_dims[1]),
        details=f"thigh_dims={thigh_dims}",
    )
    ctx.check(
        "shank link reads as slender",
        shank_dims is not None
        and shank_dims[2] > 0.40
        and shank_dims[2] > 4.0 * max(shank_dims[0], shank_dims[1]),
        details=f"shank_dims={shank_dims}",
    )
    ctx.check(
        "ankle foot stays compact",
        foot_dims is not None and foot_dims[0] < 0.28 and foot_dims[2] < 0.09,
        details=f"foot_dims={foot_dims}",
    )

    knee_flexed_foot = None
    with ctx.pose({knee_pitch: 1.2}):
        knee_flexed_foot = ctx.part_world_position(foot_section)
    ctx.check(
        "knee flex tucks the foot rearward and upward",
        foot_pos is not None
        and knee_flexed_foot is not None
        and knee_flexed_foot[0] < foot_pos[0] - 0.20
        and knee_flexed_foot[2] > foot_pos[2] + 0.20,
        details=f"rest_foot={foot_pos}, knee_flexed_foot={knee_flexed_foot}",
    )

    hip_swung_foot = None
    with ctx.pose({hip_pitch: 0.5}):
        hip_swung_foot = ctx.part_world_position(foot_section)
    ctx.check(
        "hip swings the whole chain forward",
        foot_pos is not None
        and hip_swung_foot is not None
        and hip_swung_foot[0] > foot_pos[0] + 0.25
        and hip_swung_foot[2] > foot_pos[2] + 0.05,
        details=f"rest_foot={foot_pos}, hip_swung_foot={hip_swung_foot}",
    )

    toe_rest_aabb = ctx.part_element_world_aabb(foot_section, elem="toe_pad")
    toe_lifted_aabb = None
    with ctx.pose({ankle_pitch: 0.4}):
        toe_lifted_aabb = ctx.part_element_world_aabb(foot_section, elem="toe_pad")
    toe_rest_center = _aabb_center(toe_rest_aabb)
    toe_lifted_center = _aabb_center(toe_lifted_aabb)
    ctx.check(
        "ankle dorsiflexion lifts the toe pad",
        toe_rest_center is not None
        and toe_lifted_center is not None
        and toe_lifted_center[2] > toe_rest_center[2] + 0.05,
        details=(
            f"toe_rest_center={toe_rest_center}, "
            f"toe_lifted_center={toe_lifted_center}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
