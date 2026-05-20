from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    ConeGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    section_loft,
)


ENGINE_LENGTH = 4.20
BODY_FRONT_X = -2.08
BODY_REAR_X = 2.12
FAN_X = -1.74
NOZZLE_HINGE_X = 2.02
NOZZLE_HINGE_RADIUS = 0.560
PETAL_COUNT = 8
BAY_DOOR_OPEN_ANGLE = math.radians(56.0)
PETAL_BASE_OPEN_ANGLE = math.radians(24.0)


def _save_mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def _merge_geometries(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _radial_pattern_about_x(base: MeshGeometry, count: int, *, angle_offset: float = 0.0) -> MeshGeometry:
    patterned = MeshGeometry()
    for index in range(count):
        patterned.merge(base.copy().rotate_x(angle_offset + index * math.tau / count))
    return patterned


def _rounded_yz_section(
    x_pos: float,
    width_y: float,
    height_z: float,
    radius: float,
) -> list[tuple[float, float, float]]:
    corner_radius = min(radius, width_y * 0.45, height_z * 0.45)
    points: list[tuple[float, float, float]] = []
    segments = 6
    half_y = width_y * 0.5
    half_z = height_z * 0.5
    centers = (
        (half_y - corner_radius, half_z - corner_radius, 0.0, math.pi / 2.0),
        (-half_y + corner_radius, half_z - corner_radius, math.pi / 2.0, math.pi),
        (-half_y + corner_radius, -half_z + corner_radius, math.pi, 3.0 * math.pi / 2.0),
        (half_y - corner_radius, -half_z + corner_radius, 3.0 * math.pi / 2.0, 2.0 * math.pi),
    )
    for cy, cz, start_angle, end_angle in centers:
        for step in range(segments):
            t = step / segments
            angle = start_angle + (end_angle - start_angle) * t
            points.append((x_pos, cy + corner_radius * math.cos(angle), cz + corner_radius * math.sin(angle)))
    return points


def _build_engine_shell_mesh() -> MeshGeometry:
    shell = LatheGeometry.from_shell_profiles(
        [
            (0.60, BODY_FRONT_X),
            (0.68, -1.94),
            (0.70, -1.36),
            (0.69, -0.40),
            (0.66, 0.62),
            (0.61, 1.34),
            (0.55, 1.78),
            (0.48, BODY_REAR_X),
        ],
        [
            (0.54, BODY_FRONT_X - 0.05),
            (0.60, -1.94),
            (0.62, -1.34),
            (0.61, -0.36),
            (0.58, 0.68),
            (0.53, 1.34),
            (0.45, 1.80),
            (0.34, BODY_REAR_X + 0.03),
        ],
        segments=88,
        start_cap="round",
        end_cap="flat",
        lip_samples=10,
    ).rotate_y(math.pi / 2.0)

    nozzle_ring = (
        LatheGeometry.from_shell_profiles(
            [(0.547, NOZZLE_HINGE_X - 0.035), (0.547, NOZZLE_HINGE_X + 0.035)],
            [(0.475, NOZZLE_HINGE_X - 0.035), (0.475, NOZZLE_HINGE_X + 0.035)],
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ).rotate_y(math.pi / 2.0)
    )
    bearing_spindle = (
        CylinderGeometry(radius=0.11, height=0.10, radial_segments=28)
        .rotate_y(math.pi / 2.0)
        .translate(-1.505, 0.0, 0.0)
    )
    base_strut = BoxGeometry((0.10, 0.58, 0.08)).translate(-1.505, 0.39, 0.0)
    struts = _radial_pattern_about_x(base_strut, 4)
    shell.merge(nozzle_ring)
    shell.merge(bearing_spindle)
    shell.merge(struts)
    return shell


def _build_bay_housing_mesh() -> MeshGeometry:
    sections = [
        _rounded_yz_section(0.88, 0.18, 0.34, 0.03),
        _rounded_yz_section(1.24, 0.24, 0.46, 0.04),
        _rounded_yz_section(1.60, 0.18, 0.34, 0.03),
    ]
    housing_skin = repair_loft(section_loft(sections))
    housing_skin.translate(0.0, 0.60, -0.02)

    bay_backplate = BoxGeometry((0.68, 0.08, 0.44)).translate(1.24, 0.56, -0.02)
    bay_top = BoxGeometry((0.70, 0.20, 0.04)).translate(1.24, 0.64, 0.20)
    bay_bottom = BoxGeometry((0.70, 0.20, 0.04)).translate(1.24, 0.64, -0.24)
    bay_front_frame = BoxGeometry((0.04, 0.20, 0.36)).translate(0.90, 0.64, -0.02)
    bay_rear_frame = BoxGeometry((0.04, 0.20, 0.36)).translate(1.58, 0.64, -0.02)
    hinge_upper_boss = BoxGeometry((0.04, 0.06, 0.08)).translate(0.90, 0.72, 0.13)
    hinge_mid_boss = BoxGeometry((0.04, 0.06, 0.08)).translate(0.90, 0.72, 0.0)
    hinge_lower_boss = BoxGeometry((0.04, 0.06, 0.08)).translate(0.90, 0.72, -0.13)

    return _merge_geometries(
        [
            housing_skin,
            bay_backplate,
            bay_top,
            bay_bottom,
            bay_front_frame,
            bay_rear_frame,
            hinge_upper_boss,
            hinge_mid_boss,
            hinge_lower_boss,
        ]
    )


def _build_bay_equipment_mesh() -> MeshGeometry:
    controller_box = BoxGeometry((0.22, 0.17, 0.16)).translate(1.06, 0.63, 0.03)
    relay_box = BoxGeometry((0.18, 0.15, 0.13)).translate(1.36, 0.63, -0.08)
    cylindrical_unit = (
        CylinderGeometry(radius=0.075, height=0.22, radial_segments=28)
        .rotate_y(math.pi / 2.0)
        .translate(1.30, 0.62, -0.17)
    )
    upper_manifold = BoxGeometry((0.46, 0.05, 0.05)).translate(1.24, 0.60, 0.19)
    lower_manifold = BoxGeometry((0.42, 0.05, 0.05)).translate(1.22, 0.60, -0.22)
    forward_rib = BoxGeometry((0.06, 0.14, 0.22)).translate(0.98, 0.60, -0.03)
    return _merge_geometries(
        [
            controller_box,
            relay_box,
            cylindrical_unit,
            upper_manifold,
            lower_manifold,
            forward_rib,
        ]
    )


def _fan_blade_section(
    radius_y: float,
    x_center: float,
    chord: float,
    thickness_z: float,
    lean_z: float,
) -> list[tuple[float, float, float]]:
    half_t = thickness_z * 0.5
    return [
        (x_center - 0.52 * chord, radius_y, lean_z - 0.95 * half_t),
        (x_center + 0.18 * chord, radius_y, lean_z - 0.22 * half_t),
        (x_center + 0.34 * chord, radius_y, lean_z + 0.95 * half_t),
        (x_center - 0.28 * chord, radius_y, lean_z + 0.30 * half_t),
    ]


def _build_fan_rotor_mesh() -> MeshGeometry:
    spinner = (
        ConeGeometry(radius=0.185, height=0.34, radial_segments=48)
        .rotate_y(-math.pi / 2.0)
        .translate(-0.07, 0.0, 0.0)
    )
    hub = (
        CylinderGeometry(radius=0.23, height=0.26, radial_segments=48)
        .rotate_y(math.pi / 2.0)
        .translate(0.05, 0.0, 0.0)
    )
    hub_collar = (
        CylinderGeometry(radius=0.26, height=0.05, radial_segments=40)
        .rotate_y(math.pi / 2.0)
        .translate(0.16, 0.0, 0.0)
    )
    blade = repair_loft(
        section_loft(
            [
                _fan_blade_section(0.17, -0.03, 0.24, 0.045, -0.020),
                _fan_blade_section(0.37, -0.02, 0.20, 0.036, -0.004),
                _fan_blade_section(0.57, -0.01, 0.14, 0.022, 0.012),
            ]
        )
    )
    fan_blades = _radial_pattern_about_x(blade, 16, angle_offset=math.pi / 16.0)
    return _merge_geometries([spinner, hub, hub_collar, fan_blades])


def _build_door_mesh() -> MeshGeometry:
    door_panel = BoxGeometry((0.68, 0.020, 0.42)).translate(0.34, 0.010, 0.0)
    stiffener = BoxGeometry((0.56, 0.016, 0.08)).translate(0.36, 0.016, -0.13)
    latch_block = BoxGeometry((0.07, 0.025, 0.10)).translate(0.63, 0.014, 0.0)
    hinge_upper = CylinderGeometry(radius=0.015, height=0.10, radial_segments=24).translate(0.0, 0.0, 0.13)
    hinge_mid = CylinderGeometry(radius=0.015, height=0.10, radial_segments=24).translate(0.0, 0.0, 0.0)
    hinge_lower = CylinderGeometry(radius=0.015, height=0.10, radial_segments=24).translate(0.0, 0.0, -0.13)
    door = _merge_geometries([door_panel, stiffener, latch_block, hinge_upper, hinge_mid, hinge_lower])
    door.rotate_z(BAY_DOOR_OPEN_ANGLE)
    return door


def _build_petal_mesh() -> MeshGeometry:
    petal_skin = repair_loft(
        section_loft(
            [
                [
                    (0.00, -0.008, -0.088),
                    (0.00, 0.008, -0.066),
                    (0.00, 0.008, 0.066),
                    (0.00, -0.008, 0.088),
                ],
                [
                    (-0.16, 0.004, -0.080),
                    (-0.16, 0.014, -0.060),
                    (-0.16, 0.014, 0.060),
                    (-0.16, 0.004, 0.080),
                ],
                [
                    (-0.30, 0.010, -0.058),
                    (-0.30, 0.018, -0.040),
                    (-0.30, 0.018, 0.040),
                    (-0.30, 0.010, 0.058),
                ],
            ]
        )
    )
    petal_skin.rotate_z(-PETAL_BASE_OPEN_ANGLE)
    return petal_skin


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="military_turbofan")

    body_gray = model.material("body_gray", rgba=(0.43, 0.45, 0.46, 1.0))
    equipment_gray = model.material("equipment_gray", rgba=(0.20, 0.22, 0.24, 1.0))
    fan_metal = model.material("fan_metal", rgba=(0.73, 0.76, 0.80, 1.0))
    nozzle_metal = model.material("nozzle_metal", rgba=(0.42, 0.36, 0.31, 1.0))
    heated_titanium = model.material("heated_titanium", rgba=(0.55, 0.47, 0.38, 1.0))

    engine_body = model.part("engine_body")
    engine_body.visual(
        _save_mesh("engine_shell", _build_engine_shell_mesh()),
        material=body_gray,
        name="engine_shell",
    )
    engine_body.visual(
        _save_mesh("bay_housing", _build_bay_housing_mesh()),
        material=body_gray,
        name="bay_housing_shell",
    )
    engine_body.visual(
        _save_mesh("bay_equipment", _build_bay_equipment_mesh()),
        material=equipment_gray,
        name="bay_equipment",
    )
    for index in range(PETAL_COUNT):
        theta = index * math.tau / PETAL_COUNT
        mount_radius = 0.537
        engine_body.visual(
            Box((0.08, 0.020, 0.18)),
            origin=Origin(
                xyz=(
                    NOZZLE_HINGE_X,
                    mount_radius * math.cos(theta),
                    mount_radius * math.sin(theta),
                ),
                rpy=(theta, 0.0, 0.0),
            ),
            material=nozzle_metal,
            name=f"petal_mount_{index:02d}",
        )
    engine_body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.72, length=ENGINE_LENGTH),
        mass=1180.0,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    fan_rotor = model.part("fan_rotor")
    fan_rotor.visual(
        _save_mesh("fan_rotor", _build_fan_rotor_mesh()),
        material=fan_metal,
        name="fan_rotor_mesh",
    )
    fan_rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.58, length=0.32),
        mass=92.0,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    bay_door = model.part("bay_door")
    bay_door.visual(
        _save_mesh("bay_door", _build_door_mesh()),
        material=body_gray,
        name="door_panel",
    )
    bay_door.inertial = Inertial.from_geometry(
        Box((0.72, 0.09, 0.48)),
        mass=12.0,
        origin=Origin(xyz=(0.34, 0.02, 0.0)),
    )

    model.articulation(
        "body_to_fan",
        ArticulationType.CONTINUOUS,
        parent=engine_body,
        child=fan_rotor,
        origin=Origin(xyz=(FAN_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=45.0),
    )

    model.articulation(
        "body_to_bay_door",
        ArticulationType.REVOLUTE,
        parent=engine_body,
        child=bay_door,
        origin=Origin(xyz=(0.90, 0.76, -0.02)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=-BAY_DOOR_OPEN_ANGLE,
            upper=math.radians(18.0),
        ),
    )

    for index in range(PETAL_COUNT):
        theta = index * math.tau / PETAL_COUNT
        petal = model.part(f"nozzle_petal_{index:02d}")
        petal.visual(
            _save_mesh(f"nozzle_petal_{index:02d}", _build_petal_mesh()),
            material=heated_titanium if index % 2 == 0 else nozzle_metal,
            name="petal_skin",
        )
        petal.visual(
            Cylinder(radius=0.013, length=0.15),
            material=nozzle_metal,
            name="hinge_barrel",
        )
        petal.inertial = Inertial.from_geometry(
            Box((0.60, 0.08, 0.22)),
            mass=4.5,
            origin=Origin(xyz=(-0.28, 0.02, 0.0)),
        )
        model.articulation(
            f"body_to_nozzle_petal_{index:02d}",
            ArticulationType.REVOLUTE,
            parent=engine_body,
            child=petal,
            origin=Origin(
                xyz=(
                    NOZZLE_HINGE_X,
                    NOZZLE_HINGE_RADIUS * math.cos(theta),
                    NOZZLE_HINGE_RADIUS * math.sin(theta),
                ),
                rpy=(theta, 0.0, 0.0),
            ),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=1.5,
                lower=math.radians(-20.0),
                upper=math.radians(22.0),
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    engine_body = object_model.get_part("engine_body")
    fan_rotor = object_model.get_part("fan_rotor")
    bay_door = object_model.get_part("bay_door")
    bay_hinge = object_model.get_articulation("body_to_bay_door")
    fan_joint = object_model.get_articulation("body_to_fan")
    petal_joint = object_model.get_articulation("body_to_nozzle_petal_00")
    petal = object_model.get_part("nozzle_petal_00")

    ctx.check(
        "fan uses continuous axial spin",
        fan_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(fan_joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={fan_joint.articulation_type}, axis={fan_joint.axis}",
    )
    ctx.check(
        "bay door uses side hinge",
        bay_hinge.articulation_type == ArticulationType.REVOLUTE and tuple(bay_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"type={bay_hinge.articulation_type}, axis={bay_hinge.axis}",
    )
    ctx.check(
        "nozzle petals hinge tangentially",
        petal_joint.articulation_type == ArticulationType.REVOLUTE and tuple(petal_joint.axis) == (0.0, 0.0, -1.0),
        details=f"type={petal_joint.articulation_type}, axis={petal_joint.axis}",
    )
    ctx.expect_contact(
        petal,
        engine_body,
        elem_a="hinge_barrel",
        elem_b="petal_mount_00",
        contact_tol=0.0002,
        name="petal hinge barrel is supported by the nozzle mount",
    )

    ctx.expect_overlap(
        fan_rotor,
        engine_body,
        axes="yz",
        elem_a="fan_rotor_mesh",
        elem_b="engine_shell",
        min_overlap=0.95,
        name="front fan stays centered in the inlet",
    )

    with ctx.pose({bay_hinge: -BAY_DOOR_OPEN_ANGLE}):
        ctx.expect_overlap(
            bay_door,
            engine_body,
            axes="xz",
            elem_a="door_panel",
            elem_b="bay_housing_shell",
            min_overlap=0.30,
            name="bay door covers the housing opening when swung shut",
        )

    closed_door_aabb = None
    with ctx.pose({bay_hinge: -BAY_DOOR_OPEN_ANGLE}):
        closed_door_aabb = ctx.part_element_world_aabb(bay_door, elem="door_panel")
    open_door_aabb = ctx.part_element_world_aabb(bay_door, elem="door_panel")
    door_opens_outward = (
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][1] > closed_door_aabb[1][1] + 0.18
    )
    ctx.check(
        "bay door opens outward from the housing",
        door_opens_outward,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    more_closed_petal_aabb = None
    more_open_petal_aabb = None
    with ctx.pose({petal_joint: math.radians(-12.0)}):
        more_closed_petal_aabb = ctx.part_element_world_aabb(petal, elem="petal_skin")
    with ctx.pose({petal_joint: math.radians(12.0)}):
        more_open_petal_aabb = ctx.part_element_world_aabb(petal, elem="petal_skin")
    petals_open_outward = (
        more_closed_petal_aabb is not None
        and more_open_petal_aabb is not None
        and more_open_petal_aabb[1][1] > more_closed_petal_aabb[1][1] + 0.05
    )
    ctx.check(
        "nozzle petals open wider with positive motion",
        petals_open_outward,
        details=f"closed={more_closed_petal_aabb}, open={more_open_petal_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
