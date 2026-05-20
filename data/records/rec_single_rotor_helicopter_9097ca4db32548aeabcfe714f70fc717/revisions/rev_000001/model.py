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
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _yz_section(
    *,
    x: float,
    width: float,
    height: float,
    radius: float,
    z_center: float = 0.0,
    corner_segments: int = 10,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z + z_center)
        for z, y in rounded_rect_profile(height, width, radius, corner_segments=corner_segments)
    ]


def _segment_visual(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str,
) -> None:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=(
                0.5 * (start[0] + end[0]),
                0.5 * (start[1] + end[1]),
                0.5 * (start[2] + end[2]),
            ),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def _merged_mesh(*geometries) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="medical_transport_helicopter")

    rescue_white = model.material("rescue_white", rgba=(0.92, 0.93, 0.95, 1.0))
    rescue_red = model.material("rescue_red", rgba=(0.74, 0.15, 0.12, 1.0))
    window_tint = model.material("window_tint", rgba=(0.18, 0.24, 0.30, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.14, 0.15, 0.16, 1.0))
    steel = model.material("steel", rgba=(0.60, 0.63, 0.67, 1.0))

    airframe = model.part("airframe")
    airframe.inertial = Inertial.from_geometry(
        Box((10.8, 2.8, 3.4)),
        mass=2600.0,
        origin=Origin(xyz=(-1.8, 0.0, 0.30)),
    )

    fuselage_sections = [
        _yz_section(x=2.25, width=0.18, height=0.28, radius=0.06, z_center=0.10),
        _yz_section(x=1.55, width=1.40, height=1.65, radius=0.22, z_center=0.18),
        _yz_section(x=0.30, width=2.20, height=2.45, radius=0.28, z_center=0.28),
        _yz_section(x=-1.20, width=2.28, height=2.56, radius=0.30, z_center=0.28),
        _yz_section(x=-2.35, width=1.62, height=1.92, radius=0.22, z_center=0.42),
    ]
    fuselage_mesh = mesh_from_geometry(section_loft(fuselage_sections), "fuselage_shell")
    airframe.visual(fuselage_mesh, material=rescue_white, name="fuselage_shell")

    airframe.visual(
        Box((1.55, 1.70, 0.10)),
        origin=Origin(xyz=(0.30, 0.0, -0.88)),
        material=dark_trim,
        name="belly_keel",
    )
    airframe.visual(
        Box((0.92, 0.12, 0.84)),
        origin=Origin(xyz=(-7.35, 0.0, 1.16)),
        material=rescue_white,
        name="tail_fin",
    )
    airframe.visual(
        Box((1.18, 1.35, 0.08)),
        origin=Origin(xyz=(-6.55, 0.0, 0.64)),
        material=rescue_white,
        name="tail_stabilizer",
    )
    airframe.visual(
        Cylinder(radius=0.22, length=5.50),
        origin=Origin(xyz=(-5.07, 0.0, 0.58), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rescue_white,
        name="tail_boom",
    )
    airframe.visual(
        Cylinder(radius=0.12, length=0.90),
        origin=Origin(xyz=(-7.72, 0.0, 1.06), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rescue_white,
        name="tail_rotor_pylon",
    )
    airframe.visual(
        Cylinder(radius=0.16, length=0.72),
        origin=Origin(xyz=(-0.15, 0.0, 1.55)),
        material=dark_trim,
        name="mast_fairing",
    )
    airframe.visual(
        Box((3.40, 0.12, 0.10)),
        origin=Origin(xyz=(-0.35, 1.11, 0.93)),
        material=rescue_red,
        name="patient_rail",
    )
    airframe.visual(
        Box((3.20, 0.10, 0.14)),
        origin=Origin(xyz=(-0.55, 1.05, -0.62)),
        material=dark_trim,
        name="patient_sill",
    )
    airframe.visual(
        Box((0.14, 0.18, 0.24)),
        origin=Origin(xyz=(0.85, 1.03, 0.83)),
        material=dark_trim,
        name="rail_bracket_0",
    )
    airframe.visual(
        Box((0.14, 0.18, 0.24)),
        origin=Origin(xyz=(-1.15, 1.03, 0.83)),
        material=dark_trim,
        name="rail_bracket_1",
    )
    airframe.visual(
        Box((1.52, 0.19, 1.18)),
        origin=Origin(xyz=(-0.15, -1.08, 0.05)),
        material=rescue_red,
        name="bay_frame",
    )
    airframe.visual(
        Box((1.25, 1.60, 0.58)),
        origin=Origin(xyz=(1.18, 0.0, 0.34)),
        material=window_tint,
        name="cockpit_glazing",
    )

    airframe.visual(
        Cylinder(radius=0.07, length=5.55),
        origin=Origin(xyz=(-0.10, 1.18, -1.20), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="skid_0",
    )
    airframe.visual(
        Cylinder(radius=0.07, length=5.55),
        origin=Origin(xyz=(-0.10, -1.18, -1.20), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="skid_1",
    )
    airframe.visual(
        Cylinder(radius=0.075, length=2.02),
        origin=Origin(xyz=(1.05, 0.0, -0.72), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="cross_tube_0",
    )
    airframe.visual(
        Cylinder(radius=0.075, length=2.02),
        origin=Origin(xyz=(-0.95, 0.0, -0.70), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="cross_tube_1",
    )
    _segment_visual(
        airframe,
        (1.05, 0.93, -0.72),
        (1.05, 1.18, -1.18),
        radius=0.055,
        material=steel,
        name="gear_strut_0",
    )
    _segment_visual(
        airframe,
        (1.05, -0.93, -0.72),
        (1.05, -1.18, -1.18),
        radius=0.055,
        material=steel,
        name="gear_strut_1",
    )
    _segment_visual(
        airframe,
        (-0.95, 0.93, -0.70),
        (-0.95, 1.18, -1.18),
        radius=0.055,
        material=steel,
        name="gear_strut_2",
    )
    _segment_visual(
        airframe,
        (-0.95, -0.93, -0.70),
        (-0.95, -1.18, -1.18),
        radius=0.055,
        material=steel,
        name="gear_strut_3",
    )

    patient_door = model.part("patient_door")
    patient_door.inertial = Inertial.from_geometry(
        Box((2.10, 0.14, 1.58)),
        mass=90.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    patient_door.visual(
        Box((2.02, 0.10, 1.46)),
        material=rescue_white,
        name="door_panel",
    )
    patient_door.visual(
        Box((0.82, 0.02, 0.48)),
        origin=Origin(xyz=(0.18, 0.03, 0.28)),
        material=window_tint,
        name="door_window",
    )
    patient_door.visual(
        Box((1.74, 0.05, 0.08)),
        origin=Origin(xyz=(0.0, -0.035, 0.82)),
        material=dark_trim,
        name="trolley_beam",
    )
    patient_door.visual(
        Box((0.10, 0.05, 0.24)),
        origin=Origin(xyz=(0.68, -0.005, 0.66)),
        material=dark_trim,
        name="hanger_arm_0",
    )
    patient_door.visual(
        Box((0.10, 0.05, 0.24)),
        origin=Origin(xyz=(-0.68, -0.005, 0.66)),
        material=dark_trim,
        name="hanger_arm_1",
    )
    patient_door.visual(
        Cylinder(radius=0.03, length=0.14),
        origin=Origin(xyz=(0.72, 0.06, -0.35), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="door_handle",
    )

    bay_hatch = model.part("bay_hatch")
    bay_hatch.inertial = Inertial.from_geometry(
        Box((1.42, 0.12, 1.16)),
        mass=42.0,
        origin=Origin(xyz=(-0.71, -0.04, 0.0)),
    )
    bay_hatch.visual(
        Box((1.38, 0.08, 1.10)),
        origin=Origin(xyz=(-0.69, -0.04, 0.0)),
        material=rescue_white,
        name="hatch_panel",
    )
    bay_hatch.visual(
        Box((1.18, 0.03, 0.88)),
        origin=Origin(xyz=(-0.73, -0.06, 0.0)),
        material=rescue_red,
        name="hatch_inner",
    )
    bay_hatch.visual(
        Cylinder(radius=0.026, length=0.12),
        origin=Origin(xyz=(-0.92, -0.06, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="hatch_handle",
    )

    main_rotor = model.part("main_rotor")
    main_rotor.inertial = Inertial.from_geometry(
        Box((10.8, 10.8, 0.32)),
        mass=120.0,
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
    )
    main_rotor.visual(
        Cylinder(radius=0.30, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=dark_trim,
        name="hub",
    )
    main_rotor.visual(
        Cylinder(radius=0.10, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=steel,
        name="mast_stub",
    )
    main_rotor.visual(
        Box((10.60, 0.24, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=dark_trim,
        name="blade_pair_0",
    )
    main_rotor.visual(
        Box((0.24, 10.60, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=dark_trim,
        name="blade_pair_1",
    )
    main_rotor.visual(
        Box((0.70, 0.40, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=steel,
        name="grip_cross",
    )

    tail_rotor = model.part("tail_rotor")
    tail_rotor.inertial = Inertial.from_geometry(
        Box((1.62, 0.24, 1.62)),
        mass=18.0,
        origin=Origin(),
    )
    tail_rotor.visual(
        Cylinder(radius=0.11, length=0.22),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="tail_hub",
    )
    tail_rotor.visual(
        Box((1.48, 0.05, 0.14)),
        material=dark_trim,
        name="tail_blade_pair_0",
    )
    tail_rotor.visual(
        Box((0.14, 0.05, 1.48)),
        material=dark_trim,
        name="tail_blade_pair_1",
    )

    model.articulation(
        "door_slide",
        ArticulationType.PRISMATIC,
        parent=airframe,
        child=patient_door,
        origin=Origin(xyz=(0.25, 1.23, 0.10)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=150.0, velocity=1.2, lower=0.0, upper=1.35),
    )
    model.articulation(
        "bay_hinge",
        ArticulationType.REVOLUTE,
        parent=airframe,
        child=bay_hatch,
        origin=Origin(xyz=(0.50, -1.175, 0.04)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.2, lower=0.0, upper=1.45),
    )
    model.articulation(
        "mast_spin",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=main_rotor,
        origin=Origin(xyz=(-0.15, 0.0, 2.00)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=45.0),
    )
    model.articulation(
        "tail_spin",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=tail_rotor,
        origin=Origin(xyz=(-7.72, 0.56, 1.06)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=85.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    door = object_model.get_part("patient_door")
    airframe = object_model.get_part("airframe")
    hatch = object_model.get_part("bay_hatch")
    door_slide = object_model.get_articulation("door_slide")
    bay_hinge = object_model.get_articulation("bay_hinge")

    ctx.expect_contact(
        door,
        airframe,
        elem_a="trolley_beam",
        elem_b="patient_rail",
        name="patient door trolley stays on the side rail at rest",
    )
    ctx.expect_overlap(
        door,
        airframe,
        axes="x",
        elem_a="trolley_beam",
        elem_b="patient_rail",
        min_overlap=1.70,
        name="patient door trolley remains engaged along the rail at rest",
    )

    rest_door_pos = ctx.part_world_position(door)
    with ctx.pose({door_slide: door_slide.motion_limits.upper}):
        ctx.expect_contact(
            door,
            airframe,
            elem_a="trolley_beam",
            elem_b="patient_rail",
            name="patient door trolley stays on the side rail when open",
        )
        ctx.expect_overlap(
            door,
            airframe,
            axes="x",
            elem_a="trolley_beam",
            elem_b="patient_rail",
            min_overlap=1.70,
            name="patient door trolley remains retained on the rail when open",
        )
        open_door_pos = ctx.part_world_position(door)

    ctx.check(
        "patient door slides rearward",
        rest_door_pos is not None
        and open_door_pos is not None
        and open_door_pos[0] < rest_door_pos[0] - 1.0,
        details=f"rest={rest_door_pos}, open={open_door_pos}",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        return tuple(0.5 * (lo + hi) for lo, hi in zip(aabb[0], aabb[1]))

    rest_hatch_panel = ctx.part_element_world_aabb(hatch, elem="hatch_panel")
    with ctx.pose({bay_hinge: bay_hinge.motion_limits.upper}):
        open_hatch_panel = ctx.part_element_world_aabb(hatch, elem="hatch_panel")

    rest_hatch_center = _aabb_center(rest_hatch_panel)
    open_hatch_center = _aabb_center(open_hatch_panel)
    ctx.check(
        "equipment hatch swings outward",
        rest_hatch_center is not None
        and open_hatch_center is not None
        and open_hatch_center[1] < rest_hatch_center[1] - 0.45,
        details=f"rest={rest_hatch_center}, open={open_hatch_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
