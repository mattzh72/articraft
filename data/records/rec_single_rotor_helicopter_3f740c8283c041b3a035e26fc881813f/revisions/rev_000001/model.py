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
    Cylinder,
    CylinderGeometry,
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


def _merge_geometries(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _yz_section(
    x_pos: float,
    width: float,
    height: float,
    z_center: float,
    *,
    corner_radius: float,
    corner_segments: int = 6,
) -> list[tuple[float, float, float]]:
    return [
        (x_pos, y_pos, z_pos + z_center)
        for y_pos, z_pos in rounded_rect_profile(
            width,
            height,
            min(corner_radius, 0.45 * min(width, height)),
            corner_segments=corner_segments,
        )
    ]


def _radial_pattern(base_geometry: MeshGeometry, count: int, *, axis: str) -> MeshGeometry:
    patterned = MeshGeometry()
    for index in range(count):
        angle = index * math.tau / count
        rotated = base_geometry.copy()
        if axis == "z":
            rotated.rotate_z(angle)
        elif axis == "y":
            rotated.rotate_y(angle)
        else:
            raise ValueError(f"Unsupported radial axis: {axis}")
        patterned.merge(rotated)
    return patterned


def _blade_section(
    span_pos: float,
    chord: float,
    thickness: float,
    *,
    chord_offset: float = 0.0,
    thickness_offset: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (span_pos, chord_offset - 0.48 * chord, thickness_offset + 0.00 * thickness),
        (span_pos, chord_offset - 0.16 * chord, thickness_offset + 0.42 * thickness),
        (span_pos, chord_offset + 0.22 * chord, thickness_offset + 0.34 * thickness),
        (span_pos, chord_offset + 0.50 * chord, thickness_offset + 0.08 * thickness),
        (span_pos, chord_offset + 0.38 * chord, thickness_offset - 0.16 * thickness),
        (span_pos, chord_offset + 0.04 * chord, thickness_offset - 0.30 * thickness),
        (span_pos, chord_offset - 0.28 * chord, thickness_offset - 0.20 * thickness),
        (span_pos, chord_offset - 0.46 * chord, thickness_offset - 0.04 * thickness),
    ]


def _main_rotor_geometry() -> MeshGeometry:
    hub = _merge_geometries(
        CylinderGeometry(radius=0.28, height=0.18, radial_segments=28).translate(0.0, 0.0, 0.09),
        CylinderGeometry(radius=0.16, height=0.12, radial_segments=24).translate(0.0, 0.0, 0.21),
        BoxGeometry((0.52, 0.22, 0.10)).translate(0.0, 0.0, 0.14),
        BoxGeometry((0.22, 0.52, 0.10)).translate(0.0, 0.0, 0.14),
    )

    blade = section_loft(
        [
            _blade_section(0.18, 0.42, 0.08),
            _blade_section(2.45, 0.32, 0.05, chord_offset=0.05),
            _blade_section(4.55, 0.24, 0.035, chord_offset=0.09),
            _blade_section(6.05, 0.18, 0.025, chord_offset=0.14),
        ]
    )
    blade.translate(0.0, 0.0, 0.08)
    blade.merge(BoxGeometry((0.26, 0.24, 0.12)).translate(0.16, 0.0, 0.11))

    return _merge_geometries(hub, _radial_pattern(blade, 4, axis="z"))


def _tail_rotor_geometry() -> MeshGeometry:
    hub = _merge_geometries(
        CylinderGeometry(radius=0.12, height=0.18, radial_segments=24)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, 0.0, 0.0),
        BoxGeometry((0.22, 0.10, 0.22)),
    )

    blade = section_loft(
        [
            _blade_section(0.08, 0.16, 0.035),
            _blade_section(0.48, 0.12, 0.025, chord_offset=0.02),
            _blade_section(0.86, 0.08, 0.018, chord_offset=0.04),
        ]
    )
    blade.rotate_x(-math.pi / 2.0)
    blade.merge(BoxGeometry((0.16, 0.10, 0.08)).rotate_x(-math.pi / 2.0).translate(0.06, 0.0, 0.0))

    return _merge_geometries(hub, _radial_pattern(blade, 4, axis="y"))


def _aabb_center(aabb):
    if aabb is None:
        return None
    minimum, maximum = aabb
    return tuple((minimum[index] + maximum[index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="transport_helicopter")

    olive = model.material("olive", rgba=(0.36, 0.40, 0.28, 1.0))
    dark_olive = model.material("dark_olive", rgba=(0.23, 0.27, 0.19, 1.0))
    glass = model.material("glass", rgba=(0.12, 0.18, 0.24, 0.70))
    rotor_gray = model.material("rotor_gray", rgba=(0.22, 0.24, 0.26, 1.0))
    steel = model.material("steel", rgba=(0.55, 0.58, 0.60, 1.0))
    tire = model.material("tire", rgba=(0.08, 0.08, 0.08, 1.0))

    body = model.part("body")

    fuselage_geom = section_loft(
        [
            _yz_section(-3.35, 0.28, 0.34, 1.42, corner_radius=0.08),
            _yz_section(-2.70, 1.18, 1.12, 1.60, corner_radius=0.20),
            _yz_section(-1.95, 1.82, 1.58, 1.64, corner_radius=0.24),
            _yz_section(-0.75, 2.20, 2.00, 1.66, corner_radius=0.28),
            _yz_section(0.65, 2.34, 2.10, 1.66, corner_radius=0.28),
            _yz_section(1.85, 2.14, 1.78, 1.56, corner_radius=0.24),
            _yz_section(2.75, 1.02, 0.92, 1.52, corner_radius=0.18),
        ]
    )
    body.visual(
        mesh_from_geometry(fuselage_geom, "fuselage_shell"),
        material=olive,
        name="fuselage_shell",
    )

    tail_boom_geom = section_loft(
        [
            _yz_section(2.55, 1.00, 0.92, 1.52, corner_radius=0.16),
            _yz_section(4.30, 0.58, 0.52, 1.82, corner_radius=0.10),
            _yz_section(6.10, 0.34, 0.32, 2.08, corner_radius=0.07),
            _yz_section(7.35, 0.22, 0.18, 2.26, corner_radius=0.05),
        ]
    )
    body.visual(
        mesh_from_geometry(tail_boom_geom, "tail_boom"),
        material=olive,
        name="tail_boom",
    )

    fin_geom = section_loft(
        [
            _yz_section(6.45, 0.18, 0.70, 2.48, corner_radius=0.03),
            _yz_section(7.02, 0.14, 1.55, 2.86, corner_radius=0.03),
            _yz_section(7.55, 0.08, 0.98, 3.10, corner_radius=0.02),
        ]
    )
    body.visual(
        mesh_from_geometry(fin_geom, "vertical_fin"),
        material=olive,
        name="vertical_fin",
    )

    body.visual(
        Box((0.70, 1.85, 0.08)),
        origin=Origin(xyz=(6.30, 0.0, 1.95)),
        material=olive,
        name="tailplane",
    )

    body.visual(
        Box((1.55, 1.05, 0.42)),
        origin=Origin(xyz=(-0.05, 0.0, 2.86)),
        material=dark_olive,
        name="mast_fairing",
    )
    engine_pod_geom = section_loft(
        [
            _yz_section(-0.78, 0.22, 0.24, 0.0, corner_radius=0.05),
            _yz_section(-0.26, 0.50, 0.44, 0.0, corner_radius=0.10),
            _yz_section(0.34, 0.56, 0.48, 0.0, corner_radius=0.12),
            _yz_section(0.78, 0.30, 0.26, 0.0, corner_radius=0.06),
        ]
    )
    body.visual(
        mesh_from_geometry(engine_pod_geom, "engine_pod_0"),
        origin=Origin(xyz=(-0.20, 0.56, 2.98)),
        material=olive,
        name="engine_pod_0",
    )
    body.visual(
        mesh_from_geometry(engine_pod_geom, "engine_pod_1"),
        origin=Origin(xyz=(-0.20, -0.56, 2.98)),
        material=olive,
        name="engine_pod_1",
    )
    body.visual(
        Cylinder(radius=0.12, length=0.44),
        origin=Origin(xyz=(0.0, 0.0, 3.28)),
        material=steel,
        name="mast",
    )

    body.visual(
        Cylinder(radius=0.07, length=0.60),
        origin=Origin(
            xyz=(7.28, -0.22, 2.56),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="tail_rotor_support",
    )

    body.visual(
        Box((3.10, 0.08, 0.08)),
        origin=Origin(xyz=(-0.05, -1.34, 2.16)),
        material=steel,
        name="door_rail",
    )
    for x_pos, index in ((-1.58, 0), (1.33, 1)):
        body.visual(
            Box((0.04, 0.30, 0.18)),
            origin=Origin(xyz=(x_pos, -1.19, 2.09)),
            material=steel,
            name=f"rail_bracket_{index}",
        )
    body.visual(
        Box((0.80, 0.19, 1.18)),
        origin=Origin(xyz=(-2.34, 0.97, 1.60)),
        material=dark_olive,
        name="left_door_frame",
    )
    body.visual(
        Box((0.80, 0.19, 1.18)),
        origin=Origin(xyz=(-2.34, -0.97, 1.60)),
        material=dark_olive,
        name="right_door_frame",
    )
    body.visual(
        Cylinder(radius=0.035, length=1.18),
        origin=Origin(xyz=(-2.72, 1.03, 1.60)),
        material=steel,
        name="left_hinge_post",
    )
    body.visual(
        Cylinder(radius=0.035, length=1.18),
        origin=Origin(xyz=(-2.72, -1.03, 1.60)),
        material=steel,
        name="right_hinge_post",
    )

    for side_sign, index in ((1.0, 0), (-1.0, 1)):
        y_body = side_sign * 1.18
        y_wheel = side_sign * 1.46
        body.visual(
            Box((1.35, 0.34, 0.34)),
            origin=Origin(xyz=(0.15, y_body, 0.86)),
            material=olive,
            name=f"sponson_{index}",
        )
        body.visual(
            Box((0.09, 0.09, 0.62)),
            origin=Origin(xyz=(0.25, side_sign * 1.34, 0.61)),
            material=steel,
            name=f"main_strut_{index}",
        )
        body.visual(
            Box((0.16, 0.18, 0.10)),
            origin=Origin(xyz=(0.25, side_sign * 1.38, 0.30)),
            material=steel,
            name=f"main_axle_{index}",
        )
        body.visual(
            Cylinder(radius=0.26, length=0.16),
            origin=Origin(
                xyz=(0.25, y_wheel, 0.26),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=tire,
            name=f"main_wheel_{index}",
        )

    body.visual(
        Box((0.12, 0.12, 0.72)),
        origin=Origin(xyz=(-2.18, 0.0, 0.66)),
        material=steel,
        name="nose_strut",
    )
    body.visual(
        Box((0.14, 0.20, 0.18)),
        origin=Origin(xyz=(-2.15, 0.0, 0.34)),
        material=steel,
        name="nose_fork",
    )
    body.visual(
        Cylinder(radius=0.18, length=0.14),
        origin=Origin(
            xyz=(-2.15, 0.0, 0.18),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=tire,
        name="nose_wheel",
    )

    body.visual(
        Box((0.03, 1.08, 0.72)),
        origin=Origin(xyz=(-2.72, 0.0, 1.95), rpy=(0.0, -0.50, 0.0)),
        material=glass,
        name="windshield",
    )
    body.visual(
        Box((1.35, 0.025, 0.44)),
        origin=Origin(xyz=(-0.25, 1.12, 1.86)),
        material=glass,
        name="left_cabin_glass",
    )
    body.visual(
        Box((0.90, 0.025, 0.42)),
        origin=Origin(xyz=(1.05, -1.12, 1.84)),
        material=glass,
        name="right_rear_glass",
    )

    body.inertial = Inertial.from_geometry(
        Box((15.4, 2.9, 3.6)),
        mass=5200.0,
        origin=Origin(xyz=(2.0, 0.0, 1.8)),
    )

    main_rotor = model.part("main_rotor")
    main_rotor.visual(
        mesh_from_geometry(_main_rotor_geometry(), "main_rotor"),
        material=rotor_gray,
        name="main_rotor",
    )
    main_rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=6.1, length=0.24),
        mass=240.0,
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
    )

    tail_rotor = model.part("tail_rotor")
    tail_rotor.visual(
        mesh_from_geometry(_tail_rotor_geometry(), "tail_rotor"),
        material=rotor_gray,
        name="tail_rotor",
    )
    tail_rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.92, length=0.22),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    left_cockpit_door = model.part("left_cockpit_door")
    left_cockpit_door.visual(
        Box((0.92, 0.06, 1.18)),
        origin=Origin(xyz=(0.46, 0.0, 0.0)),
        material=olive,
        name="left_panel",
    )
    left_cockpit_door.visual(
        Box((0.10, 0.08, 1.18)),
        origin=Origin(xyz=(0.05, 0.0, 0.0)),
        material=dark_olive,
        name="left_hinge_strip",
    )
    left_cockpit_door.visual(
        Box((0.50, 0.025, 0.56)),
        origin=Origin(xyz=(0.48, 0.0, 0.18)),
        material=glass,
        name="left_window",
    )
    left_cockpit_door.visual(
        Cylinder(radius=0.035, length=0.18),
        origin=Origin(xyz=(0.0, -0.06, 0.34)),
        material=steel,
        name="left_hinge_upper",
    )
    left_cockpit_door.visual(
        Cylinder(radius=0.035, length=0.18),
        origin=Origin(xyz=(0.0, -0.06, -0.34)),
        material=steel,
        name="left_hinge_lower",
    )
    left_cockpit_door.inertial = Inertial.from_geometry(
        Box((0.92, 0.08, 1.18)),
        mass=35.0,
        origin=Origin(xyz=(0.46, 0.0, 0.0)),
    )

    right_cockpit_door = model.part("right_cockpit_door")
    right_cockpit_door.visual(
        Box((0.92, 0.06, 1.18)),
        origin=Origin(xyz=(0.46, 0.0, 0.0)),
        material=olive,
        name="right_panel",
    )
    right_cockpit_door.visual(
        Box((0.10, 0.08, 1.18)),
        origin=Origin(xyz=(0.05, 0.0, 0.0)),
        material=dark_olive,
        name="right_hinge_strip",
    )
    right_cockpit_door.visual(
        Box((0.50, 0.025, 0.56)),
        origin=Origin(xyz=(0.48, 0.0, 0.18)),
        material=glass,
        name="right_window",
    )
    right_cockpit_door.visual(
        Cylinder(radius=0.035, length=0.18),
        origin=Origin(xyz=(0.0, 0.06, 0.34)),
        material=steel,
        name="right_hinge_upper",
    )
    right_cockpit_door.visual(
        Cylinder(radius=0.035, length=0.18),
        origin=Origin(xyz=(0.0, 0.06, -0.34)),
        material=steel,
        name="right_hinge_lower",
    )
    right_cockpit_door.inertial = Inertial.from_geometry(
        Box((0.92, 0.08, 1.18)),
        mass=35.0,
        origin=Origin(xyz=(0.46, 0.0, 0.0)),
    )

    right_cabin_door = model.part("right_cabin_door")
    right_cabin_door.visual(
        Box((1.55, 0.06, 1.55)),
        origin=Origin(xyz=(0.775, -0.05, -0.84)),
        material=olive,
        name="cabin_panel",
    )
    right_cabin_door.visual(
        Box((0.78, 0.025, 0.50)),
        origin=Origin(xyz=(0.76, -0.055, -0.22)),
        material=glass,
        name="cabin_window",
    )
    right_cabin_door.visual(
        Box((0.10, 0.08, 0.46)),
        origin=Origin(xyz=(0.10, 0.0, -0.27)),
        material=steel,
        name="front_hanger",
    )
    right_cabin_door.visual(
        Box((0.10, 0.08, 0.46)),
        origin=Origin(xyz=(1.25, 0.0, -0.27)),
        material=steel,
        name="rear_hanger",
    )
    right_cabin_door.visual(
        Box((0.18, 0.12, 0.10)),
        origin=Origin(xyz=(0.10, 0.0, -0.11)),
        material=steel,
        name="front_carriage",
    )
    right_cabin_door.visual(
        Box((0.18, 0.12, 0.10)),
        origin=Origin(xyz=(1.25, 0.0, -0.11)),
        material=steel,
        name="rear_carriage",
    )
    right_cabin_door.inertial = Inertial.from_geometry(
        Box((1.55, 0.10, 1.60)),
        mass=70.0,
        origin=Origin(xyz=(0.78, -0.03, -0.82)),
    )

    model.articulation(
        "body_to_main_rotor",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=main_rotor,
        origin=Origin(xyz=(0.0, 0.0, 3.50)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1800.0, velocity=40.0),
    )
    model.articulation(
        "body_to_tail_rotor",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=tail_rotor,
        origin=Origin(xyz=(7.28, -0.61, 2.56)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=300.0, velocity=70.0),
    )
    model.articulation(
        "body_to_left_cockpit_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_cockpit_door,
        origin=Origin(xyz=(-2.72, 1.16, 1.60)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.4,
            lower=0.0,
            upper=math.radians(72.0),
        ),
    )
    model.articulation(
        "body_to_right_cockpit_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_cockpit_door,
        origin=Origin(xyz=(-2.72, -1.16, 1.60)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.4,
            lower=0.0,
            upper=math.radians(72.0),
        ),
    )
    model.articulation(
        "body_to_right_cabin_door",
        ArticulationType.PRISMATIC,
        parent=body,
        child=right_cabin_door,
        origin=Origin(xyz=(-1.55, -1.34, 2.16)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=450.0,
            velocity=0.8,
            lower=0.0,
            upper=1.25,
        ),
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

    body = object_model.get_part("body")
    main_rotor = object_model.get_part("main_rotor")
    tail_rotor = object_model.get_part("tail_rotor")
    left_door = object_model.get_part("left_cockpit_door")
    right_door = object_model.get_part("right_cockpit_door")
    cabin_door = object_model.get_part("right_cabin_door")

    main_rotor_joint = object_model.get_articulation("body_to_main_rotor")
    tail_rotor_joint = object_model.get_articulation("body_to_tail_rotor")
    left_door_joint = object_model.get_articulation("body_to_left_cockpit_door")
    right_door_joint = object_model.get_articulation("body_to_right_cockpit_door")
    cabin_door_joint = object_model.get_articulation("body_to_right_cabin_door")

    ctx.check(
        "main rotor is continuous about mast axis",
        main_rotor_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(main_rotor_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={main_rotor_joint.articulation_type}, axis={main_rotor_joint.axis}",
    )
    ctx.check(
        "tail rotor is continuous about transverse axis",
        tail_rotor_joint.articulation_type == ArticulationType.CONTINUOUS
        and abs(tail_rotor_joint.axis[1]) > 0.99,
        details=f"type={tail_rotor_joint.articulation_type}, axis={tail_rotor_joint.axis}",
    )

    main_pos = ctx.part_world_position(main_rotor)
    tail_pos = ctx.part_world_position(tail_rotor)
    ctx.check(
        "rotors are mounted in realistic locations",
        main_pos is not None
        and tail_pos is not None
        and main_pos[2] > 3.4
        and tail_pos[0] > 7.0
        and tail_pos[2] > 2.4
        and tail_pos[1] < -0.45,
        details=f"main={main_pos}, tail={tail_pos}",
    )

    ctx.expect_gap(
        body,
        cabin_door,
        axis="z",
        positive_elem="door_rail",
        negative_elem="front_carriage",
        min_gap=0.0,
        max_gap=0.03,
        name="front carriage hangs just below rail",
    )
    ctx.expect_overlap(
        body,
        cabin_door,
        axes="x",
        elem_a="door_rail",
        elem_b="rear_carriage",
        min_overlap=0.16,
        name="rear carriage remains engaged with rail at rest",
    )

    left_closed = _aabb_center(ctx.part_element_world_aabb(left_door, elem="left_panel"))
    right_closed = _aabb_center(ctx.part_element_world_aabb(right_door, elem="right_panel"))
    cabin_closed = _aabb_center(ctx.part_element_world_aabb(cabin_door, elem="cabin_panel"))

    with ctx.pose(
        {
            left_door_joint: math.radians(60.0),
            right_door_joint: math.radians(60.0),
        }
    ):
        left_open = _aabb_center(ctx.part_element_world_aabb(left_door, elem="left_panel"))
        right_open = _aabb_center(ctx.part_element_world_aabb(right_door, elem="right_panel"))
        ctx.check(
            "left cockpit door swings outward",
            left_closed is not None
            and left_open is not None
            and left_open[1] > left_closed[1] + 0.20,
            details=f"closed={left_closed}, open={left_open}",
        )
        ctx.check(
            "right cockpit door swings outward",
            right_closed is not None
            and right_open is not None
            and right_open[1] < right_closed[1] - 0.20,
            details=f"closed={right_closed}, open={right_open}",
        )

    with ctx.pose({cabin_door_joint: 1.25}):
        cabin_open = _aabb_center(ctx.part_element_world_aabb(cabin_door, elem="cabin_panel"))
        ctx.check(
            "cabin door slides aft along fuselage",
            cabin_closed is not None
            and cabin_open is not None
            and cabin_open[0] > cabin_closed[0] + 1.0
            and abs(cabin_open[1] - cabin_closed[1]) < 0.05
            and abs(cabin_open[2] - cabin_closed[2]) < 0.05,
            details=f"closed={cabin_closed}, open={cabin_open}",
        )
        ctx.expect_gap(
            body,
            cabin_door,
            axis="z",
            positive_elem="door_rail",
            negative_elem="rear_carriage",
            min_gap=0.0,
            max_gap=0.03,
            name="rear carriage stays just below rail when open",
        )
        ctx.expect_overlap(
            body,
            cabin_door,
            axes="x",
            elem_a="door_rail",
            elem_b="rear_carriage",
            min_overlap=0.16,
            name="rear carriage remains engaged with rail when open",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
