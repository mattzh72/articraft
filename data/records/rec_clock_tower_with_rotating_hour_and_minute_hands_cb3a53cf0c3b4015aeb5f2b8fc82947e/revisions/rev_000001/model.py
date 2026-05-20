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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    section_loft,
)


def _square_section(side: float, z_pos: float) -> list[tuple[float, float, float]]:
    half = side * 0.5
    return [
        (-half, -half, z_pos),
        (half, -half, z_pos),
        (half, half, z_pos),
        (-half, half, z_pos),
    ]


def _clock_hand_profile(*, length: float, tail: float, root_half: float, tip_half: float) -> list[tuple[float, float]]:
    shoulder = length * 0.22
    neck = length * 0.55
    return [
        (-root_half * 1.10, -tail),
        (root_half * 1.10, -tail),
        (root_half * 1.55, -tail * 0.45),
        (root_half * 1.22, shoulder),
        (tip_half * 1.40, neck),
        (0.0, length),
        (-tip_half * 1.40, neck),
        (-root_half * 1.22, shoulder),
        (-root_half * 1.55, -tail * 0.45),
    ]


def _build_shaft_mesh():
    return mesh_from_geometry(
        section_loft(
            [
                _square_section(1.56, 0.00),
                _square_section(1.34, 2.40),
                _square_section(1.10, 4.85),
                _square_section(0.80, 5.95),
                _square_section(0.02, 6.55),
            ]
        ),
        "obelisk_shaft_shell",
    )


def _build_hand_meshes():
    hour_geom = ExtrudeGeometry.from_z0(
        _clock_hand_profile(length=0.225, tail=0.070, root_half=0.020, tip_half=0.008),
        0.002,
    )
    minute_geom = ExtrudeGeometry.from_z0(
        _clock_hand_profile(length=0.315, tail=0.085, root_half=0.013, tip_half=0.006),
        0.002,
    )
    return (
        mesh_from_geometry(hour_geom, "clock_hour_hand"),
        mesh_from_geometry(minute_geom, "clock_minute_hand"),
    )


def _normalize(vec: tuple[float, float, float]) -> tuple[float, float, float]:
    length = math.sqrt(sum(component * component for component in vec))
    return tuple(component / length for component in vec)


def _cross(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _dot(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _rpy_from_axes(
    x_axis: tuple[float, float, float],
    y_axis: tuple[float, float, float],
    z_axis: tuple[float, float, float],
) -> tuple[float, float, float]:
    r00, r10, r20 = x_axis[0], x_axis[1], x_axis[2]
    r01, r11, r21 = y_axis[0], y_axis[1], y_axis[2]
    r02, r12, r22 = z_axis[0], z_axis[1], z_axis[2]

    pitch = -math.asin(max(-1.0, min(1.0, r20)))
    cp = math.cos(pitch)
    if abs(cp) > 1e-8:
        roll = math.atan2(r21, r22)
        yaw = math.atan2(r10, r00)
    else:
        roll = 0.0
        yaw = math.atan2(-r01, r11)
    return (roll, pitch, yaw)


def _shaft_half_width_and_slope(local_z: float) -> tuple[float, float]:
    if local_z <= 2.40:
        side0, side1, z0, z1 = 1.56, 1.34, 0.00, 2.40
    elif local_z <= 4.85:
        side0, side1, z0, z1 = 1.34, 1.10, 2.40, 4.85
    elif local_z <= 5.95:
        side0, side1, z0, z1 = 1.10, 0.80, 4.85, 5.95
    else:
        side0, side1, z0, z1 = 0.80, 0.02, 5.95, 6.55
    t = (local_z - z0) / (z1 - z0)
    side = side0 + (side1 - side0) * t
    half_width = side * 0.5
    half_width_slope = (side1 - side0) * 0.5 / (z1 - z0)
    return half_width, half_width_slope


def _face_mount_origin(
    face_name: str,
    *,
    hub_local_z: float,
    offset: float = 0.0,
    local_xy: tuple[float, float] = (0.0, 0.0),
) -> Origin:
    half_width, half_width_slope = _shaft_half_width_and_slope(hub_local_z)
    outward_lift = -half_width_slope
    if face_name == "north":
        normal = _normalize((0.0, 1.0, outward_lift))
        surface_point = (0.0, half_width, hub_local_z)
    elif face_name == "south":
        normal = _normalize((0.0, -1.0, outward_lift))
        surface_point = (0.0, -half_width, hub_local_z)
    elif face_name == "east":
        normal = _normalize((1.0, 0.0, outward_lift))
        surface_point = (half_width, 0.0, hub_local_z)
    else:
        normal = _normalize((-1.0, 0.0, outward_lift))
        surface_point = (-half_width, 0.0, hub_local_z)

    world_up = (0.0, 0.0, 1.0)
    up_projection = (
        world_up[0] - _dot(world_up, normal) * normal[0],
        world_up[1] - _dot(world_up, normal) * normal[1],
        world_up[2] - _dot(world_up, normal) * normal[2],
    )
    y_axis = _normalize(up_projection)
    x_axis = _normalize(_cross(y_axis, normal))
    rpy = _rpy_from_axes(x_axis, y_axis, normal)
    u, v = local_xy
    xyz = (
        surface_point[0] + u * x_axis[0] + v * y_axis[0] + offset * normal[0],
        surface_point[1] + u * x_axis[1] + v * y_axis[1] + offset * normal[1],
        surface_point[2] + u * x_axis[2] + v * y_axis[2] + offset * normal[2],
    )
    return Origin(xyz=xyz, rpy=rpy)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="obelisk_clock_tower")

    granite_base = model.material("granite_base", rgba=(0.42, 0.42, 0.44, 1.0))
    granite_shaft = model.material("granite_shaft", rgba=(0.48, 0.48, 0.50, 1.0))
    dark_stone = model.material("dark_stone", rgba=(0.28, 0.28, 0.30, 1.0))
    ivory_dial = model.material("ivory_dial", rgba=(0.87, 0.84, 0.78, 1.0))
    bronze = model.material("bronze", rgba=(0.59, 0.47, 0.30, 1.0))
    blackened_steel = model.material("blackened_steel", rgba=(0.11, 0.11, 0.12, 1.0))

    shaft_mesh = _build_shaft_mesh()
    bezel_ring_mesh = mesh_from_geometry(TorusGeometry(radius=0.372, tube=0.018), "clock_bezel_ring")
    hour_hand_mesh, minute_hand_mesh = _build_hand_meshes()

    plinth_base = model.part("plinth_base")
    plinth_base.visual(
        Box((2.90, 2.90, 0.28)),
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        material=granite_base,
        name="footing",
    )
    plinth_base.visual(
        Box((2.35, 2.35, 0.28)),
        origin=Origin(xyz=(0.0, 0.0, 0.42)),
        material=granite_base,
        name="middle_step",
    )
    plinth_base.visual(
        Box((1.92, 1.92, 0.34)),
        origin=Origin(xyz=(0.0, 0.0, 0.73)),
        material=granite_base,
        name="upper_plinth",
    )
    plinth_base.visual(
        Box((1.70, 1.70, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.96)),
        material=granite_base,
        name="shaft_seat",
    )
    plinth_base.inertial = Inertial.from_geometry(
        Box((2.90, 2.90, 1.02)),
        mass=5200.0,
        origin=Origin(xyz=(0.0, 0.0, 0.51)),
    )

    obelisk_shaft = model.part("obelisk_shaft")
    obelisk_shaft.visual(shaft_mesh, material=granite_shaft, name="shaft_form")
    obelisk_shaft.inertial = Inertial.from_geometry(
        Box((1.56, 1.56, 6.55)),
        mass=6800.0,
        origin=Origin(xyz=(0.0, 0.0, 3.275)),
    )

    model.articulation(
        "plinth_to_shaft",
        ArticulationType.FIXED,
        parent=plinth_base,
        child=obelisk_shaft,
        origin=Origin(xyz=(0.0, 0.0, 1.02)),
    )

    hub_local_z = 5.33
    face_specs = {
        "north": {
            "plane_axes": "xz",
        },
        "east": {
            "plane_axes": "yz",
        },
        "south": {
            "plane_axes": "xz",
        },
        "west": {
            "plane_axes": "yz",
        },
    }

    for face_name, spec in face_specs.items():
        obelisk_shaft.visual(
            Box((0.92, 0.92, 0.012)),
            origin=_face_mount_origin(
                face_name,
                hub_local_z=hub_local_z,
                offset=-0.004,
            ),
            material=dark_stone,
            name=f"{face_name}_dial_surround",
        )
        obelisk_shaft.visual(
            Cylinder(radius=0.356, length=0.018),
            origin=_face_mount_origin(
                face_name,
                hub_local_z=hub_local_z,
                offset=-0.010,
            ),
            material=ivory_dial,
            name=f"{face_name}_dial_face",
        )
        obelisk_shaft.visual(
            bezel_ring_mesh,
            origin=_face_mount_origin(
                face_name,
                hub_local_z=hub_local_z,
                offset=0.006,
            ),
            material=bronze,
            name=f"{face_name}_bezel",
        )
        obelisk_shaft.visual(
            Box((0.018, 0.065, 0.006)),
            origin=_face_mount_origin(
                face_name,
                hub_local_z=hub_local_z,
                offset=-0.002,
                local_xy=(0.0, 0.268),
            ),
            material=bronze,
            name=f"{face_name}_marker_12",
        )
        obelisk_shaft.visual(
            Box((0.018, 0.065, 0.006)),
            origin=_face_mount_origin(
                face_name,
                hub_local_z=hub_local_z,
                offset=-0.002,
                local_xy=(0.0, -0.268),
            ),
            material=bronze,
            name=f"{face_name}_marker_6",
        )
        obelisk_shaft.visual(
            Box((0.065, 0.018, 0.006)),
            origin=_face_mount_origin(
                face_name,
                hub_local_z=hub_local_z,
                offset=-0.002,
                local_xy=(0.268, 0.0),
            ),
            material=bronze,
            name=f"{face_name}_marker_3",
        )
        obelisk_shaft.visual(
            Box((0.065, 0.018, 0.006)),
            origin=_face_mount_origin(
                face_name,
                hub_local_z=hub_local_z,
                offset=-0.002,
                local_xy=(-0.268, 0.0),
            ),
            material=bronze,
            name=f"{face_name}_marker_9",
        )
        obelisk_shaft.visual(
            Cylinder(radius=0.014, length=0.020),
            origin=_face_mount_origin(
                face_name,
                hub_local_z=hub_local_z,
                offset=0.010,
            ),
            material=bronze,
            name=f"{face_name}_spindle",
        )

        hour_hand = model.part(f"{face_name}_hour_hand")
        hour_hand.visual(
            Cylinder(radius=0.020, length=0.002),
            origin=Origin(xyz=(0.0, 0.0, 0.001)),
            material=bronze,
            name="hour_hub",
        )
        hour_hand.visual(
            Box((0.024, 0.194, 0.002)),
            origin=Origin(xyz=(0.0, 0.114, 0.001)),
            material=blackened_steel,
            name="hour_blade",
        )
        hour_hand.visual(
            Box((0.016, 0.064, 0.002)),
            origin=Origin(xyz=(0.0, -0.044, 0.001)),
            material=blackened_steel,
            name="hour_tail",
        )
        hour_hand.inertial = Inertial.from_geometry(
            Box((0.060, 0.260, 0.004)),
            mass=2.0,
            origin=Origin(xyz=(0.0, 0.060, 0.002)),
        )

        minute_hand = model.part(f"{face_name}_minute_hand")
        minute_hand.visual(
            Cylinder(radius=0.014, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, 0.004)),
            material=bronze,
            name="minute_hub",
        )
        minute_hand.visual(
            Box((0.016, 0.282, 0.002)),
            origin=Origin(xyz=(0.0, 0.148, 0.005)),
            material=blackened_steel,
            name="minute_blade",
        )
        minute_hand.inertial = Inertial.from_geometry(
            Box((0.040, 0.350, 0.004)),
            mass=1.6,
            origin=Origin(xyz=(0.0, 0.090, 0.002)),
        )

        model.articulation(
            f"shaft_to_{face_name}_hour_hand",
            ArticulationType.REVOLUTE,
            parent=obelisk_shaft,
            child=hour_hand,
            origin=_face_mount_origin(
                face_name,
                hub_local_z=hub_local_z,
                offset=0.020,
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=1.0,
                lower=-math.pi,
                upper=math.pi,
            ),
        )
        model.articulation(
            f"shaft_to_{face_name}_minute_hand",
            ArticulationType.REVOLUTE,
            parent=obelisk_shaft,
            child=minute_hand,
            origin=_face_mount_origin(
                face_name,
                hub_local_z=hub_local_z,
                offset=0.020,
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=1.5,
                lower=-math.pi,
                upper=math.pi,
            ),
        )

    model.meta["face_specs"] = face_specs
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

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
    ctx.fail_if_parts_overlap_in_current_pose()

    plinth_base = object_model.get_part("plinth_base")
    obelisk_shaft = object_model.get_part("obelisk_shaft")
    plinth_to_shaft = object_model.get_articulation("plinth_to_shaft")
    ctx.check("plinth_to_shaft_fixed", plinth_to_shaft.articulation_type == ArticulationType.FIXED, "Shaft must be fixed to the plinth.")
    ctx.expect_contact(obelisk_shaft, plinth_base, name="shaft_seated_on_plinth")

    face_specs = object_model.meta["face_specs"]
    for face_name, spec in face_specs.items():
        hour_hand = object_model.get_part(f"{face_name}_hour_hand")
        minute_hand = object_model.get_part(f"{face_name}_minute_hand")
        hour_joint = object_model.get_articulation(f"shaft_to_{face_name}_hour_hand")
        minute_joint = object_model.get_articulation(f"shaft_to_{face_name}_minute_hand")
        dial_face = obelisk_shaft.get_visual(f"{face_name}_dial_face")
        dial_surround = obelisk_shaft.get_visual(f"{face_name}_dial_surround")
        spindle = obelisk_shaft.get_visual(f"{face_name}_spindle")

        ctx.check(
            f"{face_name}_hour_axis_is_face_normal",
            hour_joint.axis == (0.0, 0.0, 1.0),
            f"{face_name} hour hand must revolve about the dial normal.",
        )
        ctx.check(
            f"{face_name}_minute_axis_is_face_normal",
            minute_joint.axis == (0.0, 0.0, 1.0),
            f"{face_name} minute hand must revolve about the dial normal.",
        )
        ctx.check(
            f"{face_name}_hand_hubs_are_concentric",
            hour_joint.origin is not None
            and minute_joint.origin is not None
            and hour_joint.origin.xyz[:2] == minute_joint.origin.xyz[:2],
            f"{face_name} hour and minute joints must share the same hub center.",
        )

        ctx.check(
            f"{face_name}_dial_visuals_present",
            dial_face.name == f"{face_name}_dial_face" and dial_surround.name == f"{face_name}_dial_surround",
            f"{face_name} face should have explicit dial visuals embedded in the shaft.",
        )
        ctx.expect_contact(hour_hand, obelisk_shaft, elem_b=spindle, name=f"{face_name}_hour_contacts_spindle")
        ctx.expect_contact(minute_hand, hour_hand, elem_b="hour_hub", name=f"{face_name}_minute_contacts_hour_hub")
        ctx.expect_overlap(
            hour_hand,
            obelisk_shaft,
            elem_b=dial_face,
            axes=spec["plane_axes"],
            min_overlap=0.04,
            name=f"{face_name}_hour_within_dial_footprint",
        )
        ctx.expect_overlap(
            minute_hand,
            obelisk_shaft,
            elem_b=dial_face,
            axes=spec["plane_axes"],
            min_overlap=0.02,
            name=f"{face_name}_minute_within_dial_footprint",
        )

    north_minute = object_model.get_part("north_minute_hand")
    north_minute_joint = object_model.get_articulation("shaft_to_north_minute_hand")
    north_rest_aabb = ctx.part_world_aabb(north_minute)
    with ctx.pose({north_minute_joint: math.pi / 2.0}):
        north_quarter_aabb = ctx.part_world_aabb(north_minute)
        ctx.check(
            "north_minute_hand_moves_under_pose",
            north_rest_aabb is not None and north_quarter_aabb is not None and north_rest_aabb != north_quarter_aabb,
            "The north minute hand should visibly rotate about its hub.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
