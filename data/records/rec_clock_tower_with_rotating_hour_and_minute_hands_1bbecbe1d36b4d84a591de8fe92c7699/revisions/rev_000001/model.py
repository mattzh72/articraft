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
    DomeGeometry,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _rotated_xy(x: float, y: float, angle: float) -> tuple[float, float]:
    c = cos(angle)
    s = sin(angle)
    return (c * x - s * y, s * x + c * y)


def _build_hand_blade_mesh(
    name: str,
    *,
    length: float,
    tail: float,
    root_width: float,
    mid_width: float,
    tip_width: float,
    thickness: float,
):
    profile = [
        (-root_width * 0.22, -tail),
        (root_width * 0.22, -tail),
        (root_width * 0.36, -0.08),
        (mid_width * 0.50, length * 0.28),
        (tip_width * 0.48, length * 0.78),
        (0.0, length),
        (-tip_width * 0.48, length * 0.78),
        (-mid_width * 0.50, length * 0.28),
        (-root_width * 0.36, -0.08),
    ]
    return _save_mesh(name, ExtrudeGeometry.from_z0(profile, thickness))


def _face_specs() -> tuple[dict[str, object], ...]:
    dial_z = 14.2
    stage_half = 2.4
    dial_thickness = 0.02
    bezel_center = stage_half + 0.055
    dial_center = stage_half + dial_thickness * 0.5
    joint_plane = stage_half + dial_thickness
    return (
        {
            "name": "north",
            "dial_xyz": (0.0, dial_center, dial_z),
            "bezel_xyz": (0.0, bezel_center, dial_z),
            "joint_xyz": (0.0, joint_plane, dial_z),
            "rpy": (-pi / 2.0, 0.0, 0.0),
            "spin": pi,
            "plane_axes": "xz",
        },
        {
            "name": "south",
            "dial_xyz": (0.0, -dial_center, dial_z),
            "bezel_xyz": (0.0, -bezel_center, dial_z),
            "joint_xyz": (0.0, -joint_plane, dial_z),
            "rpy": (pi / 2.0, 0.0, 0.0),
            "spin": 0.0,
            "plane_axes": "xz",
        },
        {
            "name": "east",
            "dial_xyz": (dial_center, 0.0, dial_z),
            "bezel_xyz": (bezel_center, 0.0, dial_z),
            "joint_xyz": (joint_plane, 0.0, dial_z),
            "rpy": (0.0, pi / 2.0, 0.0),
            "spin": pi / 2.0,
            "plane_axes": "yz",
        },
        {
            "name": "west",
            "dial_xyz": (-dial_center, 0.0, dial_z),
            "bezel_xyz": (-bezel_center, 0.0, dial_z),
            "joint_xyz": (-joint_plane, 0.0, dial_z),
            "rpy": (0.0, -pi / 2.0, 0.0),
            "spin": -pi / 2.0,
            "plane_axes": "yz",
        },
    )


def _add_hand_visuals(
    part,
    *,
    blade_mesh,
    material,
    spin: float,
    blade_start: float,
    blade_thickness: float,
    hub_start: float,
    hub_length: float,
    hub_radius: float,
    tip_offset: float,
    tip_width: float,
) -> None:
    part.visual(
        Cylinder(radius=hub_radius, length=hub_length),
        origin=Origin(xyz=(0.0, 0.0, hub_start + hub_length * 0.5)),
        material=material,
        name="hub",
    )
    part.visual(
        blade_mesh,
        origin=Origin(xyz=(0.0, 0.0, blade_start), rpy=(0.0, 0.0, spin)),
        material=material,
        name="blade",
    )
    tip_x, tip_y = _rotated_xy(0.0, tip_offset, spin)
    part.visual(
        Box((tip_width, 0.05, blade_thickness)),
        origin=Origin(
            xyz=(tip_x, tip_y, blade_start + blade_thickness * 0.5),
            rpy=(0.0, 0.0, spin),
        ),
        material=material,
        name="tip",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="railway_station_clock_tower")

    masonry = model.material("masonry", rgba=(0.66, 0.64, 0.60, 1.0))
    trim_stone = model.material("trim_stone", rgba=(0.77, 0.75, 0.71, 1.0))
    dark_clock = model.material("dark_clock", rgba=(0.15, 0.15, 0.16, 1.0))
    clock_face = model.material("clock_face", rgba=(0.93, 0.92, 0.86, 1.0))
    hour_metal = model.material("hour_metal", rgba=(0.18, 0.18, 0.19, 1.0))
    minute_metal = model.material("minute_metal", rgba=(0.10, 0.10, 0.11, 1.0))
    lantern_glass = model.material("lantern_glass", rgba=(0.73, 0.85, 0.93, 0.35))
    lantern_copper = model.material("lantern_copper", rgba=(0.40, 0.29, 0.21, 1.0))

    dial_bezel_mesh = _save_mesh(
        "dial_bezel",
        TorusGeometry(radius=1.09, tube=0.055, radial_segments=18, tubular_segments=72),
    )
    lantern_dome_mesh = _save_mesh(
        "lantern_dome",
        DomeGeometry(radius=1.35, radial_segments=28, height_segments=14, closed=True),
    )
    hour_blade_mesh = _build_hand_blade_mesh(
        "hour_blade",
        length=0.62,
        tail=0.16,
        root_width=0.18,
        mid_width=0.08,
        tip_width=0.045,
        thickness=0.010,
    )
    minute_blade_mesh = _build_hand_blade_mesh(
        "minute_blade",
        length=0.90,
        tail=0.19,
        root_width=0.14,
        mid_width=0.055,
        tip_width=0.030,
        thickness=0.010,
    )

    tower = model.part("tower")
    tower.inertial = Inertial.from_geometry(
        Box((6.4, 6.4, 21.6)),
        mass=180000.0,
        origin=Origin(xyz=(0.0, 0.0, 10.8)),
    )
    tower.visual(
        Box((6.4, 6.4, 1.6)),
        origin=Origin(xyz=(0.0, 0.0, 0.8)),
        material=masonry,
        name="foundation_plinth",
    )
    tower.visual(
        Box((5.6, 5.6, 0.4)),
        origin=Origin(xyz=(0.0, 0.0, 1.8)),
        material=trim_stone,
        name="water_table",
    )
    tower.visual(
        Box((4.2, 4.2, 10.8)),
        origin=Origin(xyz=(0.0, 0.0, 7.4)),
        material=masonry,
        name="shaft",
    )
    for sx in (-1.93, 1.93):
        for sy in (-1.93, 1.93):
            tower.visual(
                Box((0.34, 0.34, 10.8)),
                origin=Origin(xyz=(sx, sy, 7.4)),
                material=trim_stone,
            )
    tower.visual(
        Box((4.8, 4.8, 2.8)),
        origin=Origin(xyz=(0.0, 0.0, 14.2)),
        material=masonry,
        name="clock_stage",
    )
    tower.visual(
        Box((5.2, 5.2, 0.32)),
        origin=Origin(xyz=(0.0, 0.0, 12.96)),
        material=trim_stone,
        name="lower_clock_cornice",
    )
    tower.visual(
        Box((5.4, 5.4, 0.5)),
        origin=Origin(xyz=(0.0, 0.0, 15.85)),
        material=trim_stone,
        name="upper_clock_cornice",
    )
    tower.visual(
        Cylinder(radius=1.55, length=0.45),
        origin=Origin(xyz=(0.0, 0.0, 16.325)),
        material=lantern_copper,
        name="lantern_base_ring",
    )
    tower.visual(
        Cylinder(radius=1.28, length=2.2),
        origin=Origin(xyz=(0.0, 0.0, 17.65)),
        material=lantern_glass,
        name="lantern_glazing",
    )
    tower.visual(
        Cylinder(radius=1.45, length=0.4),
        origin=Origin(xyz=(0.0, 0.0, 18.95)),
        material=lantern_copper,
        name="lantern_cap_ring",
    )
    for mullion_index in range(8):
        angle = 2.0 * pi * mullion_index / 8.0
        tower.visual(
            Cylinder(radius=0.05, length=2.2),
            origin=Origin(xyz=(1.23 * cos(angle), 1.23 * sin(angle), 17.65)),
            material=lantern_copper,
        )
    tower.visual(
        lantern_dome_mesh,
        origin=Origin(xyz=(0.0, 0.0, 19.15)),
        material=lantern_copper,
        name="lantern_dome",
    )
    tower.visual(
        Cylinder(radius=0.12, length=1.1),
        origin=Origin(xyz=(0.0, 0.0, 20.05)),
        material=lantern_copper,
        name="spire",
    )
    tower.visual(
        Cylinder(radius=0.20, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 20.69)),
        material=lantern_copper,
        name="finial",
    )

    for face in _face_specs():
        name = face["name"]
        dial_xyz = face["dial_xyz"]
        bezel_xyz = face["bezel_xyz"]
        rpy = face["rpy"]
        tower.visual(
            Cylinder(radius=1.02, length=0.02),
            origin=Origin(xyz=dial_xyz, rpy=rpy),
            material=clock_face,
            name=f"{name}_dial_face",
        )
        tower.visual(
            dial_bezel_mesh,
            origin=Origin(xyz=bezel_xyz, rpy=rpy),
            material=dark_clock,
            name=f"{name}_dial_bezel",
        )

        hour_part = model.part(f"{name}_hour_hand")
        hour_part.inertial = Inertial.from_geometry(
            Box((0.22, 0.82, 0.03)),
            mass=0.18,
            origin=Origin(xyz=(0.0, 0.23, 0.015)),
        )
        _add_hand_visuals(
            hour_part,
            blade_mesh=hour_blade_mesh,
            material=hour_metal,
            spin=face["spin"],
            blade_start=0.004,
            blade_thickness=0.010,
            hub_start=0.0,
            hub_length=0.018,
            hub_radius=0.11,
            tip_offset=0.58,
            tip_width=0.07,
        )

        minute_part = model.part(f"{name}_minute_hand")
        minute_part.inertial = Inertial.from_geometry(
            Box((0.16, 1.16, 0.04)),
            mass=0.14,
            origin=Origin(xyz=(0.0, 0.32, 0.024)),
        )
        _add_hand_visuals(
            minute_part,
            blade_mesh=minute_blade_mesh,
            material=minute_metal,
            spin=face["spin"],
            blade_start=0.024,
            blade_thickness=0.010,
            hub_start=0.018,
            hub_length=0.018,
            hub_radius=0.085,
            tip_offset=0.85,
            tip_width=0.05,
        )

        model.articulation(
            f"{name}_hour_spin",
            ArticulationType.CONTINUOUS,
            parent=tower,
            child=hour_part,
            origin=Origin(xyz=face["joint_xyz"], rpy=rpy),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=6.0, velocity=0.35),
        )
        model.articulation(
            f"{name}_minute_spin",
            ArticulationType.CONTINUOUS,
            parent=tower,
            child=minute_part,
            origin=Origin(xyz=face["joint_xyz"], rpy=rpy),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=6.0, velocity=1.5),
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

    tower = object_model.get_part("tower")
    face_names = tuple(face["name"] for face in _face_specs())

    def _elem_center(aabb):
        if aabb is None:
            return None
        return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))

    for face in _face_specs():
        name = face["name"]
        plane_axes = face["plane_axes"]
        hour_part = object_model.get_part(f"{name}_hour_hand")
        minute_part = object_model.get_part(f"{name}_minute_hand")
        hour_joint = object_model.get_articulation(f"{name}_hour_spin")
        minute_joint = object_model.get_articulation(f"{name}_minute_spin")

        ctx.check(f"{name} hour hand part present", hour_part is not None)
        ctx.check(f"{name} minute hand part present", minute_part is not None)
        ctx.check(
            f"{name} joints are continuous",
            hour_joint.joint_type == ArticulationType.CONTINUOUS
            and minute_joint.joint_type == ArticulationType.CONTINUOUS,
            details=f"hour={hour_joint.joint_type}, minute={minute_joint.joint_type}",
        )
        ctx.check(
            f"{name} joints are unbounded",
            hour_joint.motion_limits is not None
            and minute_joint.motion_limits is not None
            and hour_joint.motion_limits.lower is None
            and hour_joint.motion_limits.upper is None
            and minute_joint.motion_limits.lower is None
            and minute_joint.motion_limits.upper is None,
            details=(
                f"hour_limits={hour_joint.motion_limits}, "
                f"minute_limits={minute_joint.motion_limits}"
            ),
        )
        ctx.check(
            f"{name} hands share one axle",
            tuple(hour_joint.origin.xyz) == tuple(minute_joint.origin.xyz)
            and tuple(hour_joint.origin.rpy) == tuple(minute_joint.origin.rpy)
            and tuple(hour_joint.axis) == tuple(minute_joint.axis),
            details=(
                f"hour_origin={hour_joint.origin}, minute_origin={minute_joint.origin}, "
                f"hour_axis={hour_joint.axis}, minute_axis={minute_joint.axis}"
            ),
        )
        ctx.expect_contact(
            hour_part,
            tower,
            elem_a="hub",
            elem_b=f"{name}_dial_face",
            name=f"{name} hour hub seats on the dial",
        )
        ctx.expect_contact(
            minute_part,
            hour_part,
            elem_a="hub",
            elem_b="hub",
            name=f"{name} minute hub stacks on the hour hub",
        )
        ctx.expect_within(
            hour_part,
            tower,
            axes=plane_axes,
            inner_elem="blade",
            outer_elem=f"{name}_dial_face",
            margin=0.10,
            name=f"{name} hour blade stays inside the dial opening",
        )
        ctx.expect_within(
            minute_part,
            tower,
            axes=plane_axes,
            inner_elem="blade",
            outer_elem=f"{name}_dial_face",
            margin=0.10,
            name=f"{name} minute blade stays inside the dial opening",
        )

    north_minute = object_model.get_articulation("north_minute_spin")
    north_minute_part = object_model.get_part("north_minute_hand")
    north_rest = _elem_center(ctx.part_element_world_aabb(north_minute_part, elem="tip"))
    with ctx.pose({north_minute: pi / 2.0}):
        north_quarter = _elem_center(ctx.part_element_world_aabb(north_minute_part, elem="tip"))
    ctx.check(
        "north minute hand swings around the face normal",
        north_rest is not None
        and north_quarter is not None
        and north_quarter[0] > north_rest[0] + 0.45
        and north_quarter[2] < north_rest[2] - 0.45
        and abs(north_quarter[1] - north_rest[1]) < 0.05,
        details=f"rest={north_rest}, quarter={north_quarter}",
    )

    east_hour = object_model.get_articulation("east_hour_spin")
    east_hour_part = object_model.get_part("east_hour_hand")
    east_rest = _elem_center(ctx.part_element_world_aabb(east_hour_part, elem="tip"))
    with ctx.pose({east_hour: pi / 2.0}):
        east_quarter = _elem_center(ctx.part_element_world_aabb(east_hour_part, elem="tip"))
    ctx.check(
        "east hour hand rotates in the dial plane",
        east_rest is not None
        and east_quarter is not None
        and east_quarter[1] < east_rest[1] - 0.30
        and east_quarter[2] < east_rest[2] - 0.30
        and abs(east_quarter[0] - east_rest[0]) < 0.05,
        details=f"rest={east_rest}, quarter={east_quarter}",
    )

    ctx.check(
        "all four clock faces are authored",
        all(object_model.get_part(f"{name}_hour_hand") is not None for name in face_names)
        and all(object_model.get_part(f"{name}_minute_hand") is not None for name in face_names),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
