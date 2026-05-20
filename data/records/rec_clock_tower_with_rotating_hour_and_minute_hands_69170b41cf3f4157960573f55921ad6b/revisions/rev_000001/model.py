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
    mesh_from_geometry,
    section_loft,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _square_loop(cx: float, cy: float, z: float, sx: float, sy: float) -> list[tuple[float, float, float]]:
    hx = sx * 0.5
    hy = sy * 0.5
    return [
        (cx - hx, cy - hy, z),
        (cx + hx, cy - hy, z),
        (cx + hx, cy + hy, z),
        (cx - hx, cy + hy, z),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="church_clock_tower")

    stone = model.material("stone", rgba=(0.70, 0.68, 0.62, 1.0))
    stone_trim = model.material("stone_trim", rgba=(0.82, 0.80, 0.74, 1.0))
    roof_slate = model.material("roof_slate", rgba=(0.24, 0.25, 0.29, 1.0))
    dial_ivory = model.material("dial_ivory", rgba=(0.93, 0.91, 0.82, 1.0))
    dark_opening = model.material("dark_opening", rgba=(0.16, 0.14, 0.12, 1.0))
    oxidized_metal = model.material("oxidized_metal", rgba=(0.10, 0.11, 0.11, 1.0))
    stained_glass = model.material("stained_glass", rgba=(0.22, 0.18, 0.24, 1.0))

    facade = model.part("facade")
    facade.inertial = Inertial.from_geometry(
        Box((10.0, 5.4, 19.4)),
        mass=18000.0,
        origin=Origin(xyz=(0.0, -1.4, 9.7)),
    )

    # Stone nave wall.
    facade.visual(
        Box((9.6, 1.0, 8.2)),
        origin=Origin(xyz=(0.0, 0.0, 4.1)),
        material=stone,
        name="nave_wall",
    )
    for x in (-3.55, 3.55):
        facade.visual(
            Box((0.82, 1.42, 5.2)),
            origin=Origin(xyz=(x, 0.0, 2.6)),
            material=stone_trim,
            name=f"buttress_{'left' if x < 0 else 'right'}",
        )

    # Portal surround and recessed door.
    facade.visual(
        Box((0.34, 0.24, 3.1)),
        origin=Origin(xyz=(-1.04, 0.38, 1.55)),
        material=stone_trim,
        name="portal_jamb_left",
    )
    facade.visual(
        Box((0.34, 0.24, 3.1)),
        origin=Origin(xyz=(1.04, 0.38, 1.55)),
        material=stone_trim,
        name="portal_jamb_right",
    )
    facade.visual(
        Box((2.42, 0.24, 0.34)),
        origin=Origin(xyz=(0.0, 0.38, 3.1)),
        material=stone_trim,
        name="portal_lintel",
    )
    facade.visual(
        Box((1.58, 0.12, 2.72)),
        origin=Origin(xyz=(0.0, 0.50, 1.42)),
        material=dark_opening,
        name="portal_door",
    )

    # Facade oculus.
    facade.visual(
        Cylinder(radius=0.82, length=0.18),
        origin=Origin(xyz=(0.0, 0.45, 5.25), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=stone_trim,
        name="oculus_bezel",
    )
    facade.visual(
        Cylinder(radius=0.60, length=0.10),
        origin=Origin(xyz=(0.0, 0.50, 5.25), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=stained_glass,
        name="oculus_glass",
    )
    facade.visual(
        Box((1.10, 0.06, 0.10)),
        origin=Origin(xyz=(0.0, 0.55, 5.25)),
        material=stone_trim,
        name="oculus_bar_horizontal",
    )
    facade.visual(
        Box((0.10, 0.06, 1.10)),
        origin=Origin(xyz=(0.0, 0.55, 5.25)),
        material=stone_trim,
        name="oculus_bar_vertical",
    )

    # Tower shaft behind the facade wall, with the front face flush to the wall front.
    facade.visual(
        Box((4.2, 4.2, 4.8)),
        origin=Origin(xyz=(0.0, -1.6, 10.6)),
        material=stone,
        name="tower_shaft",
    )
    facade.visual(
        Box((4.6, 4.6, 0.26)),
        origin=Origin(xyz=(0.0, -1.6, 8.33)),
        material=stone_trim,
        name="shaft_base_band",
    )
    for x in (-1.96, 1.96):
        facade.visual(
            Box((0.28, 0.36, 4.96)),
            origin=Origin(xyz=(x, 0.32, 10.6)),
            material=stone_trim,
            name=f"shaft_pilaster_{'left' if x < 0 else 'right'}",
        )

    # Large clock face on the front of the shaft.
    clock_z = 10.6
    facade.visual(
        Cylinder(radius=1.34, length=0.14),
        origin=Origin(xyz=(0.0, 0.48, clock_z), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=stone_trim,
        name="clock_bezel",
    )
    facade.visual(
        Cylinder(radius=1.18, length=0.06),
        origin=Origin(xyz=(0.0, 0.49, clock_z), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dial_ivory,
        name="clock_dial",
    )
    facade.visual(
        Cylinder(radius=0.11, length=0.068),
        origin=Origin(xyz=(0.0, 0.550, clock_z), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=oxidized_metal,
        name="clock_spindle",
    )
    for idx in range(12):
        angle = idx * pi / 6.0
        radius = 0.92
        x = radius * sin(angle)
        z = clock_z + radius * cos(angle)
        major = idx % 3 == 0
        marker_x = 0.14 if major else 0.08
        marker_z = 0.34 if major else 0.18
        facade.visual(
            Box((marker_x, 0.05, marker_z)),
            origin=Origin(xyz=(x, 0.55, z), rpy=(0.0, angle, 0.0)),
            material=oxidized_metal,
            name=f"dial_marker_{idx:02d}",
        )

    # Bell chamber above the shaft.
    facade.visual(
        Box((5.2, 5.2, 0.28)),
        origin=Origin(xyz=(0.0, -1.9, 12.96)),
        material=stone_trim,
        name="bell_base_cornice",
    )
    facade.visual(
        Box((4.8, 4.8, 4.4)),
        origin=Origin(xyz=(0.0, -1.9, 15.2)),
        material=stone,
        name="bell_chamber",
    )
    for x in (-2.22, 2.22):
        facade.visual(
            Box((0.30, 0.36, 4.6)),
            origin=Origin(xyz=(x, 0.32, 15.2)),
            material=stone_trim,
            name=f"bell_front_quoin_{'left' if x < 0 else 'right'}",
        )
    for side_name, x in (("left", -1.05), ("right", 1.05)):
        facade.visual(
            Box((1.00, 0.18, 1.92)),
            origin=Origin(xyz=(x, 0.41, 15.1)),
            material=dark_opening,
            name=f"bell_opening_{side_name}",
        )
        facade.visual(
            Box((0.12, 0.24, 2.06)),
            origin=Origin(xyz=(x - 0.46, 0.38, 15.1)),
            material=stone_trim,
            name=f"bell_frame_{side_name}_jamb_left",
        )
        facade.visual(
            Box((0.12, 0.24, 2.06)),
            origin=Origin(xyz=(x + 0.46, 0.38, 15.1)),
            material=stone_trim,
            name=f"bell_frame_{side_name}_jamb_right",
        )
        facade.visual(
            Box((1.10, 0.24, 0.16)),
            origin=Origin(xyz=(x, 0.38, 16.05)),
            material=stone_trim,
            name=f"bell_frame_{side_name}_head",
        )

    facade.visual(
        Box((5.1, 5.1, 0.24)),
        origin=Origin(xyz=(0.0, -1.9, 17.44)),
        material=stone_trim,
        name="bell_top_cornice",
    )

    roof_geom = section_loft(
        [
            _square_loop(0.0, -1.9, 17.40, 5.05, 5.05),
            _square_loop(0.0, -1.9, 18.20, 2.90, 2.90),
            _square_loop(0.0, -1.9, 19.00, 0.22, 0.22),
        ]
    )
    facade.visual(_mesh("tower_roof", roof_geom), material=roof_slate, name="tower_roof")
    facade.visual(
        Cylinder(radius=0.07, length=0.80),
        origin=Origin(xyz=(0.0, -1.9, 19.38)),
        material=oxidized_metal,
        name="roof_finial",
    )

    hour_hand = model.part("hour_hand")
    hour_hand.inertial = Inertial.from_geometry(
        Box((0.20, 0.03, 0.86)),
        mass=3.0,
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
    )
    hour_hand.visual(
        Cylinder(radius=0.10, length=0.022),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=oxidized_metal,
        name="hour_hub",
    )
    hour_hand.visual(
        Box((0.16, 0.016, 0.46)),
        origin=Origin(xyz=(0.0, 0.0, 0.19)),
        material=oxidized_metal,
        name="hour_blade",
    )
    hour_hand.visual(
        Box((0.09, 0.016, 0.22)),
        origin=Origin(xyz=(0.0, 0.0, 0.53)),
        material=oxidized_metal,
        name="hour_tip",
    )
    hour_hand.visual(
        Box((0.06, 0.016, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, -0.10)),
        material=oxidized_metal,
        name="hour_counterweight",
    )

    minute_hand = model.part("minute_hand")
    minute_hand.inertial = Inertial.from_geometry(
        Box((0.14, 0.025, 1.30)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
    )
    minute_hand.visual(
        Cylinder(radius=0.072, length=0.018),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=oxidized_metal,
        name="minute_hub",
    )
    minute_hand.visual(
        Box((0.11, 0.012, 0.74)),
        origin=Origin(xyz=(0.0, 0.0, 0.31)),
        material=oxidized_metal,
        name="minute_blade",
    )
    minute_hand.visual(
        Box((0.06, 0.012, 0.34)),
        origin=Origin(xyz=(0.0, 0.0, 0.85)),
        material=oxidized_metal,
        name="minute_tip",
    )
    minute_hand.visual(
        Box((0.045, 0.012, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, -0.13)),
        material=oxidized_metal,
        name="minute_counterweight",
    )

    model.articulation(
        "facade_to_hour_hand",
        ArticulationType.CONTINUOUS,
        parent=facade,
        child=hour_hand,
        origin=Origin(xyz=(0.0, 0.595, clock_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0),
    )
    model.articulation(
        "facade_to_minute_hand",
        ArticulationType.CONTINUOUS,
        parent=facade,
        child=minute_hand,
        origin=Origin(xyz=(0.0, 0.615, clock_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=6.0),
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

    facade = object_model.get_part("facade")
    hour_hand = object_model.get_part("hour_hand")
    minute_hand = object_model.get_part("minute_hand")
    hour_joint = object_model.get_articulation("facade_to_hour_hand")
    minute_joint = object_model.get_articulation("facade_to_minute_hand")
    dial = facade.get_visual("clock_dial")
    bezel = facade.get_visual("clock_bezel")
    spindle = facade.get_visual("clock_spindle")
    hour_hub = hour_hand.get_visual("hour_hub")
    minute_hub = minute_hand.get_visual("minute_hub")

    ctx.check("facade part present", facade is not None)
    ctx.check("hour hand part present", hour_hand is not None)
    ctx.check("minute hand part present", minute_hand is not None)

    hour_origin = hour_joint.origin.xyz
    minute_origin = minute_joint.origin.xyz
    ctx.check(
        "clock hands use separate coaxial continuous joints",
        hour_joint.joint_type == ArticulationType.CONTINUOUS
        and minute_joint.joint_type == ArticulationType.CONTINUOUS
        and hour_joint.axis == (0.0, 1.0, 0.0)
        and minute_joint.axis == (0.0, 1.0, 0.0)
        and abs(hour_origin[0] - minute_origin[0]) < 1e-9
        and abs(hour_origin[2] - minute_origin[2]) < 1e-9
        and 0.01 < (minute_origin[1] - hour_origin[1]) < 0.05
        and hour_joint.motion_limits is not None
        and minute_joint.motion_limits is not None
        and hour_joint.motion_limits.lower is None
        and hour_joint.motion_limits.upper is None
        and minute_joint.motion_limits.lower is None
        and minute_joint.motion_limits.upper is None,
        details=f"hour_origin={hour_origin}, minute_origin={minute_origin}",
    )

    with ctx.pose({hour_joint: 0.0, minute_joint: 0.0}):
        ctx.expect_gap(
            hour_hand,
            facade,
            axis="y",
            min_gap=0.02,
            max_gap=0.08,
            negative_elem=bezel,
            name="hour hand stands off from clock bezel",
        )
        ctx.expect_contact(
            hour_hand,
            facade,
            elem_a=hour_hub,
            elem_b=spindle,
            name="hour hand is mounted on the fixed spindle",
        )
        ctx.expect_contact(
            minute_hand,
            hour_hand,
            elem_a=minute_hub,
            elem_b=hour_hub,
            name="minute hand nests against the hour hand hub",
        )
        ctx.expect_overlap(
            hour_hand,
            facade,
            axes="xz",
            min_overlap=0.12,
            elem_b=dial,
            name="hour hand remains centered on the clock face",
        )
        ctx.expect_overlap(
            minute_hand,
            facade,
            axes="xz",
            min_overlap=0.12,
            elem_b=dial,
            name="minute hand remains centered on the clock face",
        )

        minute_rest_aabb = ctx.part_world_aabb(minute_hand)
        hour_rest_aabb = ctx.part_world_aabb(hour_hand)

    with ctx.pose({minute_joint: pi / 2.0, hour_joint: 0.0}):
        minute_rotated_aabb = ctx.part_world_aabb(minute_hand)
        hour_still_aabb = ctx.part_world_aabb(hour_hand)

    with ctx.pose({minute_joint: 0.0, hour_joint: pi / 2.0}):
        minute_still_aabb = ctx.part_world_aabb(minute_hand)
        hour_rotated_aabb = ctx.part_world_aabb(hour_hand)

    def center_x(aabb) -> float | None:
        if aabb is None:
            return None
        return 0.5 * (aabb[0][0] + aabb[1][0])

    ctx.check(
        "minute hand positive rotation reads clockwise from the front",
        minute_rest_aabb is not None
        and minute_rotated_aabb is not None
        and center_x(minute_rotated_aabb) is not None
        and center_x(minute_rest_aabb) is not None
        and center_x(minute_rotated_aabb) > center_x(minute_rest_aabb) + 0.18,
        details=f"rest={minute_rest_aabb}, rotated={minute_rotated_aabb}",
    )
    ctx.check(
        "hour hand rotates independently of the minute hand",
        hour_rest_aabb is not None
        and hour_rotated_aabb is not None
        and minute_rest_aabb is not None
        and minute_still_aabb is not None
        and center_x(hour_rotated_aabb) is not None
        and center_x(hour_rest_aabb) is not None
        and center_x(minute_still_aabb) is not None
        and center_x(minute_rest_aabb) is not None
        and center_x(hour_rotated_aabb) > center_x(hour_rest_aabb) + 0.10
        and abs(center_x(minute_still_aabb) - center_x(minute_rest_aabb)) < 0.02,
        details=(
            f"hour_rest={hour_rest_aabb}, hour_rotated={hour_rotated_aabb}, "
            f"minute_rest={minute_rest_aabb}, minute_still={minute_still_aabb}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
