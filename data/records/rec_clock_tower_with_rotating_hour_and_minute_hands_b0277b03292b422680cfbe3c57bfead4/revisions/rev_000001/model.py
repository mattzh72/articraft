from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, pi, sin, cos, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    place_on_surface,
)


def _add_xz_brace(
    part,
    *,
    x0: float,
    z0: float,
    x1: float,
    z1: float,
    y: float,
    width: float,
    depth: float,
    material,
    name: str | None = None,
) -> None:
    dx = x1 - x0
    dz = z1 - z0
    length = (dx * dx + dz * dz) ** 0.5
    part.visual(
        Box((width, depth, length)),
        origin=Origin(
            xyz=((x0 + x1) * 0.5, y, (z0 + z1) * 0.5),
            rpy=(0.0, atan2(dx, dz), 0.0),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="timber_frame_clock_tower")

    dark_oak = model.material("dark_oak", rgba=(0.28, 0.19, 0.11, 1.0))
    weathered_wood = model.material("weathered_wood", rgba=(0.56, 0.43, 0.29, 1.0))
    roof_slate = model.material("roof_slate", rgba=(0.22, 0.22, 0.24, 1.0))
    roof_trim = model.material("roof_trim", rgba=(0.32, 0.25, 0.17, 1.0))
    dial_cream = model.material("dial_cream", rgba=(0.92, 0.90, 0.84, 1.0))
    aged_bronze = model.material("aged_bronze", rgba=(0.47, 0.38, 0.20, 1.0))
    hand_black = model.material("hand_black", rgba=(0.10, 0.10, 0.10, 1.0))

    shaft_width = 1.78
    shaft_depth = 1.48
    shaft_height = 5.80
    roof_width = 2.16
    roof_height = 1.55
    clock_center_z = 4.35
    dial_radius = 0.54

    tower = model.part("tower")
    tower.inertial = Inertial.from_geometry(
        Box((2.02, 1.72, shaft_height)),
        mass=2600.0,
        origin=Origin(xyz=(0.0, 0.0, shaft_height * 0.5)),
    )

    shaft_skin = tower.visual(
        Box((shaft_width, shaft_depth, shaft_height)),
        origin=Origin(xyz=(0.0, 0.0, shaft_height * 0.5)),
        material=weathered_wood,
        name="shaft_skin",
    )
    tower.visual(
        Box((2.02, 1.72, 0.36)),
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        material=dark_oak,
        name="plinth",
    )
    tower.visual(
        Box((1.94, 1.64, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, shaft_height - 0.09)),
        material=dark_oak,
        name="cap_band",
    )

    post_w = 0.14
    post_d = 0.14
    post_x = shaft_width * 0.5 - post_w * 0.5
    post_y = shaft_depth * 0.5 - post_d * 0.5
    for sx in (-post_x, post_x):
        for sy in (-post_y, post_y):
            tower.visual(
                Box((post_w, post_d, shaft_height - 0.02)),
                origin=Origin(xyz=(sx, sy, shaft_height * 0.5)),
                material=dark_oak,
            )

    front_y = shaft_depth * 0.5 + 0.04
    side_x = shaft_width * 0.5 + 0.04
    rail_levels = (0.95, 2.05, 3.15, 5.25)
    for z in rail_levels:
        tower.visual(
            Box((1.52, 0.08, 0.10)),
            origin=Origin(xyz=(0.0, front_y, z)),
            material=dark_oak,
        )
        tower.visual(
            Box((1.52, 0.08, 0.10)),
            origin=Origin(xyz=(0.0, -front_y, z)),
            material=dark_oak,
        )
        tower.visual(
            Box((0.08, 1.24, 0.10)),
            origin=Origin(xyz=(side_x, 0.0, z)),
            material=dark_oak,
        )
        tower.visual(
            Box((0.08, 1.24, 0.10)),
            origin=Origin(xyz=(-side_x, 0.0, z)),
            material=dark_oak,
        )

    _add_xz_brace(
        tower,
        x0=-0.52,
        z0=0.44,
        x1=0.12,
        z1=1.42,
        y=front_y,
        width=0.08,
        depth=0.08,
        material=dark_oak,
        name="front_lower_brace_left",
    )
    _add_xz_brace(
        tower,
        x0=0.52,
        z0=0.44,
        x1=-0.12,
        z1=1.42,
        y=front_y,
        width=0.08,
        depth=0.08,
        material=dark_oak,
        name="front_lower_brace_right",
    )
    _add_xz_brace(
        tower,
        x0=-0.48,
        z0=1.86,
        x1=0.10,
        z1=2.86,
        y=-front_y,
        width=0.08,
        depth=0.08,
        material=dark_oak,
        name="rear_mid_brace_left",
    )
    _add_xz_brace(
        tower,
        x0=0.48,
        z0=1.86,
        x1=-0.10,
        z1=2.86,
        y=-front_y,
        width=0.08,
        depth=0.08,
        material=dark_oak,
        name="rear_mid_brace_right",
    )

    roof = model.part("roof")
    roof.inertial = Inertial.from_geometry(
        Box((roof_width, roof_width, roof_height + 0.24)),
        mass=420.0,
        origin=Origin(xyz=(0.0, 0.0, 0.82)),
    )
    roof_mesh = mesh_from_geometry(
        ConeGeometry(
            radius=roof_width / sqrt(2.0),
            height=roof_height,
            radial_segments=4,
            closed=True,
        )
        .rotate_z(pi / 4.0)
        .translate(0.0, 0.0, roof_height * 0.5 + 0.01),
        "clock_tower_roof_shell",
    )
    roof.visual(roof_mesh, material=roof_slate, name="roof_shell")
    roof.visual(
        Box((roof_width + 0.10, roof_width + 0.10, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=roof_trim,
        name="eave_plate",
    )
    roof.visual(
        Cylinder(radius=0.05, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, roof_height + 0.11)),
        material=aged_bronze,
        name="finial",
    )

    face_angle = atan2(roof_height, roof_width * 0.5)
    for course_index in range(7):
        z = 0.18 + course_index * 0.16
        taper = max(0.14, 1.0 - z / roof_height)
        span = roof_width * taper - 0.10
        plane_offset = roof_width * 0.5 * taper
        roof.visual(
            Box((span, 0.18, 0.028)),
            origin=Origin(xyz=(0.0, plane_offset, z), rpy=(-face_angle, 0.0, 0.0)),
            material=roof_slate,
        )
        roof.visual(
            Box((span, 0.18, 0.028)),
            origin=Origin(xyz=(0.0, -plane_offset, z), rpy=(face_angle, 0.0, 0.0)),
            material=roof_slate,
        )
        roof.visual(
            Box((0.18, span, 0.028)),
            origin=Origin(xyz=(plane_offset, 0.0, z), rpy=(0.0, face_angle, 0.0)),
            material=roof_slate,
        )
        roof.visual(
            Box((0.18, span, 0.028)),
            origin=Origin(xyz=(-plane_offset, 0.0, z), rpy=(0.0, -face_angle, 0.0)),
            material=roof_slate,
        )

    clock_face = model.part("clock_face")
    clock_face.inertial = Inertial.from_geometry(
        Box((1.30, 0.10, 1.30)),
        mass=55.0,
        origin=Origin(xyz=(0.0, 0.03, 0.0)),
    )
    cyl_y = Origin(rpy=(-pi / 2.0, 0.0, 0.0))
    clock_face.visual(
        Cylinder(radius=dial_radius + 0.08, length=0.04),
        origin=Origin(xyz=(0.0, 0.02, 0.0), rpy=cyl_y.rpy),
        material=roof_trim,
        name="face_backplate",
    )
    clock_face.visual(
        Cylinder(radius=dial_radius + 0.04, length=0.05),
        origin=Origin(xyz=(0.0, 0.028, 0.0), rpy=cyl_y.rpy),
        material=aged_bronze,
        name="bezel",
    )
    clock_face.visual(
        Cylinder(radius=dial_radius, length=0.018),
        origin=Origin(xyz=(0.0, 0.034, 0.0), rpy=cyl_y.rpy),
        material=dial_cream,
        name="dial_face",
    )
    clock_face.visual(
        Cylinder(radius=0.046, length=0.026),
        origin=Origin(xyz=(0.0, 0.048, 0.0), rpy=cyl_y.rpy),
        material=aged_bronze,
        name="center_boss",
    )
    for tick_index in range(12):
        angle = 2.0 * pi * tick_index / 12.0
        tick_radius = 0.43
        tick_len = 0.11 if tick_index % 3 == 0 else 0.07
        tick_w = 0.028 if tick_index % 3 == 0 else 0.018
        clock_face.visual(
            Box((tick_w, 0.010, tick_len)),
            origin=Origin(
                xyz=(sin(angle) * tick_radius, 0.041, cos(angle) * tick_radius),
                rpy=(0.0, angle, 0.0),
            ),
            material=hand_black,
        )

    hour_hand = model.part("hour_hand")
    hour_hand.inertial = Inertial.from_geometry(
        Box((0.10, 0.012, 0.34)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.07, 0.13)),
    )
    hour_hand.visual(
        Cylinder(radius=0.038, length=0.018),
        origin=Origin(xyz=(0.0, 0.070, 0.0), rpy=cyl_y.rpy),
        material=hand_black,
        name="hour_hub",
    )
    hour_hand.visual(
        Box((0.042, 0.008, 0.18)),
        origin=Origin(xyz=(0.0, 0.070, 0.09)),
        material=hand_black,
        name="hour_blade",
    )
    hour_hand.visual(
        Box((0.026, 0.008, 0.11)),
        origin=Origin(xyz=(0.0, 0.070, 0.235)),
        material=hand_black,
    )
    hour_hand.visual(
        Cylinder(radius=0.016, length=0.008),
        origin=Origin(xyz=(0.0, 0.070, 0.30), rpy=cyl_y.rpy),
        material=hand_black,
        name="hour_tip",
    )
    hour_hand.visual(
        Box((0.030, 0.008, 0.10)),
        origin=Origin(xyz=(0.0, 0.070, -0.05)),
        material=hand_black,
    )

    minute_hand = model.part("minute_hand")
    minute_hand.inertial = Inertial.from_geometry(
        Box((0.11, 0.012, 0.48)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.087, 0.18)),
    )
    minute_hand.visual(
        Cylinder(radius=0.034, length=0.016),
        origin=Origin(xyz=(0.0, 0.087, 0.0), rpy=cyl_y.rpy),
        material=hand_black,
        name="minute_hub",
    )
    minute_hand.visual(
        Box((0.032, 0.008, 0.23)),
        origin=Origin(xyz=(0.0, 0.087, 0.115)),
        material=hand_black,
        name="minute_blade",
    )
    minute_hand.visual(
        Box((0.022, 0.008, 0.17)),
        origin=Origin(xyz=(0.0, 0.087, 0.315)),
        material=hand_black,
    )
    minute_hand.visual(
        Cylinder(radius=0.014, length=0.008),
        origin=Origin(xyz=(0.0, 0.087, 0.405), rpy=cyl_y.rpy),
        material=hand_black,
        name="minute_tip",
    )
    minute_hand.visual(
        Box((0.024, 0.008, 0.12)),
        origin=Origin(xyz=(0.0, 0.087, -0.06)),
        material=hand_black,
    )

    model.articulation(
        "tower_to_roof",
        ArticulationType.FIXED,
        parent=tower,
        child=roof,
        origin=Origin(xyz=(0.0, 0.0, shaft_height)),
    )
    model.articulation(
        "tower_to_clock_face",
        ArticulationType.FIXED,
        parent=tower,
        child=clock_face,
        origin=place_on_surface(
            clock_face,
            shaft_skin,
            point_hint=(0.0, shaft_depth * 0.5, clock_center_z),
            child_axis="+y",
            prefer_collisions=False,
            child_prefer_collisions=False,
        ),
    )
    model.articulation(
        "clock_to_hour_hand",
        ArticulationType.CONTINUOUS,
        parent=clock_face,
        child=hour_hand,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0),
    )
    model.articulation(
        "clock_to_minute_hand",
        ArticulationType.CONTINUOUS,
        parent=clock_face,
        child=minute_hand,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=10.0),
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
    roof = object_model.get_part("roof")
    clock_face = object_model.get_part("clock_face")
    hour_hand = object_model.get_part("hour_hand")
    minute_hand = object_model.get_part("minute_hand")
    hour_joint = object_model.get_articulation("clock_to_hour_hand")
    minute_joint = object_model.get_articulation("clock_to_minute_hand")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    with ctx.pose({hour_joint: 0.0, minute_joint: 0.0}):
        ctx.expect_gap(
            clock_face,
            tower,
            axis="y",
            positive_elem="face_backplate",
            negative_elem="shaft_skin",
            max_gap=0.002,
            max_penetration=0.0,
            name="clock face mounts flush to the timber shaft",
        )
        ctx.expect_overlap(
            clock_face,
            tower,
            axes="xz",
            elem_a="dial_face",
            elem_b="shaft_skin",
            min_overlap=1.00,
            name="clock dial stays centered on the front bay",
        )
        ctx.expect_gap(
            roof,
            tower,
            axis="z",
            positive_elem="eave_plate",
            negative_elem="cap_band",
            max_gap=0.003,
            max_penetration=0.0,
            name="roof seats on the tower cap",
        )
        ctx.expect_gap(
            hour_hand,
            clock_face,
            axis="y",
            positive_elem="hour_blade",
            negative_elem="dial_face",
            min_gap=0.004,
            max_gap=0.030,
            name="hour hand stands proud of the dial",
        )
        ctx.expect_gap(
            minute_hand,
            clock_face,
            axis="y",
            positive_elem="minute_blade",
            negative_elem="dial_face",
            min_gap=0.014,
            max_gap=0.050,
            name="minute hand clears the dial face",
        )
        ctx.expect_gap(
            minute_hand,
            hour_hand,
            axis="y",
            positive_elem="minute_blade",
            negative_elem="hour_blade",
            min_gap=0.004,
            max_gap=0.030,
            name="minute hand sits ahead of the hour hand",
        )
        ctx.expect_origin_distance(
            minute_hand,
            hour_hand,
            axes="xyz",
            max_dist=0.001,
            name="hour and minute hands share the same arbor",
        )

    dial_center = ctx.part_world_position(clock_face)
    hour_tip_rest = _aabb_center(ctx.part_element_world_aabb(hour_hand, elem="hour_tip"))
    minute_tip_rest = _aabb_center(ctx.part_element_world_aabb(minute_hand, elem="minute_tip"))

    ctx.check(
        "hour hand starts at twelve",
        dial_center is not None
        and hour_tip_rest is not None
        and abs(hour_tip_rest[0] - dial_center[0]) < 0.04
        and hour_tip_rest[2] > dial_center[2] + 0.22,
        details=f"dial_center={dial_center}, hour_tip_rest={hour_tip_rest}",
    )
    ctx.check(
        "minute hand starts at twelve",
        dial_center is not None
        and minute_tip_rest is not None
        and abs(minute_tip_rest[0] - dial_center[0]) < 0.04
        and minute_tip_rest[2] > dial_center[2] + 0.32,
        details=f"dial_center={dial_center}, minute_tip_rest={minute_tip_rest}",
    )

    with ctx.pose({minute_joint: pi / 2.0, hour_joint: 0.0}):
        minute_tip_quarter = _aabb_center(ctx.part_element_world_aabb(minute_hand, elem="minute_tip"))
        hour_tip_hold = _aabb_center(ctx.part_element_world_aabb(hour_hand, elem="hour_tip"))

    ctx.check(
        "minute hand rotates clockwise to three o'clock",
        dial_center is not None
        and minute_tip_quarter is not None
        and minute_tip_quarter[0] > dial_center[0] + 0.30
        and abs(minute_tip_quarter[2] - dial_center[2]) < 0.08,
        details=f"dial_center={dial_center}, minute_tip_quarter={minute_tip_quarter}",
    )
    ctx.check(
        "hour hand stays put when only the minute joint turns",
        dial_center is not None
        and hour_tip_hold is not None
        and abs(hour_tip_hold[0] - dial_center[0]) < 0.04
        and hour_tip_hold[2] > dial_center[2] + 0.22,
        details=f"dial_center={dial_center}, hour_tip_hold={hour_tip_hold}",
    )

    with ctx.pose({minute_joint: 0.0, hour_joint: pi}):
        minute_tip_hold = _aabb_center(ctx.part_element_world_aabb(minute_hand, elem="minute_tip"))
        hour_tip_half = _aabb_center(ctx.part_element_world_aabb(hour_hand, elem="hour_tip"))

    ctx.check(
        "hour hand rotates independently to six o'clock",
        dial_center is not None
        and hour_tip_half is not None
        and abs(hour_tip_half[0] - dial_center[0]) < 0.05
        and hour_tip_half[2] < dial_center[2] - 0.20,
        details=f"dial_center={dial_center}, hour_tip_half={hour_tip_half}",
    )
    ctx.check(
        "minute hand stays put when only the hour joint turns",
        dial_center is not None
        and minute_tip_hold is not None
        and abs(minute_tip_hold[0] - dial_center[0]) < 0.04
        and minute_tip_hold[2] > dial_center[2] + 0.32,
        details=f"dial_center={dial_center}, minute_tip_hold={minute_tip_hold}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
