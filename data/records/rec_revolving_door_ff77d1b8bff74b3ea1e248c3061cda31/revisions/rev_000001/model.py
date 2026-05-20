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
    sweep_profile_along_spline,
)


def _arc_path(
    radius: float,
    start_angle: float,
    end_angle: float,
    z: float,
    *,
    segments: int = 20,
) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    for index in range(segments + 1):
        t = index / segments
        angle = start_angle + (end_angle - start_angle) * t
        points.append((radius * math.cos(angle), radius * math.sin(angle), z))
    return points


def _build_wing(
    model: ArticulatedObject,
    name: str,
    *,
    frame_material,
    glass_material,
    rail_material,
) -> object:
    wing = model.part(name)

    panel_span = 0.86
    wing_height = 2.14
    panel_mid_z = 0.02 + wing_height * 0.5

    wing.visual(
        Cylinder(radius=0.009, length=wing_height),
        origin=Origin(xyz=(0.0, 0.0, panel_mid_z)),
        material=frame_material,
        name="hinge_barrel",
    )
    wing.visual(
        Box((0.036, 0.048, wing_height)),
        origin=Origin(xyz=(0.027, 0.0, panel_mid_z)),
        material=frame_material,
        name="root_stile",
    )
    wing.visual(
        Box((0.038, 0.046, wing_height)),
        origin=Origin(xyz=(panel_span - 0.019, 0.0, panel_mid_z)),
        material=frame_material,
        name="outer_stile",
    )
    wing.visual(
        Box((panel_span - 0.045, 0.032, 0.052)),
        origin=Origin(xyz=(0.045 + (panel_span - 0.045) * 0.5, 0.0, 0.02 + 0.026)),
        material=frame_material,
        name="bottom_rail",
    )
    wing.visual(
        Box((panel_span - 0.045, 0.032, 0.052)),
        origin=Origin(
            xyz=(
                0.045 + (panel_span - 0.045) * 0.5,
                0.0,
                0.02 + wing_height - 0.026,
            )
        ),
        material=frame_material,
        name="top_rail",
    )
    wing.visual(
        Box((panel_span - 0.11, 0.014, wing_height - 0.14)),
        origin=Origin(xyz=(0.46, 0.0, panel_mid_z)),
        material=glass_material,
        name="glass_panel",
    )
    wing.visual(
        Box((panel_span - 0.12, 0.030, 0.046)),
        origin=Origin(xyz=(0.46, 0.0, 1.03)),
        material=rail_material,
        name="push_bar",
    )

    wing.inertial = Inertial.from_geometry(
        Box((panel_span, 0.05, wing_height)),
        mass=48.0,
        origin=Origin(xyz=(panel_span * 0.5, 0.0, panel_mid_z)),
    )
    return wing


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="four_wing_revolving_door")

    frame_metal = model.material("frame_metal", rgba=(0.57, 0.60, 0.63, 1.0))
    canopy_metal = model.material("canopy_metal", rgba=(0.34, 0.36, 0.39, 1.0))
    threshold_metal = model.material("threshold_metal", rgba=(0.22, 0.23, 0.25, 1.0))
    glass = model.material("glass", rgba=(0.72, 0.84, 0.92, 0.35))
    wing_rail = model.material("wing_rail", rgba=(0.18, 0.19, 0.21, 1.0))

    drum_radius = 1.18
    wall_radius = 1.06
    wall_thickness = 0.05
    clear_height = 2.24
    canopy_thickness = 0.18
    canopy_center_z = clear_height + canopy_thickness * 0.5

    frame = model.part("frame")
    frame.visual(
        Cylinder(radius=drum_radius, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=threshold_metal,
        name="threshold_plate",
    )
    frame.visual(
        Cylinder(radius=drum_radius, length=canopy_thickness),
        origin=Origin(xyz=(0.0, 0.0, canopy_center_z)),
        material=canopy_metal,
        name="canopy_disk",
    )

    wall_profile = rounded_rect_profile(wall_thickness, clear_height, radius=0.008, corner_segments=6)
    left_wall_mesh = mesh_from_geometry(
        sweep_profile_along_spline(
            _arc_path(wall_radius, math.radians(55.0), math.radians(125.0), clear_height * 0.5, segments=18),
            profile=wall_profile,
            samples_per_segment=10,
            cap_profile=True,
        ),
        "left_drum_wall",
    )
    right_wall_mesh = mesh_from_geometry(
        sweep_profile_along_spline(
            _arc_path(wall_radius, math.radians(235.0), math.radians(305.0), clear_height * 0.5, segments=18),
            profile=wall_profile,
            samples_per_segment=10,
            cap_profile=True,
        ),
        "right_drum_wall",
    )
    frame.visual(left_wall_mesh, material=glass, name="left_wall")
    frame.visual(right_wall_mesh, material=glass, name="right_wall")

    mullion_radius = 0.02
    for index, angle_deg in enumerate((55.0, 90.0, 125.0, 235.0, 270.0, 305.0)):
        angle = math.radians(angle_deg)
        frame.visual(
            Cylinder(radius=mullion_radius, length=clear_height),
            origin=Origin(
                xyz=(wall_radius * math.cos(angle), wall_radius * math.sin(angle), clear_height * 0.5)
            ),
            material=frame_metal,
            name=f"mullion_{index:02d}",
        )

    frame.inertial = Inertial.from_geometry(
        Box((drum_radius * 2.0, drum_radius * 2.0, clear_height + canopy_thickness)),
        mass=820.0,
        origin=Origin(xyz=(0.0, 0.0, (clear_height + canopy_thickness - 0.05) * 0.5 - 0.025)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.055, length=2.11),
        origin=Origin(xyz=(0.0, 0.0, 1.105)),
        material=frame_metal,
        name="central_post",
    )
    rotor.visual(
        Cylinder(radius=0.095, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=canopy_metal,
        name="lower_bearing_ring",
    )
    rotor.visual(
        Cylinder(radius=0.095, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 2.20)),
        material=canopy_metal,
        name="upper_bearing_ring",
    )
    rotor.visual(
        Box((0.202, 0.03, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=canopy_metal,
        name="lower_spider_x",
    )
    rotor.visual(
        Box((0.03, 0.202, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=canopy_metal,
        name="lower_spider_y",
    )
    rotor.visual(
        Box((0.202, 0.03, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 2.19)),
        material=canopy_metal,
        name="upper_spider_x",
    )
    rotor.visual(
        Box((0.03, 0.202, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 2.19)),
        material=canopy_metal,
        name="upper_spider_y",
    )
    rotor.inertial = Inertial.from_geometry(
        Box((0.20, 0.20, 2.24)),
        mass=160.0,
        origin=Origin(xyz=(0.0, 0.0, 1.12)),
    )

    model.articulation(
        "frame_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=1.6),
    )

    wing_east = _build_wing(
        model,
        "wing_east",
        frame_material=frame_metal,
        glass_material=glass,
        rail_material=wing_rail,
    )
    wing_north = _build_wing(
        model,
        "wing_north",
        frame_material=frame_metal,
        glass_material=glass,
        rail_material=wing_rail,
    )
    wing_west = _build_wing(
        model,
        "wing_west",
        frame_material=frame_metal,
        glass_material=glass,
        rail_material=wing_rail,
    )
    wing_south = _build_wing(
        model,
        "wing_south",
        frame_material=frame_metal,
        glass_material=glass,
        rail_material=wing_rail,
    )

    hinge_radius = 0.11
    breakout_limits = MotionLimits(
        effort=90.0,
        velocity=1.4,
        lower=-1.35,
        upper=1.35,
    )
    for name, child, angle in (
        ("rotor_to_wing_east", wing_east, 0.0),
        ("rotor_to_wing_north", wing_north, math.pi * 0.5),
        ("rotor_to_wing_west", wing_west, math.pi),
        ("rotor_to_wing_south", wing_south, -math.pi * 0.5),
    ):
        model.articulation(
            name,
            ArticulationType.REVOLUTE,
            parent=rotor,
            child=child,
            origin=Origin(
                xyz=(hinge_radius * math.cos(angle), hinge_radius * math.sin(angle), 0.0),
                rpy=(0.0, 0.0, angle),
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=breakout_limits,
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

    frame = object_model.get_part("frame")
    rotor = object_model.get_part("rotor")
    wing_east = object_model.get_part("wing_east")
    wing_north = object_model.get_part("wing_north")
    wing_west = object_model.get_part("wing_west")
    wing_south = object_model.get_part("wing_south")

    rotor_spin = object_model.get_articulation("frame_to_rotor")
    breakout_east = object_model.get_articulation("rotor_to_wing_east")
    breakout_north = object_model.get_articulation("rotor_to_wing_north")
    breakout_west = object_model.get_articulation("rotor_to_wing_west")
    breakout_south = object_model.get_articulation("rotor_to_wing_south")

    def elem_center(part, elem: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        min_corner, max_corner = aabb
        return tuple((min_corner[i] + max_corner[i]) * 0.5 for i in range(3))

    rotor_limits = rotor_spin.motion_limits
    ctx.check(
        "rotor uses a vertical continuous articulation",
        rotor_spin.articulation_type == ArticulationType.CONTINUOUS
        and rotor_spin.axis == (0.0, 0.0, 1.0)
        and rotor_limits is not None
        and rotor_limits.lower is None
        and rotor_limits.upper is None,
        details=(
            f"type={rotor_spin.articulation_type}, axis={rotor_spin.axis}, "
            f"limits={None if rotor_limits is None else (rotor_limits.lower, rotor_limits.upper)}"
        ),
    )

    for joint in (breakout_east, breakout_north, breakout_west, breakout_south):
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name} is a vertical break-out hinge",
            joint.articulation_type == ArticulationType.REVOLUTE
            and joint.axis == (0.0, 0.0, 1.0)
            and limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower < 0.0 < limits.upper,
            details=(
                f"type={joint.articulation_type}, axis={joint.axis}, "
                f"limits={None if limits is None else (limits.lower, limits.upper)}"
            ),
        )

    ctx.expect_contact(
        frame,
        rotor,
        elem_a="threshold_plate",
        elem_b="lower_bearing_ring",
        name="lower rotor bearing seats on threshold plate",
    )
    ctx.expect_contact(
        frame,
        rotor,
        elem_a="canopy_disk",
        elem_b="upper_bearing_ring",
        name="upper rotor bearing seats under canopy",
    )

    ctx.expect_gap(
        wing_east,
        frame,
        axis="z",
        negative_elem="threshold_plate",
        min_gap=0.015,
        max_gap=0.03,
        name="east wing clears threshold",
    )
    ctx.expect_gap(
        frame,
        wing_east,
        axis="z",
        positive_elem="canopy_disk",
        min_gap=0.06,
        max_gap=0.12,
        name="east wing clears canopy",
    )

    rest_root = ctx.part_world_position(wing_east)
    with ctx.pose({rotor_spin: math.pi * 0.5}):
        quarter_turn_root = ctx.part_world_position(wing_east)
    ctx.check(
        "rotor carries the wings around the center post",
        rest_root is not None
        and quarter_turn_root is not None
        and quarter_turn_root[1] > 0.09
        and abs(quarter_turn_root[0]) < 0.03,
        details=f"rest={rest_root}, quarter_turn={quarter_turn_root}",
    )

    rest_outer = elem_center(wing_east, "outer_stile")
    with ctx.pose({breakout_east: 1.0}):
        opened_outer = elem_center(wing_east, "outer_stile")
    ctx.check(
        "east wing break-out hinge swings the panel laterally",
        rest_outer is not None
        and opened_outer is not None
        and opened_outer[1] > rest_outer[1] + 0.45,
        details=f"rest_outer={rest_outer}, opened_outer={opened_outer}",
    )

    ctx.check(
        "exactly four revolving wings are present",
        len([wing_east, wing_north, wing_west, wing_south]) == 4,
        details="Expected four cardinal wing parts on the rotor.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
