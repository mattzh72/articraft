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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


FRAME_DEPTH = 2.60
FRAME_WIDTH = 7.20
ROOF_HEIGHT = 2.84
ROOF_THICKNESS = 0.18
THRESHOLD_HEIGHT = 0.10
DRUM_RADIUS = 1.05
DRUM_HEIGHT = 2.56
DRUM_BASE_Z = THRESHOLD_HEIGHT


def _ring_band(name: str, *, outer_radius: float, inner_radius: float, z0: float, z1: float):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(outer_radius, z0), (outer_radius, z1)],
            [(inner_radius, z0), (inner_radius, z1)],
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def _enum_name(value) -> str:
    return getattr(value, "name", str(value))


def _aabb_max_z(aabb) -> float | None:
    if aabb is None:
        return None
    return aabb[1][2]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_wing_revolving_door")

    aluminum = model.material("aluminum", rgba=(0.66, 0.69, 0.73, 1.0))
    dark_aluminum = model.material("dark_aluminum", rgba=(0.32, 0.34, 0.37, 1.0))
    canopy_metal = model.material("canopy_metal", rgba=(0.54, 0.56, 0.60, 1.0))
    glass = model.material("glass", rgba=(0.68, 0.84, 0.92, 0.30))
    smoked_glass = model.material("smoked_glass", rgba=(0.55, 0.72, 0.80, 0.26))
    rubber = model.material("rubber", rgba=(0.11, 0.11, 0.12, 1.0))

    entrance_frame = model.part("entrance_frame")
    entrance_frame.visual(
        Box((FRAME_DEPTH, FRAME_WIDTH, THRESHOLD_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, THRESHOLD_HEIGHT / 2.0)),
        material=dark_aluminum,
        name="threshold_plinth",
    )
    entrance_frame.visual(
        Box((FRAME_DEPTH, FRAME_WIDTH, ROOF_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, ROOF_HEIGHT + (ROOF_THICKNESS / 2.0))),
        material=dark_aluminum,
        name="roof_slab",
    )

    outer_column_size = (0.34, 0.30, ROOF_HEIGHT)
    entrance_frame.visual(
        Box(outer_column_size),
        origin=Origin(xyz=(0.0, 3.45, ROOF_HEIGHT / 2.0)),
        material=aluminum,
        name="left_outer_column",
    )
    entrance_frame.visual(
        Box(outer_column_size),
        origin=Origin(xyz=(0.0, -3.45, ROOF_HEIGHT / 2.0)),
        material=aluminum,
        name="right_outer_column",
    )

    inner_post_size = (0.28, 0.20, ROOF_HEIGHT)
    for name, y_pos in (("left_inner_post", 1.55), ("right_inner_post", -1.55)):
        entrance_frame.visual(
            Box(inner_post_size),
            origin=Origin(xyz=(0.0, y_pos, ROOF_HEIGHT / 2.0)),
            material=aluminum,
            name=name,
        )

    entrance_frame.visual(
        Box((0.16, 3.80, 0.24)),
        origin=Origin(xyz=(1.31, 0.0, 2.72)),
        material=aluminum,
        name="front_header",
    )
    entrance_frame.visual(
        Box((0.16, 3.80, 0.20)),
        origin=Origin(xyz=(-1.31, 0.0, 2.74)),
        material=aluminum,
        name="rear_header",
    )

    entrance_frame.visual(
        Box((0.20, 2.00, 0.10)),
        origin=Origin(xyz=(0.0, 2.50, 2.61)),
        material=dark_aluminum,
        name="left_panel_top_track",
    )
    entrance_frame.visual(
        Box((0.20, 2.00, 0.10)),
        origin=Origin(xyz=(0.0, -2.50, 2.61)),
        material=dark_aluminum,
        name="right_panel_top_track",
    )
    entrance_frame.visual(
        Box((0.08, 2.00, 0.04)),
        origin=Origin(xyz=(0.20, 2.50, 0.12)),
        material=dark_aluminum,
        name="left_panel_floor_guide",
    )
    entrance_frame.visual(
        Box((0.08, 2.00, 0.04)),
        origin=Origin(xyz=(0.20, -2.50, 0.12)),
        material=dark_aluminum,
        name="right_panel_floor_guide",
    )
    entrance_frame.inertial = Inertial.from_geometry(
        Box((FRAME_DEPTH, FRAME_WIDTH, ROOF_HEIGHT + ROOF_THICKNESS)),
        mass=1200.0,
        origin=Origin(xyz=(0.0, 0.0, (ROOF_HEIGHT + ROOF_THICKNESS) / 2.0)),
    )

    drum_enclosure = model.part("drum_enclosure")
    drum_enclosure.visual(
        _ring_band(
            "revolving_door_base_ring",
            outer_radius=DRUM_RADIUS + 0.05,
            inner_radius=DRUM_RADIUS - 0.05,
            z0=DRUM_BASE_Z,
            z1=DRUM_BASE_Z + 0.06,
        ),
        material=dark_aluminum,
        name="base_ring",
    )
    drum_enclosure.visual(
        _ring_band(
            "revolving_door_top_ring",
            outer_radius=DRUM_RADIUS + 0.06,
            inner_radius=DRUM_RADIUS - 0.06,
            z0=DRUM_BASE_Z + DRUM_HEIGHT - 0.12,
            z1=DRUM_BASE_Z + DRUM_HEIGHT,
        ),
        material=dark_aluminum,
        name="top_ring",
    )

    segment_angles_deg = (55.0, 85.0, 115.0, -55.0, -85.0, -115.0)
    for index, angle_deg in enumerate(segment_angles_deg):
        angle = math.radians(angle_deg)
        drum_enclosure.visual(
            Box((0.52, 0.035, DRUM_HEIGHT - 0.12)),
            origin=Origin(
                xyz=(1.025 * math.cos(angle), 1.025 * math.sin(angle), DRUM_BASE_Z + (DRUM_HEIGHT / 2.0)),
                rpy=(0.0, 0.0, angle + (math.pi / 2.0)),
            ),
            material=smoked_glass,
            name=f"drum_segment_{index}",
        )
    drum_enclosure.inertial = Inertial.from_geometry(
        Cylinder(radius=DRUM_RADIUS + 0.06, length=DRUM_HEIGHT),
        mass=180.0,
        origin=Origin(xyz=(0.0, 0.0, DRUM_BASE_Z + (DRUM_HEIGHT / 2.0))),
    )

    rotor = model.part("revolving_drum")
    rotor.visual(
        Cylinder(radius=0.09, length=DRUM_HEIGHT - 0.06),
        origin=Origin(xyz=(0.0, 0.0, (DRUM_HEIGHT - 0.06) / 2.0)),
        material=dark_aluminum,
        name="central_shaft",
    )
    rotor.visual(
        Cylinder(radius=0.16, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=dark_aluminum,
        name="bottom_hub",
    )
    rotor.visual(
        Cylinder(radius=0.16, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, DRUM_HEIGHT - 0.06)),
        material=dark_aluminum,
        name="top_hub",
    )

    for wing_index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        wing_origin = Origin(rpy=(0.0, 0.0, angle))
        rotor.visual(
            Box((0.24, 0.06, DRUM_HEIGHT - 0.24)),
            origin=Origin(
                xyz=(0.21 * math.cos(angle), 0.21 * math.sin(angle), DRUM_HEIGHT / 2.0),
                rpy=wing_origin.rpy,
            ),
            material=aluminum,
            name=f"wing_spine_{wing_index}",
        )
        rotor.visual(
            Box((0.66, 0.035, DRUM_HEIGHT - 0.34)),
            origin=Origin(
                xyz=(0.61 * math.cos(angle), 0.61 * math.sin(angle), DRUM_HEIGHT / 2.0),
                rpy=wing_origin.rpy,
            ),
            material=glass,
            name=f"wing_glass_{wing_index}",
        )
        rotor.visual(
            Box((0.06, 0.06, DRUM_HEIGHT - 0.24)),
            origin=Origin(
                xyz=(0.95 * math.cos(angle), 0.95 * math.sin(angle), DRUM_HEIGHT / 2.0),
                rpy=wing_origin.rpy,
            ),
            material=aluminum,
            name=f"wing_outer_stile_{wing_index}",
        )
        rotor.visual(
            Box((0.74, 0.06, 0.06)),
            origin=Origin(
                xyz=(0.57 * math.cos(angle), 0.57 * math.sin(angle), DRUM_HEIGHT - 0.15),
                rpy=wing_origin.rpy,
            ),
            material=rubber,
            name=f"wing_top_rail_{wing_index}",
        )
        rotor.visual(
            Box((0.74, 0.06, 0.06)),
            origin=Origin(
                xyz=(0.57 * math.cos(angle), 0.57 * math.sin(angle), 0.15),
                rpy=wing_origin.rpy,
            ),
            material=rubber,
            name=f"wing_bottom_rail_{wing_index}",
        )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.98, length=DRUM_HEIGHT),
        mass=95.0,
        origin=Origin(xyz=(0.0, 0.0, DRUM_HEIGHT / 2.0)),
    )

    canopy = model.part("weather_canopy")
    canopy.visual(
        Box((0.98, 2.35, 0.06)),
        origin=Origin(xyz=(0.49, 0.0, -0.035)),
        material=canopy_metal,
        name="canopy_deck",
    )
    canopy.visual(
        Box((0.08, 2.35, 0.18)),
        origin=Origin(xyz=(0.94, 0.0, -0.09)),
        material=canopy_metal,
        name="canopy_front_fascia",
    )
    canopy.visual(
        Box((0.52, 0.08, 0.16)),
        origin=Origin(xyz=(0.26, 0.96, -0.08)),
        material=dark_aluminum,
        name="left_canopy_arm",
    )
    canopy.visual(
        Box((0.52, 0.08, 0.16)),
        origin=Origin(xyz=(0.26, -0.96, -0.08)),
        material=dark_aluminum,
        name="right_canopy_arm",
    )
    canopy.visual(
        Cylinder(radius=0.045, length=2.10),
        origin=Origin(xyz=(0.08, 0.0, -0.01), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_aluminum,
        name="canopy_hinge_barrel",
    )
    canopy.inertial = Inertial.from_geometry(
        Box((1.02, 2.35, 0.22)),
        mass=85.0,
        origin=Origin(xyz=(0.50, 0.0, -0.08)),
    )

    left_bypass = model.part("left_bypass_panel")
    left_bypass.visual(
        Box((0.06, 1.04, 2.22)),
        origin=Origin(xyz=(0.0, 0.0, 1.21)),
        material=glass,
        name="panel_glass",
    )
    left_bypass.visual(
        Box((0.08, 1.10, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 2.39)),
        material=aluminum,
        name="top_rail",
    )
    left_bypass.visual(
        Box((0.08, 1.10, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=aluminum,
        name="bottom_rail",
    )
    left_bypass.visual(
        Box((0.08, 0.08, 2.40)),
        origin=Origin(xyz=(0.0, 0.51, 1.30)),
        material=aluminum,
        name="leading_stile",
    )
    left_bypass.visual(
        Box((0.08, 0.08, 2.40)),
        origin=Origin(xyz=(0.0, -0.51, 1.30)),
        material=aluminum,
        name="trailing_stile",
    )
    left_bypass.inertial = Inertial.from_geometry(
        Box((0.10, 1.12, 2.42)),
        mass=70.0,
        origin=Origin(xyz=(0.0, 0.0, 1.21)),
    )

    right_bypass = model.part("right_bypass_panel")
    right_bypass.visual(
        Box((0.06, 1.04, 2.22)),
        origin=Origin(xyz=(0.0, 0.0, 1.21)),
        material=glass,
        name="panel_glass",
    )
    right_bypass.visual(
        Box((0.08, 1.10, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 2.39)),
        material=aluminum,
        name="top_rail",
    )
    right_bypass.visual(
        Box((0.08, 1.10, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=aluminum,
        name="bottom_rail",
    )
    right_bypass.visual(
        Box((0.08, 0.08, 2.40)),
        origin=Origin(xyz=(0.0, 0.51, 1.30)),
        material=aluminum,
        name="leading_stile",
    )
    right_bypass.visual(
        Box((0.08, 0.08, 2.40)),
        origin=Origin(xyz=(0.0, -0.51, 1.30)),
        material=aluminum,
        name="trailing_stile",
    )
    right_bypass.inertial = Inertial.from_geometry(
        Box((0.10, 1.12, 2.42)),
        mass=70.0,
        origin=Origin(xyz=(0.0, 0.0, 1.21)),
    )

    model.articulation(
        "frame_to_drum_enclosure",
        ArticulationType.FIXED,
        parent=entrance_frame,
        child=drum_enclosure,
        origin=Origin(),
    )
    model.articulation(
        "drum_rotation",
        ArticulationType.CONTINUOUS,
        parent=drum_enclosure,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, DRUM_BASE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2),
    )
    model.articulation(
        "frame_to_weather_canopy",
        ArticulationType.REVOLUTE,
        parent=entrance_frame,
        child=canopy,
        origin=Origin(xyz=(1.39, 0.0, 2.60)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.8,
            lower=0.0,
            upper=math.radians(70.0),
        ),
    )
    model.articulation(
        "frame_to_left_bypass",
        ArticulationType.PRISMATIC,
        parent=entrance_frame,
        child=left_bypass,
        origin=Origin(xyz=(0.0, 2.40, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.35,
            lower=0.0,
            upper=0.30,
        ),
    )
    model.articulation(
        "frame_to_right_bypass",
        ArticulationType.PRISMATIC,
        parent=entrance_frame,
        child=right_bypass,
        origin=Origin(xyz=(0.0, -2.40, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.35,
            lower=0.0,
            upper=0.30,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("entrance_frame")
    enclosure = object_model.get_part("drum_enclosure")
    rotor = object_model.get_part("revolving_drum")
    canopy = object_model.get_part("weather_canopy")
    left_bypass = object_model.get_part("left_bypass_panel")
    right_bypass = object_model.get_part("right_bypass_panel")

    drum_rotation = object_model.get_articulation("drum_rotation")
    canopy_joint = object_model.get_articulation("frame_to_weather_canopy")
    left_slide = object_model.get_articulation("frame_to_left_bypass")
    right_slide = object_model.get_articulation("frame_to_right_bypass")

    primary_parts_ok = all(
        part is not None
        for part in (frame, enclosure, rotor, canopy, left_bypass, right_bypass)
    )
    ctx.check("primary parts exist", primary_parts_ok, details="One or more primary parts were not resolved.")

    ctx.check(
        "articulation types and axes match the requested mechanisms",
        _enum_name(drum_rotation.joint_type) == "CONTINUOUS"
        and tuple(drum_rotation.axis) == (0.0, 0.0, 1.0)
        and _enum_name(canopy_joint.joint_type) == "REVOLUTE"
        and tuple(canopy_joint.axis) == (0.0, -1.0, 0.0)
        and _enum_name(left_slide.joint_type) == "PRISMATIC"
        and tuple(left_slide.axis) == (0.0, 1.0, 0.0)
        and _enum_name(right_slide.joint_type) == "PRISMATIC"
        and tuple(right_slide.axis) == (0.0, -1.0, 0.0),
        details=(
            f"drum={_enum_name(drum_rotation.joint_type)} axis={drum_rotation.axis}, "
            f"canopy={_enum_name(canopy_joint.joint_type)} axis={canopy_joint.axis}, "
            f"left={_enum_name(left_slide.joint_type)} axis={left_slide.axis}, "
            f"right={_enum_name(right_slide.joint_type)} axis={right_slide.axis}"
        ),
    )

    with ctx.pose({drum_rotation: 0.0}):
        ctx.expect_within(
            rotor,
            enclosure,
            axes="xy",
            margin=0.03,
            name="revolving drum stays within enclosure footprint at rest",
        )

    with ctx.pose({drum_rotation: math.radians(40.0)}):
        ctx.expect_within(
            rotor,
            enclosure,
            axes="xy",
            margin=0.03,
            name="revolving drum stays within enclosure footprint when rotated",
        )

    canopy_closed_top = _aabb_max_z(ctx.part_element_world_aabb(canopy, elem="canopy_front_fascia"))
    with ctx.pose({canopy_joint: math.radians(55.0)}):
        canopy_open_top = _aabb_max_z(ctx.part_element_world_aabb(canopy, elem="canopy_front_fascia"))
    ctx.check(
        "canopy tilts upward for maintenance access",
        canopy_closed_top is not None
        and canopy_open_top is not None
        and canopy_open_top > canopy_closed_top + 0.45,
        details=f"closed_top={canopy_closed_top}, open_top={canopy_open_top}",
    )

    left_closed = ctx.part_world_position(left_bypass)
    right_closed = ctx.part_world_position(right_bypass)
    with ctx.pose({left_slide: 0.30, right_slide: 0.30}):
        left_open = ctx.part_world_position(left_bypass)
        right_open = ctx.part_world_position(right_bypass)
        ctx.expect_gap(
            frame,
            left_bypass,
            axis="y",
            min_gap=0.03,
            positive_elem="left_outer_column",
            negative_elem="leading_stile",
            name="left bypass panel clears the outer column when open",
        )
        ctx.expect_gap(
            right_bypass,
            frame,
            axis="y",
            min_gap=0.03,
            positive_elem="trailing_stile",
            negative_elem="right_outer_column",
            name="right bypass panel clears the outer column when open",
        )

    ctx.check(
        "left bypass panel slides outward to the left",
        left_closed is not None
        and left_open is not None
        and left_open[1] > left_closed[1] + 0.20,
        details=f"closed={left_closed}, open={left_open}",
    )
    ctx.check(
        "right bypass panel slides outward to the right",
        right_closed is not None
        and right_open is not None
        and right_open[1] < right_closed[1] - 0.20,
        details=f"closed={right_closed}, open={right_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
