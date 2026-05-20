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
    TorusGeometry,
    mesh_from_geometry,
)


FLOOR_TOP = 0.12
ROOF_BOTTOM = 3.08
VESTIBULE_HALF_DEPTH = 1.75
VESTIBULE_HALF_WIDTH = 2.25
SIDE_WALL_Y = 2.20
DOOR_PLANE_Y = 2.12
DOOR_OPENING_HALF = 0.56
DOOR_WIDTH = 1.05
DOOR_HEIGHT = 2.86
DRUM_JOINT_Z = 0.18
DRUM_HEIGHT = 2.84
DRUM_OUTER_RADIUS = 1.24


def _aabb_center(aabb):
    if aabb is None:
        return None
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    return (
        0.5 * (min_x + max_x),
        0.5 * (min_y + max_y),
        0.5 * (min_z + max_z),
    )


def _add_bypass_door(part, *, swing_sign: float, frame_material, glass_material, accent_material, prefix: str) -> None:
    stile_width = 0.06
    rail_height = 0.08
    leaf_thickness = 0.055
    glass_width = DOOR_WIDTH - 2.0 * stile_width
    glass_height = DOOR_HEIGHT - 2.0 * rail_height

    part.visual(
        Box((glass_width, 0.016, glass_height)),
        origin=Origin(
            xyz=(swing_sign * (0.5 * stile_width + 0.5 * glass_width), 0.0, rail_height + 0.5 * glass_height)
        ),
        material=glass_material,
        name=f"{prefix}_glass",
    )
    part.visual(
        Box((stile_width, leaf_thickness, DOOR_HEIGHT)),
        origin=Origin(xyz=(swing_sign * (0.5 * stile_width), 0.0, 0.5 * DOOR_HEIGHT)),
        material=frame_material,
        name=f"{prefix}_hinge_stile",
    )
    part.visual(
        Box((stile_width, leaf_thickness, DOOR_HEIGHT)),
        origin=Origin(xyz=(swing_sign * (DOOR_WIDTH - 0.5 * stile_width), 0.0, 0.5 * DOOR_HEIGHT)),
        material=frame_material,
        name=f"{prefix}_free_stile",
    )
    part.visual(
        Box((DOOR_WIDTH, leaf_thickness, rail_height)),
        origin=Origin(xyz=(swing_sign * (0.5 * DOOR_WIDTH), 0.0, 0.5 * rail_height)),
        material=frame_material,
        name=f"{prefix}_bottom_rail",
    )
    part.visual(
        Box((DOOR_WIDTH, leaf_thickness, rail_height)),
        origin=Origin(xyz=(swing_sign * (0.5 * DOOR_WIDTH), 0.0, DOOR_HEIGHT - 0.5 * rail_height)),
        material=frame_material,
        name=f"{prefix}_top_rail",
    )
    part.visual(
        Box((0.025, 0.045, 1.00)),
        origin=Origin(xyz=(swing_sign * (0.74 * DOOR_WIDTH), 0.0, 1.42)),
        material=accent_material,
        name=f"{prefix}_pull_stile",
    )
    part.visual(
        Box((0.42, 0.018, 0.05)),
        origin=Origin(xyz=(swing_sign * (0.53 * DOOR_WIDTH), 0.0, 1.42)),
        material=accent_material,
        name=f"{prefix}_push_bar",
    )


def _add_rotor_wing(part, *, angle: float, index: int, frame_material, glass_material) -> None:
    yaw = angle
    c = math.cos(angle)
    s = math.sin(angle)
    radial_length = 1.20
    glass_length = 1.14
    panel_height = 2.42
    rail_height = 0.08
    stile_depth = 0.06
    glass_thickness = 0.022
    outer_stile_x = radial_length - 0.04
    panel_z = 1.41

    part.visual(
        Box((glass_length, glass_thickness, panel_height)),
        origin=Origin(xyz=(c * 0.5 * glass_length, s * 0.5 * glass_length, panel_z), rpy=(0.0, 0.0, yaw)),
        material=glass_material,
        name=f"wing_{index}_glass",
    )
    part.visual(
        Box((radial_length, stile_depth, rail_height)),
        origin=Origin(xyz=(c * 0.5 * radial_length, s * 0.5 * radial_length, 0.16), rpy=(0.0, 0.0, yaw)),
        material=frame_material,
        name=f"wing_{index}_bottom_rail",
    )
    part.visual(
        Box((radial_length, stile_depth, rail_height)),
        origin=Origin(xyz=(c * 0.5 * radial_length, s * 0.5 * radial_length, 2.68), rpy=(0.0, 0.0, yaw)),
        material=frame_material,
        name=f"wing_{index}_top_rail",
    )
    part.visual(
        Box((0.08, 0.08, 2.52)),
        origin=Origin(xyz=(c * outer_stile_x, s * outer_stile_x, 1.42), rpy=(0.0, 0.0, yaw)),
        material=frame_material,
        name=f"wing_{index}_outer_stile",
    )


def _build_track_ring(name: str, radius: float, tube: float):
    return mesh_from_geometry(
        TorusGeometry(radius=radius, tube=tube, radial_segments=16, tubular_segments=72),
        name,
    )


def _add_drum_wall_panel(part, *, angle_deg: float, radius: float, glass_material, frame_material, name: str) -> None:
    angle = math.radians(angle_deg)
    c = math.cos(angle)
    s = math.sin(angle)
    panel_width = 0.43
    panel_thickness = 0.024
    panel_height = ROOF_BOTTOM - FLOOR_TOP
    center = (radius * c, radius * s, FLOOR_TOP + 0.5 * panel_height)
    tangent_yaw = angle + 0.5 * math.pi

    part.visual(
        Box((panel_width, panel_thickness, panel_height)),
        origin=Origin(xyz=center, rpy=(0.0, 0.0, tangent_yaw)),
        material=glass_material,
        name=f"{name}_glass",
    )
    part.visual(
        Box((0.06, 0.07, panel_height)),
        origin=Origin(xyz=center, rpy=(0.0, 0.0, tangent_yaw)),
        material=frame_material,
        name=f"{name}_mullion",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hotel_grand_entrance_revolving_door")

    bronze = model.material("bronze", rgba=(0.41, 0.30, 0.20, 1.0))
    dark_bronze = model.material("dark_bronze", rgba=(0.19, 0.14, 0.10, 1.0))
    stone = model.material("stone", rgba=(0.72, 0.70, 0.67, 1.0))
    glass = model.material("glass", rgba=(0.63, 0.80, 0.89, 0.30))
    glass_dark = model.material("glass_dark", rgba=(0.47, 0.68, 0.78, 0.40))
    steel = model.material("steel", rgba=(0.71, 0.72, 0.74, 1.0))

    floor_track = _build_track_ring("floor_track_ring", radius=1.40, tube=0.035)
    header_track = _build_track_ring("header_track_ring", radius=1.40, tube=0.035)

    vestibule = model.part("vestibule")
    vestibule.visual(
        Box((3.70, 4.70, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=stone,
        name="floor_slab",
    )
    vestibule.visual(
        Box((3.32, 4.34, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, FLOOR_TOP - 0.01)),
        material=dark_bronze,
        name="threshold_band",
    )
    vestibule.visual(
        Box((3.70, 4.70, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, ROOF_BOTTOM + 0.09)),
        material=bronze,
        name="roof_canopy",
    )
    vestibule.visual(
        floor_track,
        origin=Origin(xyz=(0.0, 0.0, FLOOR_TOP + 0.035)),
        material=dark_bronze,
        name="floor_track",
    )
    vestibule.visual(
        header_track,
        origin=Origin(xyz=(0.0, 0.0, ROOF_BOTTOM - 0.035)),
        material=dark_bronze,
        name="header_track",
    )
    vestibule.visual(
        Cylinder(radius=0.12, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=steel,
        name="floor_pivot",
    )
    vestibule.visual(
        Cylinder(radius=0.12, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 3.05)),
        material=steel,
        name="ceiling_bearing",
    )
    post_height = ROOF_BOTTOM - FLOOR_TOP
    post_center_z = FLOOR_TOP + 0.5 * post_height
    for panel_index, angle_deg in enumerate((-130.0, -108.0, -86.0, -64.0, -42.0, 42.0, 64.0, 86.0, 108.0, 130.0)):
        _add_drum_wall_panel(
            vestibule,
            angle_deg=angle_deg,
            radius=DRUM_OUTER_RADIUS,
            glass_material=glass_dark,
            frame_material=bronze,
            name=f"drum_wall_{panel_index}",
        )
    for angle_deg, name in ((-42.0, "front_right_drum_jamb"), (42.0, "front_left_drum_jamb"), (138.0, "rear_left_drum_jamb"), (-138.0, "rear_right_drum_jamb")):
        angle = math.radians(angle_deg)
        vestibule.visual(
            Cylinder(radius=0.045, length=post_height),
            origin=Origin(xyz=(DRUM_OUTER_RADIUS * math.cos(angle), DRUM_OUTER_RADIUS * math.sin(angle), post_center_z)),
            material=dark_bronze,
            name=name,
        )
    for x in (-1.60, 1.60):
        for y in (-SIDE_WALL_Y, SIDE_WALL_Y):
            vestibule.visual(
                Cylinder(radius=0.05, length=post_height),
                origin=Origin(xyz=(x, y, post_center_z)),
                material=bronze,
            )

    for side_sign in (-1.0, 1.0):
        wall_y = side_sign * SIDE_WALL_Y
        hinge_x = DOOR_OPENING_HALF if side_sign < 0.0 else -DOOR_OPENING_HALF
        strike_x = -DOOR_OPENING_HALF if side_sign < 0.0 else DOOR_OPENING_HALF
        prefix = "left_wall" if side_sign < 0.0 else "right_wall"

        vestibule.visual(
            Cylinder(radius=0.04, length=post_height),
            origin=Origin(xyz=(hinge_x, wall_y, post_center_z)),
            material=dark_bronze,
            name=f"{prefix}_hinge_post",
        )
        vestibule.visual(
            Box((0.06, 0.10, post_height)),
            origin=Origin(xyz=(strike_x, wall_y, post_center_z)),
            material=bronze,
            name=f"{prefix}_strike_post",
        )
        vestibule.visual(
            Box((0.90, 0.02, post_height)),
            origin=Origin(xyz=(1.05, wall_y, post_center_z)),
            material=glass_dark,
            name=f"{prefix}_front_sidelite",
        )
        vestibule.visual(
            Box((0.90, 0.02, post_height)),
            origin=Origin(xyz=(-1.05, wall_y, post_center_z)),
            material=glass_dark,
            name=f"{prefix}_rear_sidelite",
        )

    for x in (-1.58, 1.58):
        face_name = "rear" if x < 0.0 else "front"
        vestibule.visual(
            Box((0.02, 0.78, post_height)),
            origin=Origin(xyz=(x, -1.72, post_center_z)),
            material=glass_dark,
            name=f"{face_name}_left_screen",
        )
        vestibule.visual(
            Box((0.02, 0.78, post_height)),
            origin=Origin(xyz=(x, 1.72, post_center_z)),
            material=glass_dark,
            name=f"{face_name}_right_screen",
        )

    vestibule.inertial = Inertial.from_geometry(
        Box((3.70, 4.70, 3.26)),
        mass=1400.0,
        origin=Origin(xyz=(0.0, 0.0, 1.63)),
    )

    drum = model.part("revolving_drum")
    drum.visual(
        Cylinder(radius=0.15, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=dark_bronze,
        name="bottom_hub",
    )
    drum.visual(
        Cylinder(radius=0.065, length=2.72),
        origin=Origin(xyz=(0.0, 0.0, 1.36)),
        material=steel,
        name="central_shaft",
    )
    drum.visual(
        Cylinder(radius=0.24, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material=dark_bronze,
        name="lower_spider",
    )
    drum.visual(
        Cylinder(radius=0.24, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 2.68)),
        material=dark_bronze,
        name="upper_spider",
    )
    drum.visual(
        Cylinder(radius=0.15, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 2.78)),
        material=dark_bronze,
        name="top_cap",
    )
    for index, angle in enumerate((0.0, 0.5 * math.pi, math.pi, 1.5 * math.pi)):
        _add_rotor_wing(
            drum,
            angle=angle,
            index=index,
            frame_material=bronze,
            glass_material=glass,
        )
    drum.inertial = Inertial.from_geometry(
        Box((2.60, 2.60, DRUM_HEIGHT)),
        mass=260.0,
        origin=Origin(xyz=(0.0, 0.0, 0.5 * DRUM_HEIGHT)),
    )

    left_door = model.part("left_bypass_door")
    _add_bypass_door(
        left_door,
        swing_sign=-1.0,
        frame_material=bronze,
        glass_material=glass,
        accent_material=dark_bronze,
        prefix="left_door",
    )
    left_door.inertial = Inertial.from_geometry(
        Box((DOOR_WIDTH, 0.06, DOOR_HEIGHT)),
        mass=85.0,
        origin=Origin(xyz=(-0.5 * DOOR_WIDTH, 0.0, 0.5 * DOOR_HEIGHT)),
    )

    right_door = model.part("right_bypass_door")
    _add_bypass_door(
        right_door,
        swing_sign=1.0,
        frame_material=bronze,
        glass_material=glass,
        accent_material=dark_bronze,
        prefix="right_door",
    )
    right_door.inertial = Inertial.from_geometry(
        Box((DOOR_WIDTH, 0.06, DOOR_HEIGHT)),
        mass=85.0,
        origin=Origin(xyz=(0.5 * DOOR_WIDTH, 0.0, 0.5 * DOOR_HEIGHT)),
    )

    model.articulation(
        "drum_rotation",
        ArticulationType.CONTINUOUS,
        parent=vestibule,
        child=drum,
        origin=Origin(xyz=(0.0, 0.0, DRUM_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=250.0, velocity=1.2),
    )
    model.articulation(
        "left_bypass_swing",
        ArticulationType.REVOLUTE,
        parent=vestibule,
        child=left_door,
        origin=Origin(xyz=(DOOR_OPENING_HALF, -DOOR_PLANE_Y, FLOOR_TOP)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.6,
            lower=0.0,
            upper=math.radians(115.0),
        ),
    )
    model.articulation(
        "right_bypass_swing",
        ArticulationType.REVOLUTE,
        parent=vestibule,
        child=right_door,
        origin=Origin(xyz=(-DOOR_OPENING_HALF, DOOR_PLANE_Y, FLOOR_TOP)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.6,
            lower=0.0,
            upper=math.radians(115.0),
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
    vestibule = object_model.get_part("vestibule")
    drum = object_model.get_part("revolving_drum")
    left_door = object_model.get_part("left_bypass_door")
    right_door = object_model.get_part("right_bypass_door")

    drum_joint = object_model.get_articulation("drum_rotation")
    left_joint = object_model.get_articulation("left_bypass_swing")
    right_joint = object_model.get_articulation("right_bypass_swing")

    ctx.expect_contact(
        drum,
        vestibule,
        elem_a="bottom_hub",
        elem_b="floor_pivot",
        contact_tol=1e-4,
        name="drum is seated on the floor pivot",
    )
    ctx.expect_contact(
        drum,
        vestibule,
        elem_a="top_cap",
        elem_b="ceiling_bearing",
        contact_tol=1e-4,
        name="drum is captured by the ceiling bearing",
    )

    wing_rest = _aabb_center(ctx.part_element_world_aabb(drum, elem="wing_0_outer_stile"))
    with ctx.pose({drum_joint: 0.5 * math.pi}):
        wing_quarter = _aabb_center(ctx.part_element_world_aabb(drum, elem="wing_0_outer_stile"))
    ctx.check(
        "drum rotates a quarter turn about the vertical shaft",
        wing_rest is not None
        and wing_quarter is not None
        and wing_rest[0] > 1.0
        and abs(wing_rest[1]) < 0.10
        and wing_quarter[1] > 1.0
        and abs(wing_quarter[0]) < 0.10,
        details=f"rest={wing_rest}, quarter_turn={wing_quarter}",
    )

    left_rest = _aabb_center(ctx.part_element_world_aabb(left_door, elem="left_door_free_stile"))
    with ctx.pose({left_joint: left_joint.motion_limits.upper}):
        left_open = _aabb_center(ctx.part_element_world_aabb(left_door, elem="left_door_free_stile"))
    ctx.check(
        "left bypass door swings outward from the side wall",
        left_rest is not None
        and left_open is not None
        and left_open[1] < left_rest[1] - 0.45,
        details=f"rest={left_rest}, open={left_open}",
    )

    right_rest = _aabb_center(ctx.part_element_world_aabb(right_door, elem="right_door_free_stile"))
    with ctx.pose({right_joint: right_joint.motion_limits.upper}):
        right_open = _aabb_center(ctx.part_element_world_aabb(right_door, elem="right_door_free_stile"))
    ctx.check(
        "right bypass door swings outward from the side wall",
        right_rest is not None
        and right_open is not None
        and right_open[1] > right_rest[1] + 0.45,
        details=f"rest={right_rest}, open={right_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
