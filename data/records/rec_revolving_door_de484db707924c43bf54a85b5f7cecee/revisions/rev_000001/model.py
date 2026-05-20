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
)


FLOOR_THICKNESS = 0.04
FLOOR_TOP = FLOOR_THICKNESS
CANOPY_UNDERSIDE = 2.54
CANOPY_THICKNESS = 0.18
CANOPY_TOP = CANOPY_UNDERSIDE + CANOPY_THICKNESS
GLASS_HEIGHT = 2.46
GLASS_THICKNESS = 0.012
OUTER_SIDE = 3.20
OUTER_HALF = OUTER_SIDE * 0.5
PORTAL_WIDTH = 1.50
CORNER_LEG = (OUTER_SIDE - PORTAL_WIDTH) * 0.5
CORNER_PANEL_LENGTH = CORNER_LEG * math.sqrt(2.0)
CORNER_PANEL_CENTER = OUTER_HALF - CORNER_LEG * 0.5
HUB_SIZE = 0.24
SUPPORT_TIP_RADIUS = 0.18
WING_LENGTH = 1.28


def _rotated_box_origin(
    x: float,
    y: float,
    z: float,
    *,
    yaw_deg: float = 0.0,
) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(0.0, 0.0, math.radians(yaw_deg)))


def _quadrant_direction(yaw_deg: float, radius: float) -> tuple[float, float]:
    yaw = math.radians(yaw_deg)
    return (radius * math.cos(yaw), radius * math.sin(yaw))


def _make_corner_panel(
    model: ArticulatedObject,
    *,
    name: str,
    xy: tuple[float, float],
    yaw_deg: float,
    glass_material,
    metal_material,
) -> None:
    panel = model.part(name)
    panel.visual(
        Box((CORNER_PANEL_LENGTH, GLASS_THICKNESS, GLASS_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, GLASS_HEIGHT * 0.5)),
        material=glass_material,
        name="corner_glass",
    )
    panel.visual(
        Box((CORNER_PANEL_LENGTH, 0.08, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=metal_material,
        name="bottom_channel",
    )
    panel.visual(
        Box((CORNER_PANEL_LENGTH, 0.08, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, GLASS_HEIGHT - 0.0225)),
        material=metal_material,
        name="top_channel",
    )
    panel.inertial = Inertial.from_geometry(
        Box((CORNER_PANEL_LENGTH, 0.08, GLASS_HEIGHT)),
        mass=55.0,
        origin=Origin(xyz=(0.0, 0.0, GLASS_HEIGHT * 0.5)),
    )
    model.articulation(
        f"drum_structure_to_{name}",
        ArticulationType.FIXED,
        parent="drum_structure",
        child=panel,
        origin=_rotated_box_origin(xy[0], xy[1], FLOOR_TOP, yaw_deg=yaw_deg),
    )


def _make_wing(
    model: ArticulatedObject,
    *,
    name: str,
    yaw_deg: float,
    glass_material,
    metal_material,
) -> None:
    wing = model.part(name)
    wing.visual(
        Box((WING_LENGTH, GLASS_THICKNESS, GLASS_HEIGHT)),
        origin=Origin(xyz=(WING_LENGTH * 0.5, 0.0, GLASS_HEIGHT * 0.5)),
        material=glass_material,
        name="wing_glass",
    )
    wing.visual(
        Box((0.08, 0.034, GLASS_HEIGHT)),
        origin=Origin(xyz=(0.04, 0.0, GLASS_HEIGHT * 0.5)),
        material=metal_material,
        name="carrier_strip",
    )
    wing.visual(
        Box((0.14, 0.09, 0.06)),
        origin=Origin(xyz=(0.07, 0.0, 0.03)),
        material=metal_material,
        name="bottom_patch",
    )
    wing.visual(
        Box((0.14, 0.09, 0.06)),
        origin=Origin(xyz=(0.07, 0.0, GLASS_HEIGHT - 0.03)),
        material=metal_material,
        name="top_patch",
    )
    wing.inertial = Inertial.from_geometry(
        Box((WING_LENGTH, 0.09, GLASS_HEIGHT)),
        mass=72.0,
        origin=Origin(xyz=(WING_LENGTH * 0.5, 0.0, GLASS_HEIGHT * 0.5)),
    )

    mount_x, mount_y = _quadrant_direction(yaw_deg, SUPPORT_TIP_RADIUS)
    model.articulation(
        f"rotor_to_{name}",
        ArticulationType.FIXED,
        parent="wing_rotor",
        child=wing,
        origin=_rotated_box_origin(mount_x, mount_y, 0.0, yaw_deg=yaw_deg),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="square_glass_revolving_door")

    stainless = model.material("stainless", rgba=(0.62, 0.65, 0.69, 1.0))
    canopy_metal = model.material("canopy_metal", rgba=(0.45, 0.48, 0.52, 1.0))
    floor_stone = model.material("floor_stone", rgba=(0.30, 0.31, 0.33, 1.0))
    clear_glass = model.material("clear_glass", rgba=(0.75, 0.88, 0.96, 0.25))

    drum_structure = model.part("drum_structure")
    drum_structure.visual(
        Box((3.30, 3.30, FLOOR_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, FLOOR_THICKNESS * 0.5)),
        material=floor_stone,
        name="floor_slab",
    )
    drum_structure.visual(
        Box((3.30, 3.30, CANOPY_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, CANOPY_UNDERSIDE + CANOPY_THICKNESS * 0.5)),
        material=canopy_metal,
        name="canopy",
    )
    drum_structure.visual(
        Box((0.28, 0.28, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, CANOPY_UNDERSIDE + 0.05)),
        material=stainless,
        name="bearing_housing",
    )
    post_half_span = 1.56
    post_height = CANOPY_UNDERSIDE - FLOOR_TOP
    post_center_z = FLOOR_TOP + post_height * 0.5
    for x in (-post_half_span, post_half_span):
        for y in (-post_half_span, post_half_span):
            drum_structure.visual(
                Box((0.08, 0.08, post_height)),
                origin=Origin(xyz=(x, y, post_center_z)),
                material=stainless,
                name=f"corner_post_{'p' if x > 0 else 'n'}x_{'p' if y > 0 else 'n'}y",
            )
    drum_structure.inertial = Inertial.from_geometry(
        Box((3.30, 3.30, CANOPY_TOP)),
        mass=650.0,
        origin=Origin(xyz=(0.0, 0.0, CANOPY_TOP * 0.5)),
    )

    _make_corner_panel(
        model,
        name="corner_panel_front_left",
        xy=(-CORNER_PANEL_CENTER, CORNER_PANEL_CENTER),
        yaw_deg=45.0,
        glass_material=clear_glass,
        metal_material=stainless,
    )
    _make_corner_panel(
        model,
        name="corner_panel_front_right",
        xy=(CORNER_PANEL_CENTER, CORNER_PANEL_CENTER),
        yaw_deg=-45.0,
        glass_material=clear_glass,
        metal_material=stainless,
    )
    _make_corner_panel(
        model,
        name="corner_panel_back_right",
        xy=(CORNER_PANEL_CENTER, -CORNER_PANEL_CENTER),
        yaw_deg=45.0,
        glass_material=clear_glass,
        metal_material=stainless,
    )
    _make_corner_panel(
        model,
        name="corner_panel_back_left",
        xy=(-CORNER_PANEL_CENTER, -CORNER_PANEL_CENTER),
        yaw_deg=-45.0,
        glass_material=clear_glass,
        metal_material=stainless,
    )

    rotor = model.part("wing_rotor")
    rotor.visual(
        Box((HUB_SIZE, HUB_SIZE, GLASS_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, GLASS_HEIGHT * 0.5)),
        material=stainless,
        name="hub_post",
    )
    rotor.visual(
        Cylinder(radius=0.11, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=stainless,
        name="bottom_bearing",
    )
    rotor.visual(
        Cylinder(radius=0.11, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, GLASS_HEIGHT - 0.04)),
        material=stainless,
        name="top_bearing",
    )
    rotor.inertial = Inertial.from_geometry(
        Box((0.28, 0.28, GLASS_HEIGHT)),
        mass=180.0,
        origin=Origin(xyz=(0.0, 0.0, GLASS_HEIGHT * 0.5)),
    )

    model.articulation(
        "drum_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=drum_structure,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, FLOOR_TOP)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.5),
    )

    _make_wing(
        model,
        name="wing_front_left",
        yaw_deg=135.0,
        glass_material=clear_glass,
        metal_material=stainless,
    )
    _make_wing(
        model,
        name="wing_front_right",
        yaw_deg=45.0,
        glass_material=clear_glass,
        metal_material=stainless,
    )
    _make_wing(
        model,
        name="wing_back_right",
        yaw_deg=-45.0,
        glass_material=clear_glass,
        metal_material=stainless,
    )
    _make_wing(
        model,
        name="wing_back_left",
        yaw_deg=-135.0,
        glass_material=clear_glass,
        metal_material=stainless,
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

    drum_structure = object_model.get_part("drum_structure")
    rotor = object_model.get_part("wing_rotor")
    front_right_wing = object_model.get_part("wing_front_right")
    front_left_wing = object_model.get_part("wing_front_left")
    rotor_joint = object_model.get_articulation("drum_to_rotor")

    ctx.check(
        "continuous hub rotation authored",
        rotor_joint.articulation_type == ArticulationType.CONTINUOUS
        and rotor_joint.motion_limits is not None
        and rotor_joint.motion_limits.lower is None
        and rotor_joint.motion_limits.upper is None
        and tuple(rotor_joint.axis) == (0.0, 0.0, 1.0),
        details=(
            f"type={rotor_joint.articulation_type}, "
            f"axis={rotor_joint.axis}, limits={rotor_joint.motion_limits}"
        ),
    )
    ctx.expect_origin_distance(
        rotor,
        drum_structure,
        axes="xy",
        max_dist=0.001,
        name="rotor stays centered in square drum",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
        return (
            (min_x + max_x) * 0.5,
            (min_y + max_y) * 0.5,
            (min_z + max_z) * 0.5,
        )

    rest_center = _aabb_center(
        ctx.part_element_world_aabb(front_right_wing, elem="wing_glass")
    )
    ctx.check(
        "front-right wing initially points to drum corner",
        rest_center is not None
        and rest_center[0] > 0.55
        and rest_center[1] > 0.55
        and abs(rest_center[0] - rest_center[1]) < 0.08,
        details=f"rest_center={rest_center}",
    )

    canopy_aabb = ctx.part_element_world_aabb(drum_structure, elem="canopy")
    wing_aabb = ctx.part_element_world_aabb(front_left_wing, elem="wing_glass")
    canopy_ok = (
        canopy_aabb is not None
        and wing_aabb is not None
        and wing_aabb[0][2] >= FLOOR_TOP - 1e-6
        and wing_aabb[1][2] <= canopy_aabb[0][2] - 0.03
    )
    ctx.check(
        "wing glass stays between floor and canopy",
        canopy_ok,
        details=f"wing_aabb={wing_aabb}, canopy_aabb={canopy_aabb}",
    )

    with ctx.pose({rotor_joint: math.pi / 4.0}):
        turned_center = _aabb_center(
            ctx.part_element_world_aabb(front_right_wing, elem="wing_glass")
        )
        ctx.check(
            "continuous rotation swings a wing into a side opening",
            turned_center is not None
            and abs(turned_center[0]) < 0.08
            and turned_center[1] > 0.79,
            details=f"turned_center={turned_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
