from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


TOP_BRACKET_LENGTH = 0.240
TOP_BRACKET_WIDTH = 0.090
TOP_BRACKET_THICKNESS = 0.014

SHAFT_CENTER_Z = -0.078
SHAFT_RADIUS = 0.0155
SHAFT_LENGTH = 0.216

BEARING_CENTER_X = 0.072
BEARING_SPAN_LENGTH = 0.040
BEARING_ROOF_THICKNESS = 0.012
BEARING_ROOF_WIDTH = 0.062
BEARING_CHEEK_THICKNESS = 0.010
BEARING_CHEEK_INNER_Y = SHAFT_RADIUS + 0.004
BEARING_CHEEK_CENTER_Y = BEARING_CHEEK_INNER_Y + BEARING_CHEEK_THICKNESS / 2.0
BEARING_CHEEK_BOTTOM_Z = SHAFT_CENTER_Z - SHAFT_RADIUS - 0.006
BEARING_ROOF_CENTER_Z = SHAFT_CENTER_Z + SHAFT_RADIUS + BEARING_ROOF_THICKNESS / 2.0
BEARING_CHEEK_HEIGHT = (SHAFT_CENTER_Z + SHAFT_RADIUS) - BEARING_CHEEK_BOTTOM_Z
BEARING_CHEEK_CENTER_Z = BEARING_CHEEK_BOTTOM_Z + BEARING_CHEEK_HEIGHT / 2.0

HANGER_WEB_THICKNESS = 0.022
HANGER_WEB_WIDTH = 0.030
HANGER_WEB_BOTTOM_Z = BEARING_ROOF_CENTER_Z + BEARING_ROOF_THICKNESS / 2.0
HANGER_WEB_HEIGHT = (-TOP_BRACKET_THICKNESS / 2.0) - HANGER_WEB_BOTTOM_Z
HANGER_WEB_CENTER_Z = HANGER_WEB_BOTTOM_Z + HANGER_WEB_HEIGHT / 2.0

COLLAR_RADIUS = 0.022
COLLAR_LENGTH = 0.010
COLLAR_GAP = 0.003
COLLAR_CENTER_X = BEARING_CENTER_X + BEARING_SPAN_LENGTH / 2.0 + COLLAR_GAP + COLLAR_LENGTH / 2.0

SETSCREW_BOSS_RADIUS = 0.004
SETSCREW_BOSS_HEIGHT = 0.006

BRACKET_BOTTOM_Z = BEARING_CHEEK_BOTTOM_Z
BRACKET_HEIGHT = TOP_BRACKET_THICKNESS / 2.0 - BRACKET_BOTTOM_Z
BRACKET_CENTER_Z = (TOP_BRACKET_THICKNESS / 2.0 + BRACKET_BOTTOM_Z) / 2.0


def _aabb_center(aabb):
    min_corner, max_corner = aabb
    return tuple((min_corner[i] + max_corner[i]) * 0.5 for i in range(3))


def _x_axis_origin(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(0.0, pi / 2.0, 0.0))


def _add_bearing_support_visuals(model_part, side: str, x_center: float) -> None:
    model_part.visual(
        Box((HANGER_WEB_THICKNESS, HANGER_WEB_WIDTH, HANGER_WEB_HEIGHT)),
        origin=Origin(xyz=(x_center, 0.0, HANGER_WEB_CENTER_Z)),
        material="bracket_dark",
        name=f"{side}_web",
    )
    model_part.visual(
        Box((BEARING_SPAN_LENGTH, BEARING_ROOF_WIDTH, BEARING_ROOF_THICKNESS)),
        origin=Origin(xyz=(x_center, 0.0, BEARING_ROOF_CENTER_Z)),
        material="bearing_gray",
        name=f"{side}_roof",
    )
    model_part.visual(
        Box((BEARING_SPAN_LENGTH, BEARING_CHEEK_THICKNESS, BEARING_CHEEK_HEIGHT)),
        origin=Origin(xyz=(x_center, BEARING_CHEEK_CENTER_Y, BEARING_CHEEK_CENTER_Z)),
        material="bearing_gray",
        name=f"{side}_outer_cheek",
    )
    model_part.visual(
        Box((BEARING_SPAN_LENGTH, BEARING_CHEEK_THICKNESS, BEARING_CHEEK_HEIGHT)),
        origin=Origin(xyz=(x_center, -BEARING_CHEEK_CENTER_Y, BEARING_CHEEK_CENTER_Z)),
        material="bearing_gray",
        name=f"{side}_inner_cheek",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="underslung_rotary_shaft_unit")

    model.material("bracket_dark", rgba=(0.23, 0.25, 0.28, 1.0))
    model.material("bearing_gray", rgba=(0.65, 0.67, 0.70, 1.0))
    model.material("shaft_steel", rgba=(0.44, 0.46, 0.49, 1.0))
    model.material("fastener_black", rgba=(0.10, 0.11, 0.12, 1.0))

    support_bracket = model.part("support_bracket")
    support_bracket.visual(
        Box((TOP_BRACKET_LENGTH, TOP_BRACKET_WIDTH, TOP_BRACKET_THICKNESS)),
        origin=Origin(),
        material="bracket_dark",
        name="top_plate",
    )
    _add_bearing_support_visuals(support_bracket, "left", -BEARING_CENTER_X)
    _add_bearing_support_visuals(support_bracket, "right", BEARING_CENTER_X)
    support_bracket.inertial = Inertial.from_geometry(
        Box((TOP_BRACKET_LENGTH, TOP_BRACKET_WIDTH, BRACKET_HEIGHT)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.0, BRACKET_CENTER_Z)),
    )

    shaft = model.part("shaft")
    shaft.visual(
        Cylinder(radius=SHAFT_RADIUS, length=SHAFT_LENGTH),
        origin=_x_axis_origin((0.0, 0.0, 0.0)),
        material="shaft_steel",
        name="shaft_core",
    )
    shaft.visual(
        Cylinder(radius=COLLAR_RADIUS, length=COLLAR_LENGTH),
        origin=_x_axis_origin((-COLLAR_CENTER_X, 0.0, 0.0)),
        material="shaft_steel",
        name="left_collar",
    )
    shaft.visual(
        Cylinder(radius=COLLAR_RADIUS, length=COLLAR_LENGTH),
        origin=_x_axis_origin((COLLAR_CENTER_X, 0.0, 0.0)),
        material="shaft_steel",
        name="right_collar",
    )
    shaft.visual(
        Cylinder(radius=SETSCREW_BOSS_RADIUS, length=SETSCREW_BOSS_HEIGHT),
        origin=Origin(xyz=(COLLAR_CENTER_X, 0.0, COLLAR_RADIUS + SETSCREW_BOSS_HEIGHT / 2.0)),
        material="fastener_black",
        name="setscrew_boss",
    )
    shaft.inertial = Inertial.from_geometry(
        Cylinder(radius=COLLAR_RADIUS, length=SHAFT_LENGTH),
        mass=1.2,
        origin=_x_axis_origin((0.0, 0.0, 0.0)),
    )

    model.articulation(
        "shaft_spin",
        ArticulationType.CONTINUOUS,
        parent=support_bracket,
        child=shaft,
        origin=Origin(xyz=(0.0, 0.0, SHAFT_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support_bracket = object_model.get_part("support_bracket")
    shaft = object_model.get_part("shaft")
    shaft_spin = object_model.get_articulation("shaft_spin")

    top_plate = support_bracket.get_visual("top_plate")
    left_roof = support_bracket.get_visual("left_roof")
    right_roof = support_bracket.get_visual("right_roof")
    shaft_core = shaft.get_visual("shaft_core")
    setscrew_boss = shaft.get_visual("setscrew_boss")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "shaft_joint_is_continuous_about_x",
        shaft_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(value, 5) for value in shaft_spin.axis) == (1.0, 0.0, 0.0),
        details=f"type={shaft_spin.articulation_type}, axis={shaft_spin.axis}",
    )
    ctx.expect_gap(
        support_bracket,
        shaft,
        axis="z",
        positive_elem=top_plate,
        negative_elem=shaft_core,
        min_gap=0.045,
        max_gap=0.075,
        name="shaft_hangs_below_top_plate",
    )
    ctx.expect_contact(
        shaft,
        support_bracket,
        elem_a=shaft_core,
        elem_b=left_roof,
        name="left_bearing_supports_shaft",
    )
    ctx.expect_contact(
        shaft,
        support_bracket,
        elem_a=shaft_core,
        elem_b=right_roof,
        name="right_bearing_supports_shaft",
    )

    with ctx.pose({shaft_spin: 0.0}):
        setscrew_at_zero = ctx.part_element_world_aabb(shaft, elem=setscrew_boss)
    with ctx.pose({shaft_spin: pi / 2.0}):
        setscrew_at_quarter_turn = ctx.part_element_world_aabb(shaft, elem=setscrew_boss)

    if setscrew_at_zero is None or setscrew_at_quarter_turn is None:
        ctx.fail("setscrew_pose_tracking", "Could not resolve setscrew boss AABB in one or more poses.")
    else:
        zero_center = _aabb_center(setscrew_at_zero)
        quarter_turn_center = _aabb_center(setscrew_at_quarter_turn)
        ctx.check(
            "continuous_shaft_rotation_moves_marker_around_centerline",
            abs(zero_center[0] - quarter_turn_center[0]) < 1e-4
            and zero_center[2] > SHAFT_CENTER_Z + 0.015
            and quarter_turn_center[1] < -0.015
            and abs(quarter_turn_center[2] - SHAFT_CENTER_Z) < 0.010,
            details=(
                f"zero_center={zero_center}, quarter_turn_center={quarter_turn_center}, "
                f"shaft_center_z={SHAFT_CENTER_Z}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
