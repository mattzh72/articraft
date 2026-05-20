from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import os
import math
from pathlib import Path


_REAL_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _REAL_GETCWD()
    except FileNotFoundError:
        os.chdir("/")
        return "/"


os.getcwd = _safe_getcwd

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

SAFE_ASSET_ROOT = Path("/")
SAFE_ASSETS = AssetContext(SAFE_ASSET_ROOT)
os.chdir(SAFE_ASSET_ROOT)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="yard_ramp", assets=SAFE_ASSETS)

    steel = model.material("steel", rgba=(0.42, 0.45, 0.48, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.23, 0.25, 0.28, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    deck_length = 2.45
    deck_width = 0.72
    deck_thickness = 0.03
    deck_pitch = math.radians(17.0)
    deck_visual_pitch = -deck_pitch
    deck_center_z = 0.53
    stringer_width = 0.07
    stringer_height = 0.16
    hinge_radius = 0.026
    lip_length = 0.26
    lip_width = 0.64
    lip_thickness = 0.016
    wheel_radius = 0.135
    wheel_width = 0.04
    wheel_hub_radius = 0.055
    axle_x = -1.02
    axle_z = 0.165
    axle_y = 0.41

    def deck_point(along: float, normal: float = 0.0, lateral: float = 0.0) -> tuple[float, float, float]:
        return (
            along * math.cos(deck_pitch) - normal * math.sin(deck_pitch),
            lateral,
            deck_center_z + along * math.sin(deck_pitch) + normal * math.cos(deck_pitch),
        )

    deck = model.part("deck")
    deck.inertial = Inertial.from_geometry(
        Box((2.60, 0.84, 0.24)),
        mass=320.0,
        origin=Origin(xyz=(0.0, 0.0, 0.42)),
    )

    deck.visual(
        Box((deck_length, deck_width, deck_thickness)),
        origin=Origin(xyz=(0.0, 0.0, deck_center_z), rpy=(0.0, deck_visual_pitch, 0.0)),
        material=steel,
        name="deck_plate",
    )
    for y_sign, name in ((1.0, "left_stringer"), (-1.0, "right_stringer")):
        deck.visual(
            Box((deck_length * 0.96, stringer_width, stringer_height)),
            origin=Origin(
                xyz=deck_point(
                    0.0,
                    -(deck_thickness * 0.5 + stringer_height * 0.5 - 0.012),
                    y_sign * (deck_width * 0.5 - stringer_width * 0.5 - 0.01),
                ),
                rpy=(0.0, deck_visual_pitch, 0.0),
            ),
            material=dark_steel,
            name=name,
        )

    for along, length, name in (
        (-0.76, 0.10, "rear_crossmember"),
        (-0.12, 0.09, "center_crossmember"),
        (0.60, 0.10, "front_crossmember"),
    ):
        deck.visual(
            Box((length, deck_width - 0.14, 0.07)),
            origin=Origin(
                xyz=deck_point(along, -(deck_thickness * 0.5 + 0.02)),
                rpy=(0.0, deck_visual_pitch, 0.0),
            ),
            material=dark_steel,
            name=name,
        )

    deck.visual(
        Box((0.22, deck_width - 0.10, 0.022)),
        origin=Origin(
            xyz=deck_point(-0.20, deck_thickness * 0.5 + 0.010),
            rpy=(0.0, deck_visual_pitch, 0.0),
        ),
        material=dark_steel,
        name="mid_rib",
    )

    for y_sign, side_name in ((1.0, "left"), (-1.0, "right")):
        deck.visual(
            Box((0.09, 0.04, 0.24)),
            origin=Origin(xyz=(axle_x, y_sign * 0.37, 0.23)),
            material=dark_steel,
            name=f"{side_name}_axle_bracket",
        )
        deck.visual(
            Box((0.20, 0.04, 0.05)),
            origin=Origin(xyz=(axle_x + 0.04, y_sign * 0.35, 0.32)),
            material=dark_steel,
            name=f"{side_name}_axle_saddle",
        )

    hinge_origin = deck_point(deck_length * 0.5 + 0.04, deck_thickness * 0.5 - hinge_radius)
    deck.visual(
        Cylinder(radius=hinge_radius, length=0.16),
        origin=Origin(xyz=(hinge_origin[0], -0.19, hinge_origin[2]), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=dark_steel,
        name="deck_left_barrel",
    )
    deck.visual(
        Cylinder(radius=hinge_radius, length=0.16),
        origin=Origin(xyz=(hinge_origin[0], 0.19, hinge_origin[2]), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=dark_steel,
        name="deck_right_barrel",
    )
    deck.visual(
        Box((0.12, deck_width - 0.22, 0.06)),
        origin=Origin(xyz=(1.11, 0.0, 0.83)),
        material=dark_steel,
        name="front_hinge_beam",
    )
    for y_sign, name in ((1.0, "left_hinge_support"), (-1.0, "right_hinge_support")):
        deck.visual(
            Box((0.10, 0.08, 0.12)),
            origin=Origin(xyz=(1.14, y_sign * 0.19, 0.855)),
            material=dark_steel,
            name=name,
        )
    lip = model.part("lip")
    lip.inertial = Inertial.from_geometry(
        Box((lip_length, lip_width, 0.05)),
        mass=42.0,
        origin=Origin(xyz=(0.19, 0.0, 0.02)),
    )
    lip.visual(
        Box((lip_length, lip_width, lip_thickness)),
        origin=Origin(xyz=(0.19, 0.0, hinge_radius - lip_thickness * 0.5)),
        material=steel,
        name="lip_plate",
    )
    lip.visual(
        Box((0.17, lip_width - 0.20, 0.018)),
        origin=Origin(xyz=(0.17, 0.0, hinge_radius - lip_thickness - 0.009)),
        material=dark_steel,
        name="lip_rib",
    )
    lip.visual(
        Cylinder(radius=hinge_radius, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=dark_steel,
        name="lip_center_barrel",
    )
    lip.visual(
        Box((0.16, 0.16, 0.018)),
        origin=Origin(xyz=(0.10, 0.0, 0.015)),
        material=dark_steel,
        name="lip_hinge_connector",
    )

    left_wheel = model.part("left_wheel")
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=wheel_width),
        mass=24.0,
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
    )
    left_wheel.visual(
        Cylinder(radius=wheel_radius, length=wheel_width),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=rubber,
        name="tire",
    )
    left_wheel.visual(
        Cylinder(radius=wheel_hub_radius, length=wheel_width),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=steel,
        name="hub",
    )
    left_wheel.visual(
        Cylinder(radius=0.028, length=0.01),
        origin=Origin(xyz=(0.0, wheel_width * 0.5, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=dark_steel,
        name="outer_cap",
    )

    right_wheel = model.part("right_wheel")
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=wheel_width),
        mass=24.0,
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
    )
    right_wheel.visual(
        Cylinder(radius=wheel_radius, length=wheel_width),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=rubber,
        name="tire",
    )
    right_wheel.visual(
        Cylinder(radius=wheel_hub_radius, length=wheel_width),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=steel,
        name="hub",
    )
    right_wheel.visual(
        Cylinder(radius=0.028, length=0.01),
        origin=Origin(xyz=(0.0, -wheel_width * 0.5, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=dark_steel,
        name="outer_cap",
    )

    model.articulation(
        "front_lip_hinge",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=lip,
        origin=Origin(xyz=hinge_origin, rpy=(0.0, deck_visual_pitch, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=1.2, lower=-0.65, upper=0.20),
    )
    model.articulation(
        "left_wheel_axle",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=left_wheel,
        origin=Origin(xyz=(axle_x, axle_y, axle_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=10.0),
    )
    model.articulation(
        "right_wheel_axle",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=right_wheel,
        origin=Origin(xyz=(axle_x, -axle_y, axle_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=SAFE_ASSET_ROOT)
    deck = object_model.get_part("deck")
    lip = object_model.get_part("lip")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    lip_hinge = object_model.get_articulation("front_lip_hinge")
    left_axle = object_model.get_articulation("left_wheel_axle")
    right_axle = object_model.get_articulation("right_wheel_axle")

    deck_plate = deck.get_visual("deck_plate")
    deck_left_barrel = deck.get_visual("deck_left_barrel")
    deck_right_barrel = deck.get_visual("deck_right_barrel")
    left_bracket = deck.get_visual("left_axle_bracket")
    right_bracket = deck.get_visual("right_axle_bracket")
    lip_plate = lip.get_visual("lip_plate")
    lip_barrel = lip.get_visual("lip_center_barrel")
    left_hub = left_wheel.get_visual("hub")
    right_hub = right_wheel.get_visual("hub")
    left_tire = left_wheel.get_visual("tire")
    right_tire = right_wheel.get_visual("tire")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    deck_aabb = ctx.part_world_aabb(deck)
    ctx.check(
        "deck_has_realistic_span",
        deck_aabb is not None
        and (deck_aabb[1][0] - deck_aabb[0][0]) > 2.3
        and 0.70 < (deck_aabb[1][1] - deck_aabb[0][1]) < 0.90,
        details=f"aabb={deck_aabb}",
    )

    lip_limits = lip_hinge.motion_limits
    ctx.check(
        "front_lip_hinge_axis_and_limits",
        lip_hinge.axis == (0.0, 1.0, 0.0)
        and lip_limits is not None
        and lip_limits.lower is not None
        and lip_limits.upper is not None
        and lip_limits.lower < 0.0 < lip_limits.upper <= 0.25,
        details=f"axis={lip_hinge.axis}, limits={lip_limits}",
    )
    ctx.check(
        "wheel_axles_spin_about_y",
        left_axle.axis == (0.0, 1.0, 0.0) and right_axle.axis == (0.0, 1.0, 0.0),
        details=f"left_axis={left_axle.axis}, right_axis={right_axle.axis}",
    )
    ctx.check(
        "wheel_axles_are_continuous",
        left_axle.motion_limits is not None
        and right_axle.motion_limits is not None
        and left_axle.motion_limits.lower is None
        and left_axle.motion_limits.upper is None
        and right_axle.motion_limits.lower is None
        and right_axle.motion_limits.upper is None,
        details=f"left_limits={left_axle.motion_limits}, right_limits={right_axle.motion_limits}",
    )

    ctx.expect_origin_distance(lip, deck, axes="y", max_dist=0.001)
    ctx.expect_within(lip, deck, axes="y", inner_elem=lip_plate, outer_elem=deck_plate)
    ctx.expect_gap(
        lip,
        deck,
        axis="x",
        min_gap=0.05,
        max_gap=0.12,
        positive_elem=lip_plate,
        negative_elem=deck_plate,
    )
    ctx.expect_contact(lip, deck, elem_a=lip_barrel, elem_b=deck_left_barrel)
    ctx.expect_contact(lip, deck, elem_a=lip_barrel, elem_b=deck_right_barrel)
    ctx.expect_contact(left_wheel, deck, elem_a=left_hub, elem_b=left_bracket)
    ctx.expect_contact(right_wheel, deck, elem_a=right_hub, elem_b=right_bracket)
    ctx.expect_origin_distance(left_wheel, right_wheel, axes="xz", max_dist=0.001)
    ctx.expect_gap(
        left_wheel,
        right_wheel,
        axis="y",
        min_gap=0.74,
        positive_elem=left_tire,
        negative_elem=right_tire,
    )

    if lip_limits is not None and lip_limits.lower is not None and lip_limits.upper is not None:
        with ctx.pose({lip_hinge: lip_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="front_lip_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="front_lip_lower_no_floating")
            ctx.expect_origin_distance(lip, deck, axes="y", max_dist=0.001)
            ctx.expect_contact(lip, deck, elem_a=lip_barrel, elem_b=deck_left_barrel)
            ctx.expect_contact(lip, deck, elem_a=lip_barrel, elem_b=deck_right_barrel)
        with ctx.pose({lip_hinge: lip_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="front_lip_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="front_lip_upper_no_floating")
            ctx.expect_origin_distance(lip, deck, axes="y", max_dist=0.001)
            ctx.expect_contact(lip, deck, elem_a=lip_barrel, elem_b=deck_left_barrel)
            ctx.expect_contact(lip, deck, elem_a=lip_barrel, elem_b=deck_right_barrel)

    with ctx.pose({left_axle: 1.4, right_axle: -0.9}):
        ctx.fail_if_parts_overlap_in_current_pose(name="wheel_spin_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="wheel_spin_pose_no_floating")
        ctx.expect_contact(left_wheel, deck, elem_a=left_hub, elem_b=left_bracket)
        ctx.expect_contact(right_wheel, deck, elem_a=right_hub, elem_b=right_bracket)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
