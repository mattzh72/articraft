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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cargo_folding_scooter")

    frame_red = model.material("frame_red", rgba=(0.72, 0.12, 0.11, 1.0))
    matte_black = model.material("matte_black", rgba=(0.10, 0.10, 0.11, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.30, 0.32, 0.35, 1.0))
    silver = model.material("silver", rgba=(0.76, 0.78, 0.80, 1.0))
    basket_gray = model.material("basket_gray", rgba=(0.38, 0.40, 0.43, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.82, 0.17, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=frame_red,
        name="main_deck",
    )
    frame.visual(
        Box((0.12, 0.055, 0.030)),
        origin=Origin(xyz=(0.242, 0.0, 0.001)),
        material=frame_red,
        name="front_spine",
    )
    frame.visual(
        Box((0.082, 0.012, 0.075)),
        origin=Origin(xyz=(0.236, 0.046, 0.0235)),
        material=frame_red,
        name="left_neck_cheek",
    )
    frame.visual(
        Box((0.082, 0.012, 0.075)),
        origin=Origin(xyz=(0.236, -0.046, 0.0235)),
        material=frame_red,
        name="right_neck_cheek",
    )
    frame.visual(
        Box((0.040, 0.012, 0.046)),
        origin=Origin(xyz=(0.257, 0.040, 0.053)),
        material=dark_metal,
        name="left_hinge_lug",
    )
    frame.visual(
        Box((0.040, 0.012, 0.046)),
        origin=Origin(xyz=(0.257, -0.040, 0.053)),
        material=dark_metal,
        name="right_hinge_lug",
    )
    frame.visual(
        Box((0.055, 0.070, 0.022)),
        origin=Origin(xyz=(0.315, 0.0, -0.003)),
        material=frame_red,
        name="front_gusset",
    )
    frame.visual(
        Box((0.040, 0.012, 0.196)),
        origin=Origin(xyz=(0.322, 0.034, -0.082)),
        material=dark_metal,
        name="front_fork_left",
    )
    frame.visual(
        Box((0.040, 0.012, 0.196)),
        origin=Origin(xyz=(0.322, -0.034, -0.082)),
        material=dark_metal,
        name="front_fork_right",
    )
    frame.visual(
        Box((0.028, 0.076, 0.030)),
        origin=Origin(xyz=(0.323, 0.0, 0.001)),
        material=dark_metal,
        name="front_bridge",
    )
    frame.visual(
        Box((0.060, 0.012, 0.184)),
        origin=Origin(xyz=(-0.332, 0.034, -0.082)),
        material=dark_metal,
        name="rear_dropout_left",
    )
    frame.visual(
        Box((0.060, 0.012, 0.184)),
        origin=Origin(xyz=(-0.332, -0.034, -0.082)),
        material=dark_metal,
        name="rear_dropout_right",
    )
    frame.visual(
        Box((0.060, 0.078, 0.024)),
        origin=Origin(xyz=(-0.332, 0.0, -0.002)),
        material=dark_metal,
        name="rear_bridge",
    )
    frame.visual(
        Box((0.026, 0.012, 0.032)),
        origin=Origin(xyz=(-0.320, 0.030, -0.030)),
        material=dark_metal,
        name="kick_lug_left",
    )
    frame.visual(
        Box((0.026, 0.012, 0.032)),
        origin=Origin(xyz=(-0.320, 0.090, -0.030)),
        material=dark_metal,
        name="kick_lug_right",
    )
    frame.visual(
        Box((0.032, 0.060, 0.020)),
        origin=Origin(xyz=(-0.320, 0.060, -0.004)),
        material=dark_metal,
        name="kickstand_side_bracket",
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.82, 0.17, 0.028)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    basket = model.part("basket")
    basket.visual(
        Box((0.10, 0.012, 0.018)),
        origin=Origin(xyz=(-0.010, 0.050, 0.009)),
        material=dark_metal,
        name="mount_left",
    )
    basket.visual(
        Box((0.10, 0.012, 0.018)),
        origin=Origin(xyz=(-0.010, -0.050, 0.009)),
        material=dark_metal,
        name="mount_right",
    )
    basket.visual(
        Box((0.24, 0.15, 0.008)),
        origin=Origin(xyz=(0.040, 0.0, 0.022)),
        material=basket_gray,
        name="floor",
    )
    basket.visual(
        Box((0.24, 0.006, 0.12)),
        origin=Origin(xyz=(0.040, 0.072, 0.086)),
        material=basket_gray,
        name="side_left",
    )
    basket.visual(
        Box((0.24, 0.006, 0.12)),
        origin=Origin(xyz=(0.040, -0.072, 0.086)),
        material=basket_gray,
        name="side_right",
    )
    basket.visual(
        Box((0.006, 0.15, 0.14)),
        origin=Origin(xyz=(0.157, 0.0, 0.096)),
        material=basket_gray,
        name="front_wall",
    )
    basket.visual(
        Box((0.006, 0.15, 0.075)),
        origin=Origin(xyz=(-0.077, 0.0, 0.0635)),
        material=basket_gray,
        name="rear_wall",
    )
    basket.inertial = Inertial.from_geometry(
        Box((0.24, 0.15, 0.14)),
        mass=1.2,
        origin=Origin(xyz=(0.040, 0.0, 0.070)),
    )

    front_wheel = model.part("front_wheel")
    front_wheel.visual(
        Cylinder(radius=0.10, length=0.036),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="tire",
    )
    front_wheel.visual(
        Cylinder(radius=0.073, length=0.026),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="rim",
    )
    front_wheel.visual(
        Cylinder(radius=0.032, length=0.056),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="hub",
    )
    front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.10, length=0.036),
        mass=1.4,
    )

    rear_wheel = model.part("rear_wheel")
    rear_wheel.visual(
        Cylinder(radius=0.10, length=0.036),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="tire",
    )
    rear_wheel.visual(
        Cylinder(radius=0.073, length=0.026),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="rim",
    )
    rear_wheel.visual(
        Cylinder(radius=0.032, length=0.056),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="hub",
    )
    rear_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.10, length=0.036),
        mass=1.4,
    )

    stem = model.part("stem")
    stem.visual(
        Cylinder(radius=0.024, length=0.068),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="hinge_barrel",
    )
    stem.visual(
        Box((0.042, 0.074, 0.036)),
        origin=Origin(xyz=(-0.015, 0.0, 0.0)),
        material=dark_metal,
        name="hinge_collar",
    )
    stem.visual(
        Cylinder(radius=0.022, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=silver,
        name="lower_sleeve",
    )
    stem.visual(
        Cylinder(radius=0.017, length=0.74),
        origin=Origin(xyz=(0.0, 0.0, 0.46)),
        material=silver,
        name="stem_tube",
    )
    stem.visual(
        Cylinder(radius=0.013, length=0.50),
        origin=Origin(xyz=(0.0, 0.0, 0.80), rpy=(pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="handlebar_bar",
    )
    stem.visual(
        Cylinder(radius=0.017, length=0.10),
        origin=Origin(xyz=(0.0, 0.30, 0.80), rpy=(pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="left_grip",
    )
    stem.visual(
        Cylinder(radius=0.017, length=0.10),
        origin=Origin(xyz=(0.0, -0.30, 0.80), rpy=(pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="right_grip",
    )
    stem.inertial = Inertial.from_geometry(
        Cylinder(radius=0.020, length=0.92),
        mass=2.6,
        origin=Origin(xyz=(0.0, 0.0, 0.46)),
    )

    kickstand = model.part("kickstand")
    kickstand.visual(
        Cylinder(radius=0.012, length=0.048),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="hinge_barrel",
    )
    kickstand.visual(
        Cylinder(radius=0.010, length=0.20),
        origin=Origin(xyz=(-0.100, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="prop_bar",
    )
    kickstand.visual(
        Box((0.045, 0.055, 0.012)),
        origin=Origin(xyz=(-0.205, 0.0, -0.009)),
        material=matte_black,
        name="foot_pad",
    )
    kickstand.inertial = Inertial.from_geometry(
        Cylinder(radius=0.012, length=0.22),
        mass=0.4,
        origin=Origin(xyz=(-0.100, 0.0, 0.0)),
    )

    model.articulation(
        "frame_to_basket",
        ArticulationType.FIXED,
        parent=frame,
        child=basket,
        origin=Origin(xyz=(0.385, 0.0, 0.014)),
    )
    model.articulation(
        "frame_to_front_wheel",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=front_wheel,
        origin=Origin(xyz=(0.322, 0.0, -0.118)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=15.0,
            lower=-6.0,
            upper=6.0,
        ),
    )
    model.articulation(
        "frame_to_rear_wheel",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=rear_wheel,
        origin=Origin(xyz=(-0.332, 0.0, -0.118)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=15.0,
            lower=-6.0,
            upper=6.0,
        ),
    )
    model.articulation(
        "frame_to_stem",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=stem,
        origin=Origin(xyz=(0.262, 0.0, 0.053)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=3.0,
            lower=0.0,
            upper=1.05,
        ),
    )
    model.articulation(
        "frame_to_kickstand",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=kickstand,
        origin=Origin(xyz=(-0.320, 0.060, -0.030)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=3.0,
            lower=0.0,
            upper=1.10,
        ),
    )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    basket = object_model.get_part("basket")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")
    stem = object_model.get_part("stem")
    kickstand = object_model.get_part("kickstand")

    front_axle = object_model.get_articulation("frame_to_front_wheel")
    rear_axle = object_model.get_articulation("frame_to_rear_wheel")
    stem_hinge = object_model.get_articulation("frame_to_stem")
    kickstand_hinge = object_model.get_articulation("frame_to_kickstand")

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

    ctx.expect_contact(
        basket,
        frame,
        elem_a="mount_left",
        elem_b="main_deck",
        name="left basket mount sits on the deck",
    )
    ctx.expect_contact(
        basket,
        frame,
        elem_a="mount_right",
        elem_b="main_deck",
        name="right basket mount sits on the deck",
    )
    ctx.expect_overlap(
        basket,
        frame,
        axes="x",
        elem_a="floor",
        elem_b="main_deck",
        min_overlap=0.10,
        name="basket floor covers the front deck section",
    )
    ctx.expect_within(
        basket,
        frame,
        axes="y",
        inner_elem="floor",
        outer_elem="main_deck",
        margin=0.0,
        name="basket floor stays within deck width",
    )
    ctx.expect_contact(
        front_wheel,
        frame,
        elem_a="hub",
        elem_b="front_fork_left",
        name="front hub seats against the fork",
    )
    ctx.expect_contact(
        rear_wheel,
        frame,
        elem_a="hub",
        elem_b="rear_dropout_left",
        name="rear hub seats against the dropout",
    )
    ctx.expect_contact(
        stem,
        frame,
        elem_a="hinge_barrel",
        elem_b="left_hinge_lug",
        name="stem hinge barrel seats between the neck lugs",
    )
    ctx.expect_contact(
        kickstand,
        frame,
        elem_a="hinge_barrel",
        elem_b="kick_lug_left",
        name="kickstand hinge sits under the rear bridge",
    )
    ctx.expect_gap(
        frame,
        front_wheel,
        axis="z",
        positive_elem="main_deck",
        negative_elem="tire",
        min_gap=0.002,
        max_gap=0.020,
        name="front tire clears the deck",
    )
    ctx.expect_gap(
        frame,
        rear_wheel,
        axis="z",
        positive_elem="main_deck",
        negative_elem="tire",
        min_gap=0.002,
        max_gap=0.020,
        name="rear tire clears the deck",
    )

    ctx.check(
        "front wheel axle is a transverse revolute joint",
        front_axle.articulation_type == ArticulationType.REVOLUTE
        and abs(front_axle.axis[0]) < 1e-9
        and abs(front_axle.axis[1] - 1.0) < 1e-9
        and abs(front_axle.axis[2]) < 1e-9,
        details=f"type={front_axle.articulation_type}, axis={front_axle.axis}",
    )
    ctx.check(
        "rear wheel axle is a transverse revolute joint",
        rear_axle.articulation_type == ArticulationType.REVOLUTE
        and abs(rear_axle.axis[0]) < 1e-9
        and abs(rear_axle.axis[1] - 1.0) < 1e-9
        and abs(rear_axle.axis[2]) < 1e-9,
        details=f"type={rear_axle.articulation_type}, axis={rear_axle.axis}",
    )
    ctx.check(
        "stem hinge folds backward from the deck neck",
        stem_hinge.articulation_type == ArticulationType.REVOLUTE
        and abs(stem_hinge.axis[0]) < 1e-9
        and abs(stem_hinge.axis[1] + 1.0) < 1e-9
        and abs(stem_hinge.axis[2]) < 1e-9
        and stem_hinge.motion_limits is not None
        and stem_hinge.motion_limits.lower == 0.0
        and stem_hinge.motion_limits.upper is not None
        and stem_hinge.motion_limits.upper >= 0.9,
        details=f"type={stem_hinge.articulation_type}, axis={stem_hinge.axis}, limits={stem_hinge.motion_limits}",
    )
    ctx.check(
        "kickstand hinge swings downward from the rear underside",
        kickstand_hinge.articulation_type == ArticulationType.REVOLUTE
        and abs(kickstand_hinge.axis[0]) < 1e-9
        and abs(kickstand_hinge.axis[1] + 1.0) < 1e-9
        and abs(kickstand_hinge.axis[2]) < 1e-9
        and kickstand_hinge.motion_limits is not None
        and kickstand_hinge.motion_limits.lower == 0.0
        and kickstand_hinge.motion_limits.upper is not None
        and kickstand_hinge.motion_limits.upper >= 1.0,
        details=f"type={kickstand_hinge.articulation_type}, axis={kickstand_hinge.axis}, limits={kickstand_hinge.motion_limits}",
    )

    basket_origin = ctx.part_world_position(basket)
    ctx.check(
        "basket sits on the front half of the scooter",
        basket_origin is not None and basket_origin[0] > 0.05,
        details=f"basket_origin={basket_origin}",
    )

    stem_upper = stem_hinge.motion_limits.upper if stem_hinge.motion_limits is not None else None
    rest_bar = _aabb_center(ctx.part_element_world_aabb(stem, elem="handlebar_bar"))
    folded_bar = None
    rear_wall_top = None
    if stem_upper is not None:
        with ctx.pose({stem_hinge: stem_upper}):
            folded_bar = _aabb_center(ctx.part_element_world_aabb(stem, elem="handlebar_bar"))
            rear_wall_aabb = ctx.part_element_world_aabb(basket, elem="rear_wall")
            rear_wall_top = rear_wall_aabb[1][2] if rear_wall_aabb is not None else None
    ctx.check(
        "stem handlebar moves down and back when folded",
        rest_bar is not None
        and folded_bar is not None
        and folded_bar[0] < rest_bar[0] - 0.25
        and folded_bar[2] < rest_bar[2] - 0.20,
        details=f"rest_bar={rest_bar}, folded_bar={folded_bar}",
    )
    ctx.check(
        "folded handlebar still clears the low basket rear wall",
        folded_bar is not None and rear_wall_top is not None and folded_bar[2] > rear_wall_top + 0.05,
        details=f"folded_bar={folded_bar}, rear_wall_top={rear_wall_top}",
    )

    kick_upper = kickstand_hinge.motion_limits.upper if kickstand_hinge.motion_limits is not None else None
    rest_foot = _aabb_center(ctx.part_element_world_aabb(kickstand, elem="foot_pad"))
    deployed_foot = None
    if kick_upper is not None:
        with ctx.pose({kickstand_hinge: kick_upper}):
            deployed_foot = _aabb_center(ctx.part_element_world_aabb(kickstand, elem="foot_pad"))
    ctx.check(
        "kickstand drops below the deck when deployed",
        rest_foot is not None
        and deployed_foot is not None
        and deployed_foot[2] < rest_foot[2] - 0.12
        and deployed_foot[0] < -0.36,
        details=f"rest_foot={rest_foot}, deployed_foot={deployed_foot}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
