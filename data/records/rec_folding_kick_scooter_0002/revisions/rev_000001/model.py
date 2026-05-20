from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
from pathlib import Path

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

ASSETS = AssetContext.from_script(__file__)
HERE = Path(__file__).resolve().parent


def _add_wheel_visuals(part, *, prefix: str, tire_radius: float, tire_width: float, hub_width: float, rim_radius: float, rubber, metal) -> None:
    spin_origin = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))
    part.visual(
        Cylinder(radius=tire_radius, length=tire_width),
        origin=spin_origin,
        material=rubber,
        name=f"{prefix}_tire",
    )
    part.visual(
        Cylinder(radius=rim_radius, length=0.018),
        origin=spin_origin,
        material=metal,
        name=f"{prefix}_rim",
    )
    part.visual(
        Cylinder(radius=0.022, length=hub_width),
        origin=spin_origin,
        material=metal,
        name=f"{prefix}_hub",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_kick_scooter", assets=ASSETS)

    wheel_radius = 0.10
    tire_width = 0.024
    hub_width = 0.044

    frame_gray = model.material("frame_gray", rgba=(0.67, 0.69, 0.72, 1.0))
    deck_black = model.material("deck_black", rgba=(0.10, 0.10, 0.11, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.07, 0.07, 0.07, 1.0))
    accent_red = model.material("accent_red", rgba=(0.82, 0.18, 0.16, 1.0))

    deck = model.part("deck")
    deck.visual(Box((0.44, 0.10, 0.032)), origin=Origin(xyz=(0.0, 0.0, 0.040)), material=frame_gray, name="deck_body_core")
    deck.visual(Box((0.48, 0.13, 0.006)), origin=Origin(xyz=(0.0, 0.0, 0.059)), material=deck_black, name="deck_body_top")
    deck.visual(Box((0.48, 0.010, 0.038)), origin=Origin(xyz=(0.0, 0.060, 0.045)), material=frame_gray, name="deck_side_left")
    deck.visual(Box((0.48, 0.010, 0.038)), origin=Origin(xyz=(0.0, -0.060, 0.045)), material=frame_gray, name="deck_side_right")
    deck.visual(Box((0.070, 0.090, 0.028)), origin=Origin(xyz=(0.225, 0.0, 0.056)), material=accent_red, name="deck_nose")
    deck.visual(Box((0.020, 0.040, 0.056)), origin=Origin(xyz=(0.000, 0.0, 0.090)), material=frame_gray, name="fold_rest_tower")
    deck.visual(Box((0.050, 0.052, 0.010)), origin=Origin(xyz=(0.000, 0.0, 0.123)), material=accent_red, name="fold_rest")
    deck.visual(Box((0.280, 0.018, 0.028)), origin=Origin(xyz=(-0.240, 0.026, 0.058)), material=frame_gray, name="rear_stay_left")
    deck.visual(Box((0.280, 0.018, 0.028)), origin=Origin(xyz=(-0.240, -0.026, 0.058)), material=frame_gray, name="rear_stay_right")
    deck.visual(Box((0.020, 0.010, 0.052)), origin=Origin(xyz=(-0.369, 0.026, 0.078)), material=frame_gray, name="rear_dropout_left")
    deck.visual(Box((0.020, 0.010, 0.052)), origin=Origin(xyz=(-0.369, -0.026, 0.078)), material=frame_gray, name="rear_dropout_right")
    deck.inertial = Inertial.from_geometry(Box((0.62, 0.13, 0.16)), mass=4.3, origin=Origin(xyz=(-0.05, 0.0, 0.060)))

    front_fork = model.part("front_fork")
    front_fork.visual(Box((0.028, 0.050, 0.010)), origin=Origin(xyz=(0.0, 0.0, 0.005)), material=frame_gray, name="steer_base_plate")
    front_fork.visual(Cylinder(radius=0.016, length=0.150), origin=Origin(xyz=(0.0, 0.0, 0.075)), material=frame_gray, name="steering_post")
    front_fork.visual(Box((0.020, 0.008, 0.024)), origin=Origin(xyz=(0.010, 0.020, 0.075)), material=frame_gray, name="hinge_ear_left")
    front_fork.visual(Box((0.020, 0.008, 0.024)), origin=Origin(xyz=(0.010, -0.020, 0.075)), material=frame_gray, name="hinge_ear_right")
    front_fork.visual(Box((0.180, 0.070, 0.020)), origin=Origin(xyz=(0.090, 0.0, 0.112)), material=frame_gray, name="fork_crown")
    front_fork.visual(Box((0.016, 0.008, 0.180)), origin=Origin(xyz=(0.165, 0.026, 0.005)), material=frame_gray, name="fork_left")
    front_fork.visual(Box((0.016, 0.008, 0.180)), origin=Origin(xyz=(0.165, -0.026, 0.005)), material=frame_gray, name="fork_right")
    front_fork.inertial = Inertial.from_geometry(Box((0.22, 0.06, 0.26)), mass=1.4, origin=Origin(xyz=(0.090, 0.0, 0.070)))

    stem = model.part("stem")
    stem.visual(Cylinder(radius=0.008, length=0.034), origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)), material=frame_gray, name="hinge_knuckle")
    stem.visual(Box((0.032, 0.028, 0.018)), origin=Origin(), material=accent_red, name="stem_base")
    stem.visual(Box((0.024, 0.024, 0.060)), origin=Origin(xyz=(-0.015, 0.0, 0.030)), material=frame_gray, name="stem_neck")
    stem.visual(Cylinder(radius=0.017, length=0.580), origin=Origin(xyz=(-0.015, 0.0, 0.405)), material=frame_gray, name="stem_tube")
    stem.visual(Box((0.050, 0.036, 0.030)), origin=Origin(xyz=(-0.015, 0.0, 0.705)), material=frame_gray, name="center_clamp")
    stem.visual(Cylinder(radius=0.014, length=0.540), origin=Origin(xyz=(-0.015, 0.0, 0.724), rpy=(math.pi / 2.0, 0.0, 0.0)), material=frame_gray, name="crossbar")
    stem.visual(Cylinder(radius=0.017, length=0.100), origin=Origin(xyz=(-0.015, 0.320, 0.724), rpy=(math.pi / 2.0, 0.0, 0.0)), material=rubber_black, name="left_grip")
    stem.visual(Cylinder(radius=0.017, length=0.100), origin=Origin(xyz=(-0.015, -0.320, 0.724), rpy=(math.pi / 2.0, 0.0, 0.0)), material=rubber_black, name="right_grip")
    stem.visual(Box((0.028, 0.024, 0.024)), origin=Origin(xyz=(-0.020, 0.0, 0.190)), material=frame_gray, name="rest_bracket")
    stem.visual(Box((0.014, 0.050, 0.028)), origin=Origin(xyz=(-0.037, 0.0, 0.190)), material=accent_red, name="stem_rest_pad")
    stem.inertial = Inertial.from_geometry(Box((0.12, 0.66, 0.74)), mass=3.2, origin=Origin(xyz=(-0.015, 0.0, 0.390)))

    front_wheel = model.part("front_wheel")
    _add_wheel_visuals(front_wheel, prefix="front", tire_radius=wheel_radius, tire_width=tire_width, hub_width=hub_width, rim_radius=0.078, rubber=rubber_black, metal=frame_gray)
    front_wheel.inertial = Inertial.from_geometry(Cylinder(radius=wheel_radius, length=tire_width), mass=1.0, origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)))

    rear_wheel = model.part("rear_wheel")
    _add_wheel_visuals(rear_wheel, prefix="rear", tire_radius=wheel_radius, tire_width=tire_width, hub_width=hub_width, rim_radius=0.078, rubber=rubber_black, metal=frame_gray)
    rear_wheel.inertial = Inertial.from_geometry(Cylinder(radius=wheel_radius, length=tire_width), mass=1.0, origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)))

    model.articulation("deck_to_front_fork", ArticulationType.REVOLUTE, parent=deck, child=front_fork, origin=Origin(xyz=(0.225, 0.0, 0.070)), axis=(0.0, 0.0, 1.0), motion_limits=MotionLimits(effort=16.0, velocity=2.5, lower=-0.65, upper=0.65))
    model.articulation("front_fork_to_stem", ArticulationType.REVOLUTE, parent=front_fork, child=stem, origin=Origin(xyz=(0.010, 0.0, 0.075)), axis=(0.0, 1.0, 0.0), motion_limits=MotionLimits(effort=18.0, velocity=2.2, lower=-1.45, upper=0.0))
    model.articulation("front_fork_to_front_wheel", ArticulationType.CONTINUOUS, parent=front_fork, child=front_wheel, origin=Origin(xyz=(0.165, 0.0, 0.0)), axis=(0.0, 1.0, 0.0), motion_limits=MotionLimits(effort=14.0, velocity=28.0))
    model.articulation("deck_to_rear_wheel", ArticulationType.CONTINUOUS, parent=deck, child=rear_wheel, origin=Origin(xyz=(-0.391, 0.0, wheel_radius)), axis=(0.0, 1.0, 0.0), motion_limits=MotionLimits(effort=14.0, velocity=28.0))

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)

    deck = object_model.get_part("deck")
    front_fork = object_model.get_part("front_fork")
    stem = object_model.get_part("stem")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")

    steer_joint = object_model.get_articulation("deck_to_front_fork")
    fold_joint = object_model.get_articulation("front_fork_to_stem")
    front_spin = object_model.get_articulation("front_fork_to_front_wheel")
    rear_spin = object_model.get_articulation("deck_to_rear_wheel")

    deck_nose = deck.get_visual("deck_nose")
    deck_body_top = deck.get_visual("deck_body_top")
    fold_rest = deck.get_visual("fold_rest")
    steer_base_plate = front_fork.get_visual("steer_base_plate")
    hinge_ear_left = front_fork.get_visual("hinge_ear_left")
    hinge_ear_right = front_fork.get_visual("hinge_ear_right")
    hinge_knuckle = stem.get_visual("hinge_knuckle")
    fork_left = front_fork.get_visual("fork_left")
    fork_right = front_fork.get_visual("fork_right")
    stem_rest_pad = stem.get_visual("stem_rest_pad")
    center_clamp = stem.get_visual("center_clamp")
    front_tire = front_wheel.get_visual("front_tire")
    front_hub = front_wheel.get_visual("front_hub")
    rear_tire = rear_wheel.get_visual("rear_tire")
    rear_hub = rear_wheel.get_visual("rear_hub")
    rear_dropout_left = deck.get_visual("rear_dropout_left")
    rear_dropout_right = deck.get_visual("rear_dropout_right")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.allow_overlap(front_fork, stem, reason="steering post and folding stem nest inside the scooter head assembly")
    ctx.allow_overlap(front_fork, front_wheel, reason="front wheel hub sits captured between simplified fork legs and steering geometry")
    ctx.allow_overlap(stem, rear_wheel, reason="folded stow pose uses a simplified straight stem tube that passes over the rear wheel envelope")
    ctx.fail_if_isolated_parts(contact_tol=0.005)
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(front_fork, deck, elem_a=steer_base_plate, elem_b=deck_nose)
    ctx.expect_contact(stem, front_fork, elem_a=hinge_knuckle, elem_b=hinge_ear_left)
    ctx.expect_contact(stem, front_fork, elem_a=hinge_knuckle, elem_b=hinge_ear_right)
    ctx.expect_contact(front_wheel, front_fork, elem_a=front_hub, elem_b=fork_left, contact_tol=0.005)
    ctx.expect_contact(front_wheel, front_fork, elem_a=front_hub, elem_b=fork_right, contact_tol=0.005)
    ctx.expect_contact(rear_wheel, deck, elem_a=rear_hub, elem_b=rear_dropout_left)
    ctx.expect_contact(rear_wheel, deck, elem_a=rear_hub, elem_b=rear_dropout_right)
    ctx.expect_origin_distance(front_wheel, rear_wheel, axes="y", max_dist=0.002)
    ctx.expect_gap(front_wheel, deck, axis="x", positive_elem=front_tire, negative_elem=deck_nose, min_gap=0.005, max_gap=0.040)
    ctx.expect_gap(deck, rear_wheel, axis="x", positive_elem=deck_body_top, negative_elem=rear_tire, min_gap=0.0, max_gap=0.06)
    ctx.expect_gap(stem, deck, axis="z", positive_elem=center_clamp, negative_elem=deck_body_top, min_gap=0.77)

    with ctx.pose({steer_joint: steer_joint.motion_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="steer_left_no_overlap")
        ctx.fail_if_isolated_parts(contact_tol=0.005, name="steer_left_no_floating")

    with ctx.pose({steer_joint: steer_joint.motion_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="steer_right_no_overlap")
        ctx.fail_if_isolated_parts(contact_tol=0.005, name="steer_right_no_floating")

    with ctx.pose({fold_joint: fold_joint.motion_limits.lower}):
        ctx.expect_contact(stem, deck, elem_a=stem_rest_pad, elem_b=fold_rest, contact_tol=0.003)
        ctx.expect_gap(stem, deck, axis="z", positive_elem=center_clamp, negative_elem=deck_body_top, max_gap=0.13, max_penetration=0.0)
        ctx.fail_if_parts_overlap_in_current_pose(name="folded_no_overlap")
        ctx.fail_if_isolated_parts(contact_tol=0.005, name="folded_no_floating")

    with ctx.pose({front_spin: math.pi / 2.0, rear_spin: math.pi}):
        ctx.expect_contact(front_wheel, front_fork, elem_a=front_hub, elem_b=fork_left, contact_tol=0.005, name="front_wheel_spin_mount_left")
        ctx.expect_contact(front_wheel, front_fork, elem_a=front_hub, elem_b=fork_right, contact_tol=0.005, name="front_wheel_spin_mount_right")
        ctx.expect_contact(rear_wheel, deck, elem_a=rear_hub, elem_b=rear_dropout_left, name="rear_wheel_spin_mount_left")
        ctx.expect_contact(rear_wheel, deck, elem_a=rear_hub, elem_b=rear_dropout_right, name="rear_wheel_spin_mount_right")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
