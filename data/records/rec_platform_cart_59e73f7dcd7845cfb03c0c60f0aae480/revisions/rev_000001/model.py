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


def _add_caster(
    model: ArticulatedObject,
    deck,
    *,
    name: str,
    position: tuple[float, float, float],
    steel,
    wheel_rubber,
    hub_steel,
) -> tuple[object, object, object, object]:
    caster = model.part(f"{name}_caster")
    caster.visual(
        Box((0.064, 0.056, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=steel,
        name="swivel_plate",
    )
    caster.visual(
        Cylinder(radius=0.011, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, -0.019)),
        material=hub_steel,
        name="swivel_stem",
    )
    caster.visual(
        Box((0.040, 0.028, 0.014)),
        origin=Origin(xyz=(0.0, -0.010, -0.039)),
        material=steel,
        name="fork_crown",
    )
    caster.visual(
        Box((0.005, 0.024, 0.062)),
        origin=Origin(xyz=(-0.015, -0.018, -0.077)),
        material=steel,
        name="left_fork_leg",
    )
    caster.visual(
        Box((0.005, 0.024, 0.062)),
        origin=Origin(xyz=(0.015, -0.018, -0.077)),
        material=steel,
        name="right_fork_leg",
    )
    caster.visual(
        Cylinder(radius=0.009, length=0.003),
        origin=Origin(xyz=(-0.011, -0.018, -0.100), rpy=(0.0, pi / 2.0, 0.0)),
        material=hub_steel,
        name="left_axle_stub",
    )
    caster.visual(
        Cylinder(radius=0.009, length=0.003),
        origin=Origin(xyz=(0.011, -0.018, -0.100), rpy=(0.0, pi / 2.0, 0.0)),
        material=hub_steel,
        name="right_axle_stub",
    )
    caster.inertial = Inertial.from_geometry(
        Box((0.064, 0.056, 0.114)),
        mass=0.45,
        origin=Origin(xyz=(0.0, -0.010, -0.060)),
    )

    wheel = model.part(f"{name}_wheel")
    wheel.visual(
        Cylinder(radius=0.050, length=0.022),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=wheel_rubber,
        name="tire",
    )
    wheel.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=hub_steel,
        name="hub",
    )
    wheel.visual(
        Cylinder(radius=0.015, length=0.006),
        origin=Origin(xyz=(-0.008, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="left_cap",
    )
    wheel.visual(
        Cylinder(radius=0.015, length=0.006),
        origin=Origin(xyz=(0.008, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="right_cap",
    )
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.050, length=0.022),
        mass=0.36,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    swivel = model.articulation(
        f"{name}_swivel",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=caster,
        origin=Origin(xyz=position),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=8.0),
    )
    spin = model.articulation(
        f"{name}_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=caster,
        child=wheel,
        origin=Origin(xyz=(0.0, -0.018, -0.100)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=35.0),
    )
    return caster, wheel, swivel, spin


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="shop_cart")

    cart_steel = model.material("cart_steel", rgba=(0.44, 0.47, 0.50, 1.0))
    darker_steel = model.material("darker_steel", rgba=(0.26, 0.28, 0.30, 1.0))
    hub_steel = model.material("hub_steel", rgba=(0.63, 0.66, 0.69, 1.0))
    deck_mat = model.material("deck_mat", rgba=(0.16, 0.17, 0.18, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.06, 0.06, 0.06, 1.0))
    grip_rubber = model.material("grip_rubber", rgba=(0.10, 0.10, 0.10, 1.0))

    deck = model.part("deck")
    deck.visual(
        Box((0.44, 0.72, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=cart_steel,
        name="deck_body",
    )
    deck.visual(
        Box((0.36, 0.64, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=deck_mat,
        name="deck_surface",
    )
    deck.visual(
        Box((0.38, 0.018, 0.022)),
        origin=Origin(xyz=(0.0, 0.351, 0.025)),
        material=darker_steel,
        name="front_bumper",
    )
    deck.visual(
        Box((0.018, 0.66, 0.022)),
        origin=Origin(xyz=(-0.211, 0.0, 0.025)),
        material=darker_steel,
        name="left_rub_rail",
    )
    deck.visual(
        Box((0.018, 0.66, 0.022)),
        origin=Origin(xyz=(0.211, 0.0, 0.025)),
        material=darker_steel,
        name="right_rub_rail",
    )
    deck.visual(
        Box((0.40, 0.028, 0.010)),
        origin=Origin(xyz=(0.0, -0.346, 0.019)),
        material=darker_steel,
        name="rear_hinge_beam",
    )
    deck.visual(
        Box((0.018, 0.022, 0.032)),
        origin=Origin(xyz=(-0.193, -0.343, 0.022)),
        material=darker_steel,
        name="left_handle_mount",
    )
    deck.visual(
        Box((0.018, 0.022, 0.032)),
        origin=Origin(xyz=(0.193, -0.343, 0.022)),
        material=darker_steel,
        name="right_handle_mount",
    )
    deck.visual(
        Box((0.018, 0.020, 0.050)),
        origin=Origin(xyz=(-0.160, -0.336, 0.006)),
        material=darker_steel,
        name="left_brake_tab",
    )
    deck.visual(
        Box((0.018, 0.020, 0.050)),
        origin=Origin(xyz=(0.160, -0.336, 0.006)),
        material=darker_steel,
        name="right_brake_tab",
    )
    deck.visual(
        Box((0.090, 0.090, 0.012)),
        origin=Origin(xyz=(-0.240, 0.265, -0.006)),
        material=darker_steel,
        name="front_left_caster_pad",
    )
    deck.visual(
        Box((0.090, 0.090, 0.012)),
        origin=Origin(xyz=(0.240, 0.265, -0.006)),
        material=darker_steel,
        name="front_right_caster_pad",
    )
    deck.visual(
        Box((0.090, 0.090, 0.012)),
        origin=Origin(xyz=(-0.240, -0.265, -0.006)),
        material=darker_steel,
        name="rear_left_caster_pad",
    )
    deck.visual(
        Box((0.090, 0.090, 0.012)),
        origin=Origin(xyz=(0.240, -0.265, -0.006)),
        material=darker_steel,
        name="rear_right_caster_pad",
    )
    deck.visual(
        Box((0.060, 0.500, 0.028)),
        origin=Origin(xyz=(-0.135, 0.0, -0.014)),
        material=darker_steel,
        name="left_under_rail",
    )
    deck.visual(
        Box((0.060, 0.500, 0.028)),
        origin=Origin(xyz=(0.135, 0.0, -0.014)),
        material=darker_steel,
        name="right_under_rail",
    )
    deck.visual(
        Box((0.280, 0.050, 0.022)),
        origin=Origin(xyz=(0.0, -0.230, -0.011)),
        material=darker_steel,
        name="rear_cross_member",
    )
    deck.inertial = Inertial.from_geometry(
        Box((0.44, 0.72, 0.060)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.012, length=0.055),
        origin=Origin(xyz=(-0.155, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=cart_steel,
        name="left_hinge_barrel",
    )
    handle.visual(
        Cylinder(radius=0.012, length=0.055),
        origin=Origin(xyz=(0.155, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=cart_steel,
        name="right_hinge_barrel",
    )
    handle.visual(
        Cylinder(radius=0.012, length=0.560),
        origin=Origin(xyz=(-0.155, 0.280, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=cart_steel,
        name="left_upright",
    )
    handle.visual(
        Cylinder(radius=0.012, length=0.560),
        origin=Origin(xyz=(0.155, 0.280, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=cart_steel,
        name="right_upright",
    )
    handle.visual(
        Cylinder(radius=0.012, length=0.310),
        origin=Origin(xyz=(0.0, 0.560, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=cart_steel,
        name="handle_cross_tube",
    )
    handle.visual(
        Cylinder(radius=0.013, length=0.220),
        origin=Origin(xyz=(0.0, 0.560, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip_rubber,
        name="handle_grip",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.35, 0.58, 0.04)),
        mass=2.0,
        origin=Origin(xyz=(0.0, 0.280, 0.0)),
    )

    brake_bar = model.part("brake_bar")
    brake_bar.visual(
        Cylinder(radius=0.008, length=0.300),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=cart_steel,
        name="brake_pivot_shaft",
    )
    brake_bar.visual(
        Cylinder(radius=0.008, length=0.064),
        origin=Origin(xyz=(-0.145, -0.0225, -0.0225), rpy=(3.0 * pi / 4.0, 0.0, 0.0)),
        material=cart_steel,
        name="left_brake_arm",
    )
    brake_bar.visual(
        Cylinder(radius=0.008, length=0.064),
        origin=Origin(xyz=(0.145, -0.0225, -0.0225), rpy=(3.0 * pi / 4.0, 0.0, 0.0)),
        material=cart_steel,
        name="right_brake_arm",
    )
    brake_bar.visual(
        Cylinder(radius=0.010, length=0.320),
        origin=Origin(xyz=(0.0, -0.045, -0.045), rpy=(0.0, pi / 2.0, 0.0)),
        material=darker_steel,
        name="brake_crossbar",
    )
    brake_bar.visual(
        Box((0.120, 0.034, 0.006)),
        origin=Origin(xyz=(0.0, -0.052, -0.047)),
        material=deck_mat,
        name="brake_tread",
    )
    brake_bar.inertial = Inertial.from_geometry(
        Box((0.32, 0.08, 0.08)),
        mass=0.8,
        origin=Origin(xyz=(0.0, -0.030, -0.030)),
    )

    _add_caster(
        model,
        deck,
        name="front_left",
        position=(-0.240, 0.265, -0.012),
        steel=cart_steel,
        wheel_rubber=wheel_rubber,
        hub_steel=hub_steel,
    )
    _add_caster(
        model,
        deck,
        name="front_right",
        position=(0.240, 0.265, -0.012),
        steel=cart_steel,
        wheel_rubber=wheel_rubber,
        hub_steel=hub_steel,
    )
    _add_caster(
        model,
        deck,
        name="rear_left",
        position=(-0.240, -0.265, -0.012),
        steel=cart_steel,
        wheel_rubber=wheel_rubber,
        hub_steel=hub_steel,
    )
    _add_caster(
        model,
        deck,
        name="rear_right",
        position=(0.240, -0.265, -0.012),
        steel=cart_steel,
        wheel_rubber=wheel_rubber,
        hub_steel=hub_steel,
    )

    model.articulation(
        "deck_to_handle",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=handle,
        origin=Origin(xyz=(0.0, -0.346, 0.045)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.2, lower=0.0, upper=1.55),
    )
    model.articulation(
        "deck_to_brake_bar",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=brake_bar,
        origin=Origin(xyz=(0.0, -0.338, -0.010)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=0.0, upper=0.70),
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

    deck = object_model.get_part("deck")
    handle = object_model.get_part("handle")
    brake_bar = object_model.get_part("brake_bar")

    handle_hinge = object_model.get_articulation("deck_to_handle")
    brake_joint = object_model.get_articulation("deck_to_brake_bar")

    swivel_names = (
        "front_left_swivel",
        "front_right_swivel",
        "rear_left_swivel",
        "rear_right_swivel",
    )
    spin_names = (
        "front_left_wheel_spin",
        "front_right_wheel_spin",
        "rear_left_wheel_spin",
        "rear_right_wheel_spin",
    )
    swivel_joints = [object_model.get_articulation(name) for name in swivel_names]
    spin_joints = [object_model.get_articulation(name) for name in spin_names]

    ctx.check(
        "caster swivel joints are vertical",
        all(joint.axis == (0.0, 0.0, 1.0) for joint in swivel_joints),
        details=str([joint.axis for joint in swivel_joints]),
    )
    ctx.check(
        "caster wheel joints spin on axle axis",
        all(joint.axis == (1.0, 0.0, 0.0) for joint in spin_joints),
        details=str([joint.axis for joint in spin_joints]),
    )
    ctx.check(
        "handle hinge opens upward from the deck edge",
        handle_hinge.axis == (1.0, 0.0, 0.0)
        and handle_hinge.motion_limits is not None
        and handle_hinge.motion_limits.lower == 0.0
        and handle_hinge.motion_limits.upper is not None
        and handle_hinge.motion_limits.upper > 1.4,
        details=f"axis={handle_hinge.axis}, limits={handle_hinge.motion_limits}",
    )
    ctx.check(
        "brake bar pivots on rear x-axis",
        brake_joint.axis == (1.0, 0.0, 0.0)
        and brake_joint.motion_limits is not None
        and brake_joint.motion_limits.upper is not None
        and brake_joint.motion_limits.upper >= 0.65,
        details=f"axis={brake_joint.axis}, limits={brake_joint.motion_limits}",
    )

    with ctx.pose({handle_hinge: 0.0}):
        ctx.expect_gap(
            handle,
            deck,
            axis="z",
            max_gap=0.005,
            max_penetration=0.0005,
            positive_elem="handle_grip",
            negative_elem="deck_surface",
            name="folded handle sits flush over deck",
        )
        ctx.expect_overlap(
            handle,
            deck,
            axes="xy",
            min_overlap=0.020,
            elem_a="handle_grip",
            elem_b="deck_surface",
            name="folded handle lies over the platform footprint",
        )
        closed_grip_aabb = ctx.part_element_world_aabb(handle, elem="handle_grip")

    with ctx.pose({handle_hinge: 1.45}):
        open_grip_aabb = ctx.part_element_world_aabb(handle, elem="handle_grip")

    handle_lifts = (
        closed_grip_aabb is not None
        and open_grip_aabb is not None
        and open_grip_aabb[1][2] > closed_grip_aabb[1][2] + 0.35
    )
    ctx.check(
        "handle rises into upright position",
        handle_lifts,
        details=f"closed={closed_grip_aabb}, open={open_grip_aabb}",
    )

    rear_left_wheel = object_model.get_part("rear_left_wheel")
    with ctx.pose({brake_joint: 0.0}):
        rest_brake_aabb = ctx.part_element_world_aabb(brake_bar, elem="brake_crossbar")
        rest_wheel_aabb = ctx.part_element_world_aabb(rear_left_wheel, elem="tire")

    with ctx.pose({brake_joint: 0.65}):
        engaged_brake_aabb = ctx.part_element_world_aabb(brake_bar, elem="brake_crossbar")

    brake_is_parked_high = False
    if rest_brake_aabb is not None and rest_wheel_aabb is not None:
        rest_brake_center_z = 0.5 * (rest_brake_aabb[0][2] + rest_brake_aabb[1][2])
        rest_wheel_center_z = 0.5 * (rest_wheel_aabb[0][2] + rest_wheel_aabb[1][2])
        brake_is_parked_high = (
            rest_brake_center_z > rest_wheel_center_z + 0.04
            and rest_brake_aabb[1][1] < rest_wheel_aabb[0][1] + 0.01
        )
    ctx.check(
        "brake bar parks high behind the rear caster pair",
        brake_is_parked_high,
        details=f"brake={rest_brake_aabb}, wheel={rest_wheel_aabb}",
    )

    brake_descends = (
        rest_brake_aabb is not None
        and engaged_brake_aabb is not None
        and engaged_brake_aabb[0][2] < rest_brake_aabb[0][2] - 0.015
    )
    ctx.check(
        "brake bar swings downward when engaged",
        brake_descends,
        details=f"rest={rest_brake_aabb}, engaged={engaged_brake_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
