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


def _add_wheel_visuals(part, *, radius: float, width: float, tire_mat, rim_mat, hub_mat) -> None:
    spin_origin = Origin(rpy=(-pi / 2.0, 0.0, 0.0))
    part.visual(
        Cylinder(radius=radius, length=width),
        origin=spin_origin,
        material=tire_mat,
        name="tire",
    )
    part.visual(
        Cylinder(radius=radius * 0.64, length=width * 0.82),
        origin=spin_origin,
        material=rim_mat,
        name="rim",
    )
    part.visual(
        Cylinder(radius=radius * 0.24, length=width * 0.92),
        origin=spin_origin,
        material=hub_mat,
        name="hub",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="toy_roadster")

    body_red = model.material("body_red", rgba=(0.78, 0.13, 0.12, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.05, 0.05, 0.06, 1.0))
    black_trim = model.material("black_trim", rgba=(0.12, 0.12, 0.13, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.73, 0.76, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.34, 0.35, 0.37, 1.0))
    seat_vinyl = model.material("seat_vinyl", rgba=(0.18, 0.16, 0.15, 1.0))
    windshield_glass = model.material("windshield_glass", rgba=(0.74, 0.86, 0.92, 0.38))

    body = model.part("body")
    body.visual(
        Box((0.276, 0.106, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=black_trim,
        name="undertray",
    )
    body.visual(
        Box((0.120, 0.012, 0.028)),
        origin=Origin(xyz=(-0.005, 0.040, 0.031)),
        material=body_red,
        name="left_sill",
    )
    body.visual(
        Box((0.120, 0.012, 0.028)),
        origin=Origin(xyz=(-0.005, -0.040, 0.031)),
        material=body_red,
        name="right_sill",
    )
    body.visual(
        Box((0.028, 0.098, 0.022)),
        origin=Origin(xyz=(0.026, 0.0, 0.042)),
        material=body_red,
        name="cowl",
    )
    body.visual(
        Box((0.100, 0.018, 0.020)),
        origin=Origin(xyz=(0.084, 0.045, 0.040)),
        material=body_red,
        name="front_left_fender",
    )
    body.visual(
        Box((0.100, 0.018, 0.020)),
        origin=Origin(xyz=(0.084, -0.045, 0.040)),
        material=body_red,
        name="front_right_fender",
    )
    body.visual(
        Box((0.018, 0.096, 0.022)),
        origin=Origin(xyz=(0.138, 0.0, 0.026)),
        material=body_red,
        name="nose_panel",
    )
    body.visual(
        Box((0.014, 0.090, 0.010)),
        origin=Origin(xyz=(0.086, 0.0, 0.026)),
        material=dark_steel,
        name="front_axle_beam",
    )
    body.visual(
        Box((0.040, 0.024, 0.020)),
        origin=Origin(xyz=(0.072, 0.0, 0.024)),
        material=dark_steel,
        name="front_axle_support",
    )
    body.visual(
        Box((0.012, 0.018, 0.012)),
        origin=Origin(xyz=(0.086, 0.049, 0.026)),
        material=dark_steel,
        name="front_left_hub_support",
    )
    body.visual(
        Box((0.012, 0.018, 0.012)),
        origin=Origin(xyz=(0.086, -0.049, 0.026)),
        material=dark_steel,
        name="front_right_hub_support",
    )
    body.visual(
        Box((0.008, 0.052, 0.016)),
        origin=Origin(xyz=(0.022, 0.0, 0.045)),
        material=dark_steel,
        name="hood_hinge_support",
    )
    body.visual(
        Box((0.014, 0.086, 0.036)),
        origin=Origin(xyz=(-0.052, 0.0, 0.031)),
        material=body_red,
        name="rear_bulkhead",
    )
    body.visual(
        Box((0.070, 0.018, 0.022)),
        origin=Origin(xyz=(-0.080, 0.045, 0.041)),
        material=body_red,
        name="rear_left_fender",
    )
    body.visual(
        Box((0.070, 0.018, 0.022)),
        origin=Origin(xyz=(-0.080, -0.045, 0.041)),
        material=body_red,
        name="rear_right_fender",
    )
    body.visual(
        Box((0.014, 0.098, 0.020)),
        origin=Origin(xyz=(-0.130, 0.0, 0.026)),
        material=body_red,
        name="tail_panel",
    )
    body.visual(
        Box((0.014, 0.090, 0.010)),
        origin=Origin(xyz=(-0.090, 0.0, 0.026)),
        material=dark_steel,
        name="rear_axle_beam",
    )
    body.visual(
        Box((0.038, 0.024, 0.020)),
        origin=Origin(xyz=(-0.078, 0.0, 0.024)),
        material=dark_steel,
        name="rear_axle_support",
    )
    body.visual(
        Box((0.012, 0.018, 0.012)),
        origin=Origin(xyz=(-0.090, 0.049, 0.026)),
        material=dark_steel,
        name="rear_left_hub_support",
    )
    body.visual(
        Box((0.012, 0.018, 0.012)),
        origin=Origin(xyz=(-0.090, -0.049, 0.026)),
        material=dark_steel,
        name="rear_right_hub_support",
    )
    body.visual(
        Box((0.014, 0.050, 0.018)),
        origin=Origin(xyz=(-0.130, 0.0, 0.044)),
        material=dark_steel,
        name="trunk_hinge_support",
    )
    body.visual(
        Box((0.050, 0.010, 0.014)),
        origin=Origin(xyz=(-0.090, 0.038, 0.047)),
        material=body_red,
        name="left_rear_deck_rail",
    )
    body.visual(
        Box((0.050, 0.010, 0.014)),
        origin=Origin(xyz=(-0.090, -0.038, 0.047)),
        material=body_red,
        name="right_rear_deck_rail",
    )
    body.visual(
        Box((0.028, 0.025, 0.014)),
        origin=Origin(xyz=(-0.020, 0.022, 0.025)),
        material=seat_vinyl,
        name="left_seat_base",
    )
    body.visual(
        Box((0.028, 0.025, 0.014)),
        origin=Origin(xyz=(-0.020, -0.022, 0.025)),
        material=seat_vinyl,
        name="right_seat_base",
    )
    body.visual(
        Box((0.022, 0.025, 0.030)),
        origin=Origin(xyz=(-0.034, 0.022, 0.041)),
        material=seat_vinyl,
        name="left_seat_back",
    )
    body.visual(
        Box((0.022, 0.025, 0.030)),
        origin=Origin(xyz=(-0.034, -0.022, 0.041)),
        material=seat_vinyl,
        name="right_seat_back",
    )
    body.visual(
        Box((0.008, 0.006, 0.040)),
        origin=Origin(xyz=(0.018, 0.038, 0.062)),
        material=dark_steel,
        name="left_windshield_post",
    )
    body.visual(
        Box((0.008, 0.006, 0.040)),
        origin=Origin(xyz=(0.018, -0.038, 0.062)),
        material=dark_steel,
        name="right_windshield_post",
    )
    body.visual(
        Box((0.008, 0.082, 0.006)),
        origin=Origin(xyz=(0.020, 0.0, 0.081)),
        material=dark_steel,
        name="windshield_header",
    )
    body.visual(
        Box((0.004, 0.078, 0.030)),
        origin=Origin(xyz=(0.020, 0.0, 0.066)),
        material=windshield_glass,
        name="windshield",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.276, 0.106, 0.092)),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
    )

    hood = model.part("hood")
    hood.visual(
        Box((0.122, 0.068, 0.012)),
        origin=Origin(xyz=(0.061, 0.0, 0.006)),
        material=body_red,
        name="hood_panel",
    )
    hood.visual(
        Box((0.070, 0.038, 0.006)),
        origin=Origin(xyz=(0.050, 0.0, 0.013)),
        material=body_red,
        name="hood_crown",
    )
    hood.visual(
        Box((0.004, 0.046, 0.010)),
        origin=Origin(xyz=(-0.002, 0.0, -0.005)),
        material=dark_steel,
        name="hood_hinge_leaf",
    )
    hood.inertial = Inertial.from_geometry(
        Box((0.122, 0.068, 0.016)),
        mass=0.12,
        origin=Origin(xyz=(0.061, 0.0, 0.008)),
    )

    trunk_lid = model.part("trunk_lid")
    trunk_lid.visual(
        Box((0.072, 0.064, 0.010)),
        origin=Origin(xyz=(0.036, 0.0, 0.005)),
        material=body_red,
        name="trunk_panel",
    )
    trunk_lid.visual(
        Box((0.040, 0.034, 0.006)),
        origin=Origin(xyz=(0.028, 0.0, 0.011)),
        material=body_red,
        name="trunk_crown",
    )
    trunk_lid.visual(
        Box((0.006, 0.044, 0.010)),
        origin=Origin(xyz=(-0.003, 0.0, -0.004)),
        material=dark_steel,
        name="trunk_hinge_leaf",
    )
    trunk_lid.inertial = Inertial.from_geometry(
        Box((0.072, 0.064, 0.014)),
        mass=0.08,
        origin=Origin(xyz=(0.036, 0.0, 0.007)),
    )

    wheel_radius = 0.026
    wheel_width = 0.012
    wheel_inertial_origin = Origin(rpy=(-pi / 2.0, 0.0, 0.0))

    front_left_wheel = model.part("front_left_wheel")
    _add_wheel_visuals(
        front_left_wheel,
        radius=wheel_radius,
        width=wheel_width,
        tire_mat=black_rubber,
        rim_mat=steel,
        hub_mat=dark_steel,
    )
    front_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=wheel_width),
        mass=0.05,
        origin=wheel_inertial_origin,
    )

    front_right_wheel = model.part("front_right_wheel")
    _add_wheel_visuals(
        front_right_wheel,
        radius=wheel_radius,
        width=wheel_width,
        tire_mat=black_rubber,
        rim_mat=steel,
        hub_mat=dark_steel,
    )
    front_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=wheel_width),
        mass=0.05,
        origin=wheel_inertial_origin,
    )

    rear_left_wheel = model.part("rear_left_wheel")
    _add_wheel_visuals(
        rear_left_wheel,
        radius=wheel_radius,
        width=wheel_width,
        tire_mat=black_rubber,
        rim_mat=steel,
        hub_mat=dark_steel,
    )
    rear_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=wheel_width),
        mass=0.05,
        origin=wheel_inertial_origin,
    )

    rear_right_wheel = model.part("rear_right_wheel")
    _add_wheel_visuals(
        rear_right_wheel,
        radius=wheel_radius,
        width=wheel_width,
        tire_mat=black_rubber,
        rim_mat=steel,
        hub_mat=dark_steel,
    )
    rear_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=wheel_width),
        mass=0.05,
        origin=wheel_inertial_origin,
    )

    model.articulation(
        "body_to_hood",
        ArticulationType.REVOLUTE,
        parent=body,
        child=hood,
        origin=Origin(xyz=(0.030, 0.0, 0.053)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=4.0, lower=0.0, upper=1.15),
    )
    model.articulation(
        "body_to_trunk_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=trunk_lid,
        origin=Origin(xyz=(-0.137, 0.0, 0.051)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.8, velocity=4.0, lower=0.0, upper=1.05),
    )
    model.articulation(
        "front_left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=front_left_wheel,
        origin=Origin(xyz=(0.086, 0.064, 0.026)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=30.0),
    )
    model.articulation(
        "front_right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=front_right_wheel,
        origin=Origin(xyz=(0.086, -0.064, 0.026)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=30.0),
    )
    model.articulation(
        "rear_left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=rear_left_wheel,
        origin=Origin(xyz=(-0.090, 0.064, 0.026)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=30.0),
    )
    model.articulation(
        "rear_right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=rear_right_wheel,
        origin=Origin(xyz=(-0.090, -0.064, 0.026)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=30.0),
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
    part_names = (
        "body",
        "hood",
        "trunk_lid",
        "front_left_wheel",
        "front_right_wheel",
        "rear_left_wheel",
        "rear_right_wheel",
    )
    parts = {}
    for name in part_names:
        try:
            parts[name] = object_model.get_part(name)
            ctx.check(f"{name} present", True)
        except Exception as exc:
            ctx.check(f"{name} present", False, details=str(exc))

    joint_names = (
        "body_to_hood",
        "body_to_trunk_lid",
        "front_left_wheel_spin",
        "front_right_wheel_spin",
        "rear_left_wheel_spin",
        "rear_right_wheel_spin",
    )
    joints = {}
    for name in joint_names:
        try:
            joints[name] = object_model.get_articulation(name)
            ctx.check(f"{name} present", True)
        except Exception as exc:
            ctx.check(f"{name} present", False, details=str(exc))

    if {"body", "hood", "trunk_lid"} <= parts.keys():
        ctx.expect_overlap(parts["hood"], parts["body"], axes="xy", min_overlap=0.06, name="hood covers front deck")
        ctx.expect_overlap(parts["trunk_lid"], parts["body"], axes="xy", min_overlap=0.05, name="trunk covers rear deck")

    if {"front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel"} <= parts.keys():
        ctx.expect_origin_gap(
            parts["front_left_wheel"],
            parts["rear_left_wheel"],
            axis="x",
            min_gap=0.15,
            name="left front wheel is ahead of left rear wheel",
        )
        ctx.expect_origin_gap(
            parts["front_left_wheel"],
            parts["front_right_wheel"],
            axis="y",
            min_gap=0.12,
            name="front wheels are spread across the body",
        )
        ctx.expect_origin_gap(
            parts["rear_left_wheel"],
            parts["rear_right_wheel"],
            axis="y",
            min_gap=0.12,
            name="rear wheels are spread across the body",
        )

    for joint_name in (
        "front_left_wheel_spin",
        "front_right_wheel_spin",
        "rear_left_wheel_spin",
        "rear_right_wheel_spin",
    ):
        joint = joints.get(joint_name)
        if joint is None:
            continue
        limits = joint.motion_limits
        axis = joint.axis
        ctx.check(
            f"{joint_name} is continuous wheel spin",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and limits is not None
            and limits.lower is None
            and limits.upper is None
            and abs(axis[0]) < 1e-9
            and abs(abs(axis[1]) - 1.0) < 1e-9
            and abs(axis[2]) < 1e-9,
            details=f"type={joint.articulation_type}, axis={axis}, limits={limits}",
        )

    hood_joint = joints.get("body_to_hood")
    hood = parts.get("hood")
    if hood_joint is not None and hood is not None:
        closed_aabb = ctx.part_world_aabb(hood)
        open_q = hood_joint.motion_limits.upper if hood_joint.motion_limits is not None else None
        with ctx.pose({hood_joint: open_q if open_q is not None else 0.9}):
            open_aabb = ctx.part_world_aabb(hood)
        ctx.check(
            "hood opens upward from rear hinge",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[1][2] > closed_aabb[1][2] + 0.05,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )
        ctx.check(
            "hood hinge axis is horizontal across the car",
            hood_joint.articulation_type == ArticulationType.REVOLUTE and hood_joint.axis == (0.0, -1.0, 0.0),
            details=f"type={hood_joint.articulation_type}, axis={hood_joint.axis}",
        )

    trunk_joint = joints.get("body_to_trunk_lid")
    trunk_lid = parts.get("trunk_lid")
    if trunk_joint is not None and trunk_lid is not None:
        closed_aabb = ctx.part_world_aabb(trunk_lid)
        open_q = trunk_joint.motion_limits.upper if trunk_joint.motion_limits is not None else None
        with ctx.pose({trunk_joint: open_q if open_q is not None else 0.8}):
            open_aabb = ctx.part_world_aabb(trunk_lid)
        ctx.check(
            "trunk opens upward from tail hinge",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[1][2] > closed_aabb[1][2] + 0.03,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )
        ctx.check(
            "trunk hinge axis is horizontal across the car",
            trunk_joint.articulation_type == ArticulationType.REVOLUTE and trunk_joint.axis == (0.0, -1.0, 0.0),
            details=f"type={trunk_joint.articulation_type}, axis={trunk_joint.axis}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
