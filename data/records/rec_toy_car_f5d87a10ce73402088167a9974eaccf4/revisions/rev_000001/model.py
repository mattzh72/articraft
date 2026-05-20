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


def _wheel_visuals(part, *, tire_radius: float, tire_width: float, tire_mat, rim_mat, trim_mat) -> None:
    spin_origin = Origin(rpy=(pi / 2.0, 0.0, 0.0))
    part.visual(
        Cylinder(radius=tire_radius, length=tire_width),
        origin=spin_origin,
        material=tire_mat,
        name="tire",
    )
    part.visual(
        Cylinder(radius=tire_radius * 0.68, length=tire_width * 0.78),
        origin=spin_origin,
        material=rim_mat,
        name="rim",
    )
    part.visual(
        Cylinder(radius=tire_radius * 0.24, length=tire_width),
        origin=spin_origin,
        material=trim_mat,
        name="hub",
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    return (
        0.5 * (min_x + max_x),
        0.5 * (min_y + max_y),
        0.5 * (min_z + max_z),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="toy_delivery_van")

    body_paint = model.material("body_paint", rgba=(0.82, 0.20, 0.14, 1.0))
    roof_cream = model.material("roof_cream", rgba=(0.94, 0.92, 0.85, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.26, 0.28, 0.31, 1.0))
    bumper_gray = model.material("bumper_gray", rgba=(0.36, 0.38, 0.40, 1.0))
    window_smoke = model.material("window_smoke", rgba=(0.29, 0.35, 0.42, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.06, 1.0))
    wheel_gray = model.material("wheel_gray", rgba=(0.73, 0.74, 0.77, 1.0))
    hub_gray = model.material("hub_gray", rgba=(0.58, 0.60, 0.63, 1.0))
    lamp_amber = model.material("lamp_amber", rgba=(0.93, 0.64, 0.18, 1.0))
    lamp_red = model.material("lamp_red", rgba=(0.72, 0.10, 0.10, 1.0))

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((0.27, 0.12, 0.15)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
    )

    body.visual(
        Box((0.238, 0.105, 0.006)),
        origin=Origin(xyz=(-0.001, 0.0, 0.039)),
        material=trim_gray,
        name="floor_pan",
    )
    body.visual(
        Box((0.17, 0.105, 0.005)),
        origin=Origin(xyz=(-0.04, 0.0, 0.1295)),
        material=roof_cream,
        name="cargo_roof",
    )
    body.visual(
        Box((0.09, 0.098, 0.005)),
        origin=Origin(xyz=(0.06, 0.0, 0.128)),
        material=roof_cream,
        name="cab_roof",
    )
    body.visual(
        Box((0.165, 0.004, 0.082)),
        origin=Origin(xyz=(-0.0425, 0.0535, 0.083)),
        material=body_paint,
        name="left_side_shell",
    )
    body.visual(
        Box((0.11, 0.004, 0.082)),
        origin=Origin(xyz=(0.06, -0.0535, 0.083)),
        material=body_paint,
        name="right_front_shell",
    )
    body.visual(
        Box((0.045, 0.004, 0.082)),
        origin=Origin(xyz=(-0.1025, -0.0535, 0.083)),
        material=body_paint,
        name="right_rear_pillar",
    )
    body.visual(
        Box((0.085, 0.004, 0.016)),
        origin=Origin(xyz=(-0.0375, -0.0535, 0.049)),
        material=body_paint,
        name="right_door_sill",
    )
    body.visual(
        Box((0.085, 0.004, 0.016)),
        origin=Origin(xyz=(-0.0375, -0.0535, 0.117)),
        material=body_paint,
        name="right_door_header",
    )
    body.visual(
        Box((0.004, 0.097, 0.078)),
        origin=Origin(xyz=(0.036, 0.0, 0.081)),
        material=roof_cream,
        name="cab_bulkhead",
    )
    body.visual(
        Box((0.06, 0.095, 0.03)),
        origin=Origin(xyz=(0.087, 0.0, 0.074)),
        material=body_paint,
        name="hood",
    )
    body.visual(
        Box((0.014, 0.092, 0.05)),
        origin=Origin(xyz=(0.121, 0.0, 0.062)),
        material=body_paint,
        name="front_face",
    )
    body.visual(
        Box((0.012, 0.098, 0.014)),
        origin=Origin(xyz=(0.126, 0.0, 0.037)),
        material=bumper_gray,
        name="front_bumper",
    )
    body.visual(
        Box((0.003, 0.088, 0.052)),
        origin=Origin(xyz=(0.038, 0.0, 0.102), rpy=(0.0, -0.42, 0.0)),
        material=window_smoke,
        name="windshield",
    )
    body.visual(
        Box((0.05, 0.004, 0.035)),
        origin=Origin(xyz=(0.05, 0.0545, 0.102)),
        material=window_smoke,
        name="left_cab_window",
    )
    body.visual(
        Box((0.05, 0.004, 0.035)),
        origin=Origin(xyz=(0.05, -0.0545, 0.102)),
        material=window_smoke,
        name="right_cab_window",
    )
    body.visual(
        Box((0.072, 0.004, 0.066)),
        origin=Origin(xyz=(0.028, 0.0545, 0.083)),
        material=roof_cream,
        name="left_side_door",
    )
    body.visual(
        Box((0.006, 0.03, 0.012)),
        origin=Origin(xyz=(0.126, 0.025, 0.052)),
        material=lamp_amber,
        name="left_headlamp",
    )
    body.visual(
        Box((0.006, 0.03, 0.012)),
        origin=Origin(xyz=(0.126, -0.025, 0.052)),
        material=lamp_amber,
        name="right_headlamp",
    )
    body.visual(
        Box((0.014, 0.09, 0.015)),
        origin=Origin(xyz=(-0.126, 0.0, 0.041)),
        material=bumper_gray,
        name="rear_bumper",
    )
    body.visual(
        Box((0.006, 0.018, 0.018)),
        origin=Origin(xyz=(-0.126, 0.041, 0.051)),
        material=lamp_red,
        name="left_tail_lamp",
    )
    body.visual(
        Box((0.006, 0.018, 0.018)),
        origin=Origin(xyz=(-0.126, -0.041, 0.051)),
        material=lamp_red,
        name="right_tail_lamp",
    )
    body.visual(
        Cylinder(radius=0.004, length=0.116),
        origin=Origin(xyz=(0.075, 0.0, 0.033), rpy=(pi / 2.0, 0.0, 0.0)),
        material=trim_gray,
        name="front_axle",
    )
    body.visual(
        Cylinder(radius=0.004, length=0.116),
        origin=Origin(xyz=(-0.075, 0.0, 0.033), rpy=(pi / 2.0, 0.0, 0.0)),
        material=trim_gray,
        name="rear_axle",
    )

    side_door = model.part("side_cargo_door")
    side_door.inertial = Inertial.from_geometry(
        Box((0.08, 0.005, 0.068)),
        mass=0.08,
        origin=Origin(xyz=(-0.04, -0.0025, 0.034)),
    )
    side_door.visual(
        Box((0.08, 0.005, 0.068)),
        origin=Origin(xyz=(-0.04, -0.0025, 0.034)),
        material=roof_cream,
        name="door_panel",
    )
    side_door.visual(
        Box((0.05, 0.0015, 0.02)),
        origin=Origin(xyz=(-0.044, -0.0056, 0.051)),
        material=window_smoke,
        name="door_window",
    )
    side_door.visual(
        Box((0.012, 0.002, 0.006)),
        origin=Origin(xyz=(-0.024, -0.006, 0.036)),
        material=trim_gray,
        name="door_handle",
    )

    rear_hatch = model.part("rear_hatch")
    rear_hatch.inertial = Inertial.from_geometry(
        Box((0.005, 0.094, 0.082)),
        mass=0.09,
        origin=Origin(xyz=(-0.0025, 0.0, -0.041)),
    )
    rear_hatch.visual(
        Box((0.005, 0.094, 0.082)),
        origin=Origin(xyz=(-0.0025, 0.0, -0.041)),
        material=roof_cream,
        name="hatch_panel",
    )
    rear_hatch.visual(
        Box((0.0015, 0.07, 0.036)),
        origin=Origin(xyz=(-0.0055, 0.0, -0.026)),
        material=window_smoke,
        name="hatch_window",
    )
    rear_hatch.visual(
        Box((0.002, 0.028, 0.007)),
        origin=Origin(xyz=(-0.006, 0.0, -0.067)),
        material=trim_gray,
        name="hatch_handle",
    )

    front_left_wheel = model.part("front_left_wheel")
    front_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.028, length=0.016),
        mass=0.05,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _wheel_visuals(front_left_wheel, tire_radius=0.028, tire_width=0.016, tire_mat=rubber, rim_mat=wheel_gray, trim_mat=hub_gray)

    front_right_wheel = model.part("front_right_wheel")
    front_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.028, length=0.016),
        mass=0.05,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _wheel_visuals(front_right_wheel, tire_radius=0.028, tire_width=0.016, tire_mat=rubber, rim_mat=wheel_gray, trim_mat=hub_gray)

    rear_left_wheel = model.part("rear_left_wheel")
    rear_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.028, length=0.016),
        mass=0.05,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _wheel_visuals(rear_left_wheel, tire_radius=0.028, tire_width=0.016, tire_mat=rubber, rim_mat=wheel_gray, trim_mat=hub_gray)

    rear_right_wheel = model.part("rear_right_wheel")
    rear_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.028, length=0.016),
        mass=0.05,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _wheel_visuals(rear_right_wheel, tire_radius=0.028, tire_width=0.016, tire_mat=rubber, rim_mat=wheel_gray, trim_mat=hub_gray)

    model.articulation(
        "side_cargo_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=side_door,
        origin=Origin(xyz=(0.005, -0.0555, 0.049)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.2, velocity=2.0, lower=0.0, upper=1.45),
    )
    model.articulation(
        "rear_hatch_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rear_hatch,
        origin=Origin(xyz=(-0.126, 0.0, 0.126)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=2.0, lower=0.0, upper=1.2),
    )
    model.articulation(
        "front_left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=front_left_wheel,
        origin=Origin(xyz=(0.075, 0.066, 0.028)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=20.0),
    )
    model.articulation(
        "front_right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=front_right_wheel,
        origin=Origin(xyz=(0.075, -0.066, 0.028)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=20.0),
    )
    model.articulation(
        "rear_left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=rear_left_wheel,
        origin=Origin(xyz=(-0.075, 0.066, 0.028)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=20.0),
    )
    model.articulation(
        "rear_right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=rear_right_wheel,
        origin=Origin(xyz=(-0.075, -0.066, 0.028)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=20.0),
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

    body = object_model.get_part("body")
    side_door = object_model.get_part("side_cargo_door")
    rear_hatch = object_model.get_part("rear_hatch")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")

    door_hinge = object_model.get_articulation("side_cargo_door_hinge")
    hatch_hinge = object_model.get_articulation("rear_hatch_hinge")
    wheel_joints = [
        object_model.get_articulation("front_left_wheel_spin"),
        object_model.get_articulation("front_right_wheel_spin"),
        object_model.get_articulation("rear_left_wheel_spin"),
        object_model.get_articulation("rear_right_wheel_spin"),
    ]

    ctx.check(
        "wheel joints are continuous axle spins",
        all(
            joint.articulation_type == ArticulationType.CONTINUOUS
            and joint.motion_limits is not None
            and joint.motion_limits.lower is None
            and joint.motion_limits.upper is None
            and tuple(round(v, 6) for v in joint.axis) == (0.0, 1.0, 0.0)
            for joint in wheel_joints
        ),
        details=", ".join(f"{joint.name}: type={joint.articulation_type}, axis={joint.axis}" for joint in wheel_joints),
    )

    with ctx.pose({door_hinge: 0.0, hatch_hinge: 0.0}):
        ctx.expect_gap(
            body,
            side_door,
            axis="y",
            positive_elem="right_front_shell",
            negative_elem="door_panel",
            max_gap=0.003,
            max_penetration=0.0,
            name="side cargo door sits flush on the van side",
        )
        ctx.expect_overlap(
            side_door,
            body,
            axes="xz",
            min_overlap=0.05,
            elem_a="door_panel",
            name="side cargo door covers the side opening",
        )
        ctx.expect_gap(
            body,
            rear_hatch,
            axis="x",
            positive_elem="cargo_roof",
            negative_elem="hatch_panel",
            max_gap=0.004,
            max_penetration=0.0,
            name="rear hatch closes over the rear opening",
        )
        ctx.expect_overlap(
            rear_hatch,
            body,
            axes="yz",
            min_overlap=0.06,
            elem_a="hatch_panel",
            name="rear hatch spans the cargo opening",
        )

    door_closed_center = _aabb_center(ctx.part_element_world_aabb(side_door, elem="door_panel"))
    hatch_closed_center = _aabb_center(ctx.part_element_world_aabb(rear_hatch, elem="hatch_panel"))
    with ctx.pose({door_hinge: 1.1, hatch_hinge: 0.95}):
        door_open_center = _aabb_center(ctx.part_element_world_aabb(side_door, elem="door_panel"))
        hatch_open_center = _aabb_center(ctx.part_element_world_aabb(rear_hatch, elem="hatch_panel"))

    ctx.check(
        "side cargo door swings outward on a vertical hinge",
        door_closed_center is not None
        and door_open_center is not None
        and door_open_center[1] < door_closed_center[1] - 0.018,
        details=f"closed={door_closed_center}, open={door_open_center}",
    )
    ctx.check(
        "rear hatch lifts upward from the top hinge",
        hatch_closed_center is not None
        and hatch_open_center is not None
        and hatch_open_center[2] > hatch_closed_center[2] + 0.015
        and hatch_open_center[0] < hatch_closed_center[0] - 0.015,
        details=f"closed={hatch_closed_center}, open={hatch_open_center}",
    )

    wheel_positions = [
        ctx.part_world_position(front_left_wheel),
        ctx.part_world_position(front_right_wheel),
        ctx.part_world_position(rear_left_wheel),
        ctx.part_world_position(rear_right_wheel),
    ]
    ctx.check(
        "wheel layout matches a four-wheel van stance",
        all(pos is not None for pos in wheel_positions)
        and wheel_positions[0][0] > wheel_positions[2][0] + 0.10
        and wheel_positions[1][0] > wheel_positions[3][0] + 0.10
        and wheel_positions[0][1] > 0.05
        and wheel_positions[1][1] < -0.05
        and wheel_positions[2][1] > 0.05
        and wheel_positions[3][1] < -0.05
        and all(abs(pos[2] - 0.028) < 1e-6 for pos in wheel_positions),
        details=f"wheel_positions={wheel_positions}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
