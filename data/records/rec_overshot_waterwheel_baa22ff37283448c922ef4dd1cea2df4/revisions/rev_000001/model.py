from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


def _rot_x(point: tuple[float, float, float], angle: float) -> tuple[float, float, float]:
    x, y, z = point
    c = math.cos(angle)
    s = math.sin(angle)
    return (x, y * c - z * s, y * s + z * c)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="narrow_steel_overshot_waterwheel")

    galvanized = model.material("galvanized_steel", rgba=(0.62, 0.66, 0.68, 1.0))
    dark_steel = model.material("dark_oiled_steel", rgba=(0.16, 0.17, 0.18, 1.0))
    worn_steel = model.material("worn_gate_steel", rgba=(0.46, 0.48, 0.47, 1.0))

    frame = model.part("frame")

    # Ground frame and two simple upright supports.
    frame.visual(Box((1.10, 0.08, 0.07)), origin=Origin(xyz=(0.0, -0.44, 0.035)), material=galvanized, name="front_foot")
    frame.visual(Box((1.10, 0.08, 0.07)), origin=Origin(xyz=(0.0, 0.38, 0.035)), material=galvanized, name="rear_foot")
    frame.visual(Box((0.08, 0.88, 0.06)), origin=Origin(xyz=(-0.50, -0.03, 0.09)), material=galvanized, name="base_rail_0")
    frame.visual(Box((0.08, 0.88, 0.06)), origin=Origin(xyz=(0.50, -0.03, 0.09)), material=galvanized, name="base_rail_1")
    frame.visual(Box((0.08, 0.08, 1.54)), origin=Origin(xyz=(-0.42, 0.0, 0.80)), material=galvanized, name="upright_0")
    frame.visual(Box((0.08, 0.08, 1.54)), origin=Origin(xyz=(0.42, 0.0, 0.80)), material=galvanized, name="upright_1")
    frame.visual(Box((0.92, 0.08, 0.08)), origin=Origin(xyz=(0.0, 0.0, 1.58)), material=galvanized, name="top_crossbeam")
    frame.visual(Box((0.92, 0.06, 0.06)), origin=Origin(xyz=(0.0, -0.50, 1.50)), material=galvanized, name="chute_crossbeam")
    frame.visual(Box((0.06, 0.58, 0.06)), origin=Origin(xyz=(-0.42, -0.25, 1.52)), material=galvanized, name="chute_arm_0")
    frame.visual(Box((0.06, 0.58, 0.06)), origin=Origin(xyz=(0.42, -0.25, 1.52)), material=galvanized, name="chute_arm_1")

    # Bearing bosses touch the wheel shaft at their inner faces.
    frame.visual(
        Cylinder(radius=0.065, length=0.060),
        origin=Origin(xyz=(-0.370, 0.0, 0.82), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="bearing_0",
    )
    frame.visual(
        Cylinder(radius=0.065, length=0.060),
        origin=Origin(xyz=(0.370, 0.0, 0.82), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="bearing_1",
    )

    # Overhead steel trough/chute feeding the top of the overshot wheel.
    frame.visual(Box((0.34, 0.385, 0.025)), origin=Origin(xyz=(0.0, -0.3475, 1.56)), material=galvanized, name="chute_bottom")
    frame.visual(Box((0.34, 0.145, 0.025)), origin=Origin(xyz=(0.0, -0.0325, 1.56)), material=galvanized, name="chute_bottom_tail")
    frame.visual(Box((0.025, 0.62, 0.13)), origin=Origin(xyz=(-0.182, -0.25, 1.615)), material=galvanized, name="chute_wall_0")
    frame.visual(Box((0.025, 0.62, 0.13)), origin=Origin(xyz=(0.182, -0.25, 1.615)), material=galvanized, name="chute_wall_1")
    frame.visual(Box((0.27, 0.035, 0.10)), origin=Origin(xyz=(0.0, 0.040, 1.605)), material=galvanized, name="outlet_lip")

    # Vertical guide slots around the sliding inlet gate.  The paired lips bracket
    # the gate in the chute direction so the plate remains clipped while moving.
    for x, front_name, rear_name, top_name, bottom_name in (
        (-0.165, "front_guide_0", "rear_guide_0", "top_stop_0", "bottom_stop_0"),
        (0.165, "front_guide_1", "rear_guide_1", "top_stop_1", "bottom_stop_1"),
    ):
        frame.visual(Box((0.035, 0.014, 0.650)), origin=Origin(xyz=(x, -0.153, 1.85)), material=dark_steel, name=front_name)
        frame.visual(Box((0.035, 0.014, 0.650)), origin=Origin(xyz=(x, -0.107, 1.85)), material=dark_steel, name=rear_name)
        frame.visual(Box((0.040, 0.060, 0.030)), origin=Origin(xyz=(x, -0.130, 2.175)), material=dark_steel, name=top_name)
        frame.visual(Box((0.040, 0.060, 0.030)), origin=Origin(xyz=(x, -0.130, 1.525)), material=dark_steel, name=bottom_name)

    wheel = model.part("wheel")
    wheel.visual(
        mesh_from_geometry(
            WheelGeometry(
                0.605,
                0.205,
                rim=WheelRim(inner_radius=0.525, flange_height=0.016, flange_thickness=0.006, bead_seat_depth=0.004),
                hub=WheelHub(radius=0.072, width=0.245, cap_style="domed"),
                face=WheelFace(dish_depth=0.010, front_inset=0.004, rear_inset=0.004),
                spokes=WheelSpokes(style="straight", count=10, thickness=0.010, window_radius=0.035),
                bore=WheelBore(style="round", diameter=0.052),
            ),
            "waterwheel_rim",
        ),
        material=dark_steel,
        name="rim_spokes",
    )
    wheel.visual(
        Cylinder(radius=0.026, length=0.680),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="shaft",
    )

    # Narrow overshot buckets: thin U-shaped steel troughs around the rim.
    bucket_count = 16
    for i in range(bucket_count):
        theta = 2.0 * math.pi * i / bucket_count
        roll = theta - math.pi / 2.0
        radial_y = 0.610 * math.cos(theta)
        radial_z = 0.610 * math.sin(theta)
        bucket_center = (0.0, radial_y, radial_z)
        bucket_specs = (
            (f"bucket_{i}_floor", Box((0.215, 0.120, 0.018)), (0.0, 0.0, 0.0)),
            (f"bucket_{i}_lip_0", Box((0.215, 0.014, 0.078)), (0.0, -0.053, 0.033)),
            (f"bucket_{i}_lip_1", Box((0.215, 0.014, 0.078)), (0.0, 0.053, 0.033)),
            (f"bucket_{i}_side_0", Box((0.012, 0.120, 0.072)), (-0.108, 0.0, 0.030)),
            (f"bucket_{i}_side_1", Box((0.012, 0.120, 0.072)), (0.108, 0.0, 0.030)),
        )
        for name, geometry, local_offset in bucket_specs:
            dx, dy, dz = _rot_x(local_offset, roll)
            wheel.visual(
                geometry,
                origin=Origin(xyz=(bucket_center[0] + dx, bucket_center[1] + dy, bucket_center[2] + dz), rpy=(roll, 0.0, 0.0)),
                material=worn_steel,
                name=name,
            )

    gate = model.part("gate")
    gate.visual(Box((0.292, 0.016, 0.340)), origin=Origin(), material=worn_steel, name="gate_plate")
    gate.visual(Box((0.030, 0.020, 0.365)), origin=Origin(xyz=(-0.158, 0.0, 0.0)), material=dark_steel, name="edge_runner_0")
    gate.visual(Box((0.030, 0.020, 0.365)), origin=Origin(xyz=(0.158, 0.0, 0.0)), material=dark_steel, name="edge_runner_1")
    gate.visual(Box((0.240, 0.030, 0.030)), origin=Origin(xyz=(0.0, -0.018, 0.175)), material=dark_steel, name="lift_handle")

    model.articulation(
        "frame_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, 0.82)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=2.0),
    )

    model.articulation(
        "frame_to_gate",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=gate,
        origin=Origin(xyz=(0.0, -0.130, 1.72)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.25, lower=0.0, upper=0.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    wheel = object_model.get_part("wheel")
    gate = object_model.get_part("gate")
    wheel_joint = object_model.get_articulation("frame_to_wheel")
    gate_joint = object_model.get_articulation("frame_to_gate")

    ctx.expect_contact(wheel, frame, elem_a="shaft", elem_b="bearing_0", contact_tol=0.003, name="shaft touches first bearing")
    ctx.expect_contact(wheel, frame, elem_a="shaft", elem_b="bearing_1", contact_tol=0.003, name="shaft touches second bearing")

    ctx.expect_within(gate, frame, axes="x", inner_elem="gate_plate", outer_elem="chute_bottom", margin=0.002, name="gate spans inside chute width")
    ctx.expect_gap(gate, frame, axis="y", positive_elem="gate_plate", negative_elem="front_guide_0", min_gap=0.006, max_gap=0.030, name="gate clears front slot lips")
    ctx.expect_gap(frame, gate, axis="y", positive_elem="rear_guide_0", negative_elem="gate_plate", min_gap=0.006, max_gap=0.030, name="gate clears rear slot lips")
    ctx.expect_overlap(gate, frame, axes="z", elem_a="edge_runner_0", elem_b="front_guide_0", min_overlap=0.35, name="gate runner retained in slot")

    rest_pos = ctx.part_world_position(gate)
    with ctx.pose({gate_joint: 0.20, wheel_joint: math.pi / 2.0}):
        ctx.expect_overlap(gate, frame, axes="z", elem_a="edge_runner_0", elem_b="front_guide_0", min_overlap=0.22, name="raised gate remains clipped")
        raised_pos = ctx.part_world_position(gate)

    ctx.check(
        "gate slides upward",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.18,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


object_model = build_object_model()
