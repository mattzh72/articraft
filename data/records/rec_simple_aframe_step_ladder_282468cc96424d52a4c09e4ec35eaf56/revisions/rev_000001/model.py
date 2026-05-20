from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    Mimic,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def _beam_xz(
    part,
    name: str,
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
    *,
    width_y: float,
    thickness: float,
    material: Material,
) -> None:
    """Add a rectangular member whose long axis runs between two X/Z points."""
    dx = p1[0] - p0[0]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dz * dz)
    pitch = math.atan2(-dz, dx)
    part.visual(
        Box((length, width_y, thickness)),
        origin=Origin(
            xyz=((p0[0] + p1[0]) * 0.5, p0[1], (p0[2] + p1[2]) * 0.5),
            rpy=(0.0, pitch, 0.0),
        ),
        material=material,
        name=name,
    )


def _rail_x_at_z(
    z: float,
    top: tuple[float, float],
    bottom: tuple[float, float],
) -> float:
    top_x, top_z = top
    bottom_x, bottom_z = bottom
    t = (z - top_z) / (bottom_z - top_z)
    return top_x + (bottom_x - top_x) * t


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="detailed_a_frame_step_ladder")

    aluminum = model.material("brushed_aluminum", rgba=(0.74, 0.76, 0.74, 1.0))
    dark_aluminum = model.material("dark_anodized_edges", rgba=(0.22, 0.23, 0.23, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    red_plastic = model.material("red_locking_parts", rgba=(0.80, 0.05, 0.035, 1.0))
    yellow_label = model.material("yellow_warning_labels", rgba=(1.0, 0.78, 0.08, 1.0))

    front = model.part("front_frame")
    rear = model.part("rear_frame")

    # The root frame origin is the top hinge line; the open ladder extends
    # downward to roughly household step-ladder height.
    front_top = (0.045, -0.060)
    front_bottom = (0.395, -1.420)
    rear_top = (-0.105, -0.065)
    rear_bottom = (-0.585, -1.410)
    rail_y = 0.310

    # Front rigid frame: side rails, treads, top cap and feet.
    for i, side in enumerate((-1.0, 1.0)):
        y = side * rail_y
        foot_name = "front_foot_0" if i == 0 else "front_foot_1"
        _beam_xz(
            front,
            f"front_rail_{i}",
            (front_top[0], y, front_top[1]),
            (front_bottom[0], y, front_bottom[1]),
            width_y=0.050,
            thickness=0.044,
            material=aluminum,
        )
        front.visual(
            Box((0.155, 0.118, 0.038)),
            origin=Origin(xyz=(front_bottom[0] + 0.012, y, front_bottom[1] - 0.010)),
            material=black_rubber,
            name=foot_name,
        )
        front.visual(
            Box((0.066, 0.054, 0.060)),
            origin=Origin(xyz=(0.015, side * 0.305, -0.032)),
            material=dark_aluminum,
            name=f"front_hinge_cheek_{i}",
        )
        front.visual(
            Cylinder(radius=0.026, length=0.150),
            origin=Origin(xyz=(0.0, side * 0.305, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_aluminum,
            name=f"front_hinge_barrel_{i}",
        )
        front.visual(
            Cylinder(radius=0.021, length=0.046),
            origin=Origin(xyz=(0.225, side * 0.359, -0.785), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_aluminum,
            name=f"front_pivot_boss_{i}",
        )
        front.visual(
            Box((0.070, 0.050, 0.065)),
            origin=Origin(xyz=(0.225, side * 0.332, -0.785)),
            material=aluminum,
            name=f"front_pivot_plate_{i}",
        )

    front.visual(
        Box((0.285, 0.720, 0.048)),
        origin=Origin(xyz=(0.085, 0.0, -0.070)),
        material=aluminum,
        name="top_cap",
    )
    front.visual(
        Box((0.030, 0.650, 0.080)),
        origin=Origin(xyz=(0.205, 0.0, -0.100)),
        material=dark_aluminum,
        name="top_front_lip",
    )
    front.visual(
        Box((0.090, 0.250, 0.004)),
        origin=Origin(xyz=(0.215, 0.0, -0.056)),
        material=yellow_label,
        name="safety_label",
    )

    step_zs = [-1.155, -0.880, -0.600, -0.325]
    for idx, z in enumerate(step_zs):
        rail_x = _rail_x_at_z(z, front_top, front_bottom)
        center_x = rail_x - 0.060
        front.visual(
            Box((0.230, 0.630, 0.040)),
            origin=Origin(xyz=(center_x, 0.0, z)),
            material=aluminum,
            name=f"step_{idx}",
        )
        front.visual(
            Box((0.022, 0.626, 0.020)),
            origin=Origin(xyz=(center_x + 0.105, 0.0, z + 0.030)),
            material=dark_aluminum,
            name=f"step_nose_{idx}",
        )
        for ridge in range(4):
            x = center_x - 0.075 + ridge * 0.050
            front.visual(
                Box((0.010, 0.585, 0.008)),
                origin=Origin(xyz=(x, 0.0, z + 0.023)),
                material=dark_aluminum,
                name=f"tread_ridge_{idx}_{ridge}",
            )
        for side in (-1.0, 1.0):
            front.visual(
                Cylinder(radius=0.010, length=0.006),
                origin=Origin(xyz=(center_x + 0.090, side * 0.265, z + 0.023)),
                material=dark_aluminum,
                name=f"step_rivet_{idx}_{0 if side < 0 else 1}",
            )

    # Rear rigid support frame with cross braces and rubber feet.
    for i, side in enumerate((-1.0, 1.0)):
        y = side * rail_y
        foot_name = "rear_foot_0" if i == 0 else "rear_foot_1"
        _beam_xz(
            rear,
            f"rear_rail_{i}",
            (rear_top[0], y, rear_top[1]),
            (rear_bottom[0], y, rear_bottom[1]),
            width_y=0.045,
            thickness=0.040,
            material=aluminum,
        )
        rear.visual(
            Box((0.145, 0.108, 0.036)),
            origin=Origin(xyz=(rear_bottom[0] - 0.012, y, rear_bottom[1] - 0.010)),
            material=black_rubber,
            name=foot_name,
        )
        rear.visual(
            Box((0.066, 0.050, 0.060)),
            origin=Origin(xyz=(-0.340, side * 0.345, -0.785)),
            material=dark_aluminum,
            name=f"rear_spreader_socket_{i}",
        )
        rear.visual(
            Cylinder(radius=0.020, length=0.040),
            origin=Origin(xyz=(-0.340, side * 0.370, -0.785), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=red_plastic,
            name=f"rear_lock_pin_{i}",
        )

    rear.visual(
        Cylinder(radius=0.024, length=0.365),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_aluminum,
        name="rear_hinge_barrel",
    )
    rear.visual(
        Box((0.070, 0.440, 0.060)),
        origin=Origin(xyz=(-0.050, 0.0, -0.010)),
        material=aluminum,
        name="rear_hinge_yoke",
    )
    for i, side in enumerate((-1.0, 1.0)):
        rear.visual(
            Box((0.080, 0.115, 0.055)),
            origin=Origin(xyz=(-0.105, side * 0.255, -0.035)),
            material=aluminum,
            name=f"rear_hinge_bridge_{i}",
        )
    for j, z in enumerate((-0.420, -0.850, -1.225)):
        x = _rail_x_at_z(z, rear_top, rear_bottom)
        rear.visual(
            Box((0.060, 0.625, 0.038)),
            origin=Origin(xyz=(x, 0.0, z)),
            material=aluminum,
            name=f"rear_crossbar_{j}",
        )

    # Folding spreader arms: independent side links mimic the main hinge so the
    # safety bars visibly tuck up while the rear frame folds.
    spreader_len = 0.565
    flat_bar_len = 0.540
    for i, side in enumerate((-1.0, 1.0)):
        spreader = model.part(f"spreader_{i}")
        spreader.visual(
            Box((flat_bar_len, 0.018, 0.024)),
            origin=Origin(xyz=(-flat_bar_len * 0.5, 0.0, 0.0)),
            material=dark_aluminum,
            name="flat_bar",
        )
        spreader.visual(
            Cylinder(radius=0.032, length=0.014),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_aluminum,
            name="front_eye",
        )
        spreader.visual(
            Cylinder(radius=0.029, length=0.014),
            origin=Origin(xyz=(-spreader_len, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_aluminum,
            name="rear_eye",
        )
        spreader.visual(
            Box((0.070, 0.026, 0.030)),
            origin=Origin(xyz=(-spreader_len + 0.020, 0.0, 0.0)),
            material=red_plastic,
            name="spring_lock",
        )
        model.articulation(
            f"front_to_spreader_{i}",
            ArticulationType.REVOLUTE,
            parent=front,
            child=spreader,
            origin=Origin(xyz=(0.225, side * 0.389, -0.785)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=0.38),
            motion_properties=MotionProperties(damping=0.15, friction=0.05),
            mimic=Mimic("front_to_rear", multiplier=1.0, offset=0.0),
        )

    model.articulation(
        "front_to_rear",
        ArticulationType.REVOLUTE,
        parent=front,
        child=rear,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.5, lower=0.0, upper=0.38),
        motion_properties=MotionProperties(damping=0.25, friction=0.12),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front = object_model.get_part("front_frame")
    rear = object_model.get_part("rear_frame")
    hinge = object_model.get_articulation("front_to_rear")

    ctx.expect_gap(
        front,
        rear,
        axis="x",
        positive_elem="front_foot_0",
        negative_elem="rear_foot_0",
        min_gap=0.65,
        name="open stance has broad A-frame footprint",
    )

    step_aabb = ctx.part_element_world_aabb(front, elem="step_2")
    ctx.check(
        "steps span between both side rails",
        step_aabb is not None and step_aabb[0][1] < -0.29 and step_aabb[1][1] > 0.29,
        details=f"step_2_aabb={step_aabb}",
    )

    rest_foot = ctx.part_element_world_aabb(rear, elem="rear_foot_0")
    with ctx.pose({hinge: 0.38}):
        folded_foot = ctx.part_element_world_aabb(rear, elem="rear_foot_0")
        spreader = ctx.part_element_world_aabb("spreader_0", elem="rear_eye")

    if rest_foot is not None and folded_foot is not None:
        rest_x = (rest_foot[0][0] + rest_foot[1][0]) * 0.5
        folded_x = (folded_foot[0][0] + folded_foot[1][0]) * 0.5
        rest_z = (rest_foot[0][2] + rest_foot[1][2]) * 0.5
        folded_z = (folded_foot[0][2] + folded_foot[1][2]) * 0.5
    else:
        rest_x = folded_x = rest_z = folded_z = None
    ctx.check(
        "rear support folds toward the front frame",
        rest_x is not None
        and folded_x is not None
        and folded_x > rest_x + 0.35,
        details=f"rest=({rest_x}, {rest_z}), folded=({folded_x}, {folded_z})",
    )

    ctx.check(
        "spreader bars tuck up with the folding action",
        spreader is not None and spreader[0][2] > -0.72,
        details=f"folded_spreader_rear_eye_aabb={spreader}",
    )

    for i in range(2):
        rear_pin = f"rear_lock_pin_{i}"
        spreader_part = f"spreader_{i}"
        ctx.allow_overlap(
            rear,
            spreader_part,
            elem_a=rear_pin,
            elem_b="spring_lock",
            reason="The spring lock clip is intentionally modeled seated around the rear latch pin.",
        )
        ctx.expect_overlap(
            rear,
            spreader_part,
            axes="xz",
            elem_a=rear_pin,
            elem_b="spring_lock",
            min_overlap=0.018,
            name=f"spreader {i} spring clip captures the rear latch pin",
        )
        ctx.allow_overlap(
            rear,
            spreader_part,
            elem_a=rear_pin,
            elem_b="rear_eye",
            reason="The rear eye is a simplified solid bushing around the latch pin.",
        )
        ctx.expect_overlap(
            rear,
            spreader_part,
            axes="xz",
            elem_a=rear_pin,
            elem_b="rear_eye",
            min_overlap=0.018,
            name=f"spreader {i} rear eye remains on the latch pin",
        )

    return ctx.report()


object_model = build_object_model()
