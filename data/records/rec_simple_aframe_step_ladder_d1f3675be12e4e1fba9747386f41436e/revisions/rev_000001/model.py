from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _bar_between(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    thickness: float,
    material: Material,
    name: str,
) -> None:
    """Add a rectangular tube whose local +Z axis runs from start to end."""
    sx, sy, sz = start
    ex, ey, ez = end
    vx, vy, vz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(vx * vx + vy * vy + vz * vz)
    radial = math.sqrt(vx * vx + vy * vy)
    yaw = math.atan2(vy, vx) if radial > 1e-9 else 0.0
    pitch = math.atan2(radial, vz)
    part.visual(
        Box((thickness, thickness, length)),
        origin=Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_a_frame_step_ladder")

    aluminum = model.material("satin_aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
    tread_aluminum = model.material("brushed_tread_aluminum", rgba=(0.58, 0.60, 0.58, 1.0))
    safety_yellow = model.material("safety_yellow_plastic", rgba=(1.0, 0.77, 0.05, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.015, 0.013, 1.0))
    hinge_steel = model.material("dark_hinge_steel", rgba=(0.10, 0.11, 0.12, 1.0))

    front = model.part("front_frame")
    rear = model.part("rear_frame")

    # Front climbing frame: two inclined side rails, wide horizontal treads,
    # rubber feet, a top cap, and the visible hinge pin at the rear of the cap.
    hinge_origin = (-0.12, 0.0, 1.16)
    rail_y = 0.24
    front_top_z = 1.18
    front_foot_z = 0.055
    front_top_x = 0.035
    front_foot_x = 0.365

    for side, y in enumerate((-rail_y, rail_y)):
        suffix = str(side)
        _bar_between(
            front,
            (front_foot_x, y, front_foot_z),
            (front_top_x, y, front_top_z),
            thickness=0.045,
            material=aluminum,
            name=f"front_rail_{suffix}",
        )
        front.visual(
            Box((0.14, 0.115, 0.050)),
            origin=Origin(xyz=(front_foot_x + 0.015, y, 0.025)),
            material=black_rubber,
            name=f"front_foot_{suffix}",
        )

    step_specs = (
        (0.35, 0.265, 0.23),
        (0.64, 0.180, 0.215),
        (0.93, 0.095, 0.200),
    )
    for idx, (z, x, depth) in enumerate(step_specs):
        front.visual(
            Box((depth, 0.58, 0.040)),
            origin=Origin(xyz=(x, 0.0, z)),
            material=tread_aluminum,
            name=f"tread_{idx}",
        )
        # Raised dark ribs communicate broad, non-slip climbing treads.
        for rib_idx, rib_x in enumerate((x - depth * 0.26, x + depth * 0.26)):
            front.visual(
                Box((0.018, 0.52, 0.008)),
                origin=Origin(xyz=(rib_x, 0.0, z + 0.024)),
                material=black_rubber,
                name=f"tread_rib_{idx}_{rib_idx}",
            )

    front.visual(
        Box((0.34, 0.64, 0.070)),
        origin=Origin(xyz=(0.020, 0.0, 1.205)),
        material=safety_yellow,
        name="top_cap",
    )
    front.visual(
        Box((0.040, 0.50, 0.060)),
        origin=Origin(xyz=(-0.115, 0.0, 1.145)),
        material=hinge_steel,
        name="hinge_leaf",
    )
    front.visual(
        Cylinder(radius=0.018, length=0.66),
        origin=Origin(xyz=hinge_origin, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=hinge_steel,
        name="hinge_pin",
    )

    # Rear support frame is authored in the hinge frame, so q=0 is the open
    # A-frame stance and positive motion folds it forward toward the treads.
    rear_rail_y = 0.225
    rear_top = (-0.055, 0.0, -0.045)
    rear_foot = (-0.565, 0.0, -1.115)
    for side, y in enumerate((-rear_rail_y, rear_rail_y)):
        suffix = str(side)
        _bar_between(
            rear,
            (rear_foot[0], y, rear_foot[2]),
            (rear_top[0], y, rear_top[2]),
            thickness=0.042,
            material=aluminum,
            name=f"rear_rail_{suffix}",
        )
        rear.visual(
            Box((0.145, 0.105, 0.035)),
            origin=Origin(xyz=(rear_foot[0] - 0.010, y, rear_foot[2] - 0.010)),
            material=black_rubber,
            name=f"rear_foot_{suffix}",
        )

    for idx, z in enumerate((-0.38, -0.72, -0.96)):
        t = (z - rear_foot[2]) / (rear_top[2] - rear_foot[2])
        x = rear_foot[0] + (rear_top[0] - rear_foot[0]) * t
        rear.visual(
            Box((0.050, 0.54, 0.032)),
            origin=Origin(xyz=(x, 0.0, z)),
            material=aluminum,
            name=f"rear_crossbar_{idx}",
        )

    _bar_between(
        rear,
        (-0.505, -rear_rail_y, -0.98),
        (-0.205, rear_rail_y, -0.39),
        thickness=0.026,
        material=aluminum,
        name="rear_diagonal_0",
    )
    _bar_between(
        rear,
        (-0.505, rear_rail_y, -0.98),
        (-0.205, -rear_rail_y, -0.39),
        thickness=0.026,
        material=aluminum,
        name="rear_diagonal_1",
    )
    rear.visual(
        Box((0.065, 0.52, 0.035)),
        origin=Origin(xyz=(-0.065, 0.0, -0.050)),
        material=hinge_steel,
        name="rear_hinge_bridge",
    )
    rear.visual(
        Cylinder(radius=0.020, length=0.50),
        origin=Origin(xyz=(-0.035, 0.0, -0.025), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=hinge_steel,
        name="rear_hinge_tube",
    )

    model.articulation(
        "top_hinge",
        ArticulationType.REVOLUTE,
        parent=front,
        child=rear,
        origin=Origin(xyz=hinge_origin),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=0.0, upper=0.55),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front = object_model.get_part("front_frame")
    rear = object_model.get_part("rear_frame")
    hinge = object_model.get_articulation("top_hinge")

    # In the use pose the rear support must project behind the climbing frame,
    # making an unmistakable A-frame footprint.
    front_open_aabb = ctx.part_world_aabb(front)
    rear_open_aabb = ctx.part_world_aabb(rear)
    ctx.check(
        "open rear frame supports behind front frame",
        front_open_aabb is not None
        and rear_open_aabb is not None
        and rear_open_aabb[0][0] < front_open_aabb[0][0] - 0.20,
        details=f"front={front_open_aabb}, rear={rear_open_aabb}",
    )
    if front_open_aabb is not None and rear_open_aabb is not None:
        open_depth = max(front_open_aabb[1][0], rear_open_aabb[1][0]) - min(
            front_open_aabb[0][0], rear_open_aabb[0][0]
        )
    else:
        open_depth = None

    # At the upper limit the same single top hinge folds the rear support
    # forward, keeping the stored ladder envelope much narrower than when open.
    with ctx.pose({hinge: 0.55}):
        front_fold_aabb = ctx.part_world_aabb(front)
        rear_fold_aabb = ctx.part_world_aabb(rear)
        if front_fold_aabb is not None and rear_fold_aabb is not None:
            folded_depth = max(front_fold_aabb[1][0], rear_fold_aabb[1][0]) - min(
                front_fold_aabb[0][0], rear_fold_aabb[0][0]
            )
        else:
            folded_depth = None
        ctx.check(
            "folded envelope is narrow",
            open_depth is not None
            and folded_depth is not None
            and folded_depth < 1.10
            and folded_depth < open_depth - 0.15,
            details=f"open_depth={open_depth}, folded_depth={folded_depth}",
        )

    return ctx.report()


object_model = build_object_model()
