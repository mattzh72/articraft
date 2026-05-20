from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_a_frame_step_ladder")

    aluminum = model.material("aluminum", color=(0.78, 0.80, 0.82))
    tread_gray = model.material("tread_gray", color=(0.70, 0.72, 0.74))
    cap_red = model.material("cap_red", color=(0.82, 0.12, 0.10))
    rubber = model.material("rubber", color=(0.12, 0.12, 0.12))

    front_frame = model.part("front_frame")
    rear_frame = model.part("rear_frame")

    rail_depth = 0.024
    front_rail_width = 0.038
    rear_rail_width = 0.032
    rail_span_y = 0.362
    left_y = rail_span_y / 2.0
    right_y = -left_y

    def add_sloped_rail(
        part,
        *,
        name: str,
        top_x: float,
        top_z: float,
        bottom_x: float,
        bottom_z: float,
        y: float,
        width_y: float,
        material,
    ) -> None:
        length = math.hypot(bottom_x - top_x, bottom_z - top_z)
        pitch = math.atan2(top_x - bottom_x, top_z - bottom_z)
        part.visual(
            Box((rail_depth, width_y, length)),
            origin=Origin(
                xyz=((top_x + bottom_x) / 2.0, y, (top_z + bottom_z) / 2.0),
                rpy=(0.0, pitch, 0.0),
            ),
            material=material,
            name=name,
        )

    front_top = (0.030, -0.070)
    front_bottom = (0.228, -1.085)
    rear_top = (-0.044, -0.060)
    rear_bottom = (-0.254, -1.040)

    add_sloped_rail(
        front_frame,
        name="front_left_rail",
        top_x=front_top[0],
        top_z=front_top[1],
        bottom_x=front_bottom[0],
        bottom_z=front_bottom[1],
        y=left_y,
        width_y=front_rail_width,
        material=aluminum,
    )
    add_sloped_rail(
        front_frame,
        name="front_right_rail",
        top_x=front_top[0],
        top_z=front_top[1],
        bottom_x=front_bottom[0],
        bottom_z=front_bottom[1],
        y=right_y,
        width_y=front_rail_width,
        material=aluminum,
    )

    front_frame.visual(
        Box((0.150, 0.392, 0.070)),
        origin=Origin(xyz=(0.085, 0.0, -0.035)),
        material=cap_red,
        name="top_cap",
    )
    hinge_y = 0.162
    for side_name, y in (("left", hinge_y), ("right", -hinge_y)):
        front_frame.visual(
            Cylinder(radius=0.010, length=0.072),
            origin=Origin(xyz=(0.010, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=aluminum,
            name=f"front_hinge_barrel_{side_name}",
        )

    def front_x_at_z(z: float) -> float:
        t = (z - front_top[1]) / (front_bottom[1] - front_top[1])
        return front_top[0] + (front_bottom[0] - front_top[0]) * t

    for idx, z in enumerate((-0.255, -0.475, -0.695, -0.915), start=1):
        x = front_x_at_z(z) + 0.020
        front_frame.visual(
            Box((0.100, 0.342, 0.028)),
            origin=Origin(xyz=(x, 0.0, z)),
            material=tread_gray,
            name=f"tread_{idx}",
        )
        front_frame.visual(
            Box((0.012, 0.342, 0.020)),
            origin=Origin(xyz=(x + 0.044, 0.0, z + 0.004)),
            material=aluminum,
            name=f"tread_nosing_{idx}",
        )

    for side_name, y in (("left", left_y), ("right", right_y)):
        front_frame.visual(
            Box((0.062, 0.050, 0.022)),
            origin=Origin(xyz=(front_bottom[0] + 0.008, y, -1.092)),
            material=rubber,
            name=f"front_{side_name}_foot",
        )

    add_sloped_rail(
        rear_frame,
        name="rear_left_rail",
        top_x=rear_top[0],
        top_z=rear_top[1],
        bottom_x=rear_bottom[0],
        bottom_z=rear_bottom[1],
        y=left_y,
        width_y=rear_rail_width,
        material=aluminum,
    )
    add_sloped_rail(
        rear_frame,
        name="rear_right_rail",
        top_x=rear_top[0],
        top_z=rear_top[1],
        bottom_x=rear_bottom[0],
        bottom_z=rear_bottom[1],
        y=right_y,
        width_y=rear_rail_width,
        material=aluminum,
    )

    rear_frame.visual(
        Box((0.050, 0.346, 0.072)),
        origin=Origin(xyz=(-0.035, 0.0, -0.036)),
        material=cap_red,
        name="rear_top_bracket",
    )
    for side_name, y in (("left", hinge_y), ("right", -hinge_y)):
        rear_frame.visual(
            Cylinder(radius=0.010, length=0.072),
            origin=Origin(xyz=(-0.010, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=aluminum,
            name=f"rear_hinge_barrel_{side_name}",
        )

    def rear_x_at_z(z: float) -> float:
        t = (z - rear_top[1]) / (rear_bottom[1] - rear_top[1])
        return rear_top[0] + (rear_bottom[0] - rear_top[0]) * t

    rear_brace_z = -0.660
    rear_frame.visual(
        Box((0.022, 0.344, 0.046)),
        origin=Origin(xyz=(rear_x_at_z(rear_brace_z), 0.0, rear_brace_z)),
        material=aluminum,
        name="rear_cross_brace",
    )

    for side_name, y in (("left", left_y), ("right", right_y)):
        rear_frame.visual(
            Box((0.056, 0.044, 0.022)),
            origin=Origin(xyz=(rear_bottom[0] - 0.008, y, -1.045)),
            material=rubber,
            name=f"rear_{side_name}_foot",
        )

    model.articulation(
        "rear_frame_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=rear_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.8,
            lower=0.0,
            upper=0.52,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front_frame = object_model.get_part("front_frame")
    rear_frame = object_model.get_part("rear_frame")
    hinge = object_model.get_articulation("rear_frame_hinge")

    ctx.expect_gap(
        front_frame,
        rear_frame,
        axis="x",
        max_penetration=0.0,
        max_gap=0.01,
        name="rear support stays behind the front hinge plane when open",
    )
    ctx.expect_overlap(
        front_frame,
        rear_frame,
        axes="y",
        min_overlap=0.30,
        name="front and rear frames share the same ladder width",
    )

    rear_open_aabb = ctx.part_world_aabb(rear_frame)
    with ctx.pose({hinge: hinge.motion_limits.upper}):
        rear_folded_aabb = ctx.part_world_aabb(rear_frame)
        ctx.expect_overlap(
            front_frame,
            rear_frame,
            axes="xz",
            min_overlap=0.12,
            name="folded rear frame nests close to the front frame silhouette",
        )

    rear_open_max_x = rear_open_aabb[1][0] if rear_open_aabb is not None else None
    rear_folded_max_x = rear_folded_aabb[1][0] if rear_folded_aabb is not None else None
    ctx.check(
        "rear frame folds forward toward the top cap",
        rear_open_max_x is not None
        and rear_folded_max_x is not None
        and rear_folded_max_x > rear_open_max_x + 0.18,
        details=f"open_max_x={rear_open_max_x}, folded_max_x={rear_folded_max_x}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
