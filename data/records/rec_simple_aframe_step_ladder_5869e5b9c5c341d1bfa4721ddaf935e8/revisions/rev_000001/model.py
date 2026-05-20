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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_segment(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_box_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    width: float,
    depth: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Box((width, depth, _distance(a, b))),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_segment(a, b)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_a_frame_step_ladder")

    fiberglass = model.material("fiberglass", rgba=(0.96, 0.76, 0.14, 1.0))
    aluminum = model.material("aluminum", rgba=(0.78, 0.80, 0.82, 1.0))
    plastic_gray = model.material("plastic_gray", rgba=(0.24, 0.26, 0.30, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    front_frame = model.part("front_frame")
    front_frame.inertial = Inertial.from_geometry(
        Box((0.46, 0.24, 1.12)),
        mass=6.8,
        origin=Origin(xyz=(0.0, 0.09, 0.56)),
    )

    left_front_foot = (-0.205, 0.175, 0.025)
    right_front_foot = (0.205, 0.175, 0.025)
    left_front_top = (-0.165, -0.008, 1.055)
    right_front_top = (0.165, -0.008, 1.055)

    _add_box_member(
        front_frame,
        left_front_foot,
        left_front_top,
        width=0.050,
        depth=0.024,
        material=fiberglass,
        name="left_front_rail",
    )
    _add_box_member(
        front_frame,
        right_front_foot,
        right_front_top,
        width=0.050,
        depth=0.024,
        material=fiberglass,
        name="right_front_rail",
    )

    front_frame.visual(
        Box((0.085, 0.050, 0.030)),
        origin=Origin(xyz=(-0.205, 0.175, 0.015)),
        material=rubber,
        name="left_front_foot",
    )
    front_frame.visual(
        Box((0.085, 0.050, 0.030)),
        origin=Origin(xyz=(0.205, 0.175, 0.015)),
        material=rubber,
        name="right_front_foot",
    )

    tread_data = [
        ("tread_1", 0.245, 0.126, 0.370),
        ("tread_2", 0.470, 0.088, 0.355),
        ("tread_3", 0.695, 0.049, 0.340),
        ("tread_4", 0.905, 0.012, 0.330),
    ]
    for tread_name, z, y, width in tread_data:
        front_frame.visual(
            Box((width, 0.105, 0.032)),
            origin=Origin(xyz=(0.0, y, z)),
            material=aluminum,
            name=tread_name,
        )
        front_frame.visual(
            Box((width * 0.94, 0.014, 0.006)),
            origin=Origin(xyz=(0.0, y + 0.038, z + 0.019)),
            material=plastic_gray,
            name=f"{tread_name}_nosing",
        )

    front_frame.visual(
        Box((0.372, 0.132, 0.056)),
        origin=Origin(xyz=(0.0, 0.008, 1.060)),
        material=plastic_gray,
        name="top_cap",
    )
    front_frame.visual(
        Box((0.300, 0.080, 0.018)),
        origin=Origin(xyz=(0.0, 0.050, 1.082)),
        material=plastic_gray,
        name="top_tray_lip",
    )
    front_frame.visual(
        Box((0.310, 0.040, 0.038)),
        origin=Origin(xyz=(0.0, 0.078, 1.055)),
        material=plastic_gray,
        name="front_cap_apron",
    )
    front_frame.visual(
        Cylinder(radius=0.014, length=0.050),
        origin=Origin(xyz=(-0.110, -0.034, 1.090), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="left_hinge_barrel",
    )
    front_frame.visual(
        Cylinder(radius=0.014, length=0.050),
        origin=Origin(xyz=(0.110, -0.034, 1.090), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="right_hinge_barrel",
    )

    rear_frame = model.part("rear_frame")
    rear_frame.inertial = Inertial.from_geometry(
        Box((0.48, 0.48, 1.10)),
        mass=4.4,
        origin=Origin(xyz=(0.0, -0.22, -0.54)),
    )

    left_rear_top = (-0.122, -0.070, -0.078)
    right_rear_top = (0.122, -0.070, -0.078)
    left_rear_foot = (-0.220, -0.430, -1.070)
    right_rear_foot = (0.220, -0.430, -1.070)

    _add_box_member(
        rear_frame,
        left_rear_top,
        left_rear_foot,
        width=0.042,
        depth=0.020,
        material=fiberglass,
        name="left_rear_leg",
    )
    _add_box_member(
        rear_frame,
        right_rear_top,
        right_rear_foot,
        width=0.042,
        depth=0.020,
        material=fiberglass,
        name="right_rear_leg",
    )

    rear_frame.visual(
        Box((0.075, 0.040, 0.028)),
        origin=Origin(xyz=(-0.220, -0.430, -1.076)),
        material=rubber,
        name="left_rear_foot",
    )
    rear_frame.visual(
        Box((0.075, 0.040, 0.028)),
        origin=Origin(xyz=(0.220, -0.430, -1.076)),
        material=rubber,
        name="right_rear_foot",
    )

    rear_frame.visual(
        Box((0.330, 0.040, 0.044)),
        origin=Origin(xyz=(0.0, -0.066, -0.066)),
        material=aluminum,
        name="rear_top_crossbar",
    )
    rear_frame.visual(
        Box((0.405, 0.028, 0.032)),
        origin=Origin(xyz=(0.0, -0.220, -0.435)),
        material=aluminum,
        name="rear_mid_brace",
    )
    rear_frame.visual(
        Cylinder(radius=0.013, length=0.045),
        origin=Origin(xyz=(-0.052, -0.034, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="left_rear_hinge_knuckle",
    )
    rear_frame.visual(
        Cylinder(radius=0.013, length=0.045),
        origin=Origin(xyz=(0.052, -0.034, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="right_rear_hinge_knuckle",
    )
    _add_box_member(
        rear_frame,
        (-0.052, -0.034, -0.004),
        (-0.112, -0.048, -0.060),
        width=0.024,
        depth=0.016,
        material=aluminum,
        name="left_hinge_strap",
    )
    _add_box_member(
        rear_frame,
        (0.052, -0.034, -0.004),
        (0.112, -0.048, -0.060),
        width=0.024,
        depth=0.016,
        material=aluminum,
        name="right_hinge_strap",
    )

    model.articulation(
        "rear_support_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=rear_frame,
        origin=Origin(xyz=(0.0, -0.034, 1.090)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.2,
            lower=-0.10,
            upper=0.42,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front_frame = object_model.get_part("front_frame")
    rear_frame = object_model.get_part("rear_frame")
    hinge = object_model.get_articulation("rear_support_hinge")

    ctx.expect_gap(
        front_frame,
        rear_frame,
        axis="y",
        positive_elem="left_front_foot",
        negative_elem="left_rear_foot",
        min_gap=0.50,
        name="rear support opens to a wide lower stance",
    )
    ctx.expect_overlap(
        front_frame,
        rear_frame,
        axes="x",
        elem_a="top_cap",
        elem_b="rear_top_crossbar",
        min_overlap=0.24,
        name="rear frame stays centered beneath the top cap",
    )

    left_tread = ctx.part_element_world_aabb(front_frame, elem="tread_1")
    if left_tread is not None:
        tread_width = left_tread[1][0] - left_tread[0][0]
        tread_depth = left_tread[1][1] - left_tread[0][1]
        ctx.check(
            "front tread is wide and usable",
            tread_width >= 0.34 and tread_depth >= 0.10,
            details=f"width={tread_width:.4f}, depth={tread_depth:.4f}",
        )
    else:
        ctx.fail("front tread is wide and usable", "Could not resolve tread_1 world AABB.")

    left_front_foot = ctx.part_element_world_aabb(front_frame, elem="left_front_foot")
    left_rear_foot = ctx.part_element_world_aabb(rear_frame, elem="left_rear_foot")
    if left_front_foot is not None and left_rear_foot is not None:
        front_floor_z = left_front_foot[0][2]
        rear_floor_z = left_rear_foot[0][2]
        ctx.check(
            "both frame feet rest on the ground plane",
            abs(front_floor_z) <= 0.001 and abs(rear_floor_z) <= 0.001,
            details=f"front_min_z={front_floor_z:.5f}, rear_min_z={rear_floor_z:.5f}",
        )
    else:
        ctx.fail(
            "both frame feet rest on the ground plane",
            "Could not resolve front or rear foot world AABB.",
        )

    if left_rear_foot is not None:
        rest_rear_y = 0.5 * (left_rear_foot[0][1] + left_rear_foot[1][1])
        with ctx.pose({hinge: 0.38}):
            folded_left_rear_foot = ctx.part_element_world_aabb(rear_frame, elem="left_rear_foot")
            if folded_left_rear_foot is not None:
                folded_rear_y = 0.5 * (folded_left_rear_foot[0][1] + folded_left_rear_foot[1][1])
                ctx.check(
                    "rear support folds forward around the top hinge",
                    folded_rear_y > rest_rear_y + 0.22,
                    details=f"rest_y={rest_rear_y:.4f}, folded_y={folded_rear_y:.4f}",
                )
            else:
                ctx.fail(
                    "rear support folds forward around the top hinge",
                    "Could not resolve folded rear foot world AABB.",
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
