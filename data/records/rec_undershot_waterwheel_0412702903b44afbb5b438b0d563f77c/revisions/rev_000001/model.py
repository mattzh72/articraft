from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


WHEEL_CENTER_Z = 0.98
WHEEL_RADIUS = 0.72
PADDLE_COUNT = 16
PADDLE_CENTER_RADIUS = 0.72
PADDLE_HEIGHT = 0.23
PADDLE_WIDTH = 0.82
AXLE_LENGTH = 1.30
AXLE_RADIUS = 0.045


def _axis_to_rpy(axis: tuple[float, float, float]) -> tuple[float, float, float]:
    """Return an Origin.rpy that aligns a cylinder's local +Z to axis."""

    x, y, z = axis
    length = math.sqrt(x * x + y * y + z * z)
    if length <= 1e-9:
        return (0.0, 0.0, 0.0)
    x, y, z = x / length, y / length, z / length
    pitch = math.atan2(math.sqrt(x * x + y * y), z)
    yaw = math.atan2(y, x)
    return (0.0, pitch, yaw)


def _add_cylinder_between(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str,
) -> None:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0, (start[2] + end[2]) / 2.0),
            rpy=_axis_to_rpy((dx, dy, dz)),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_undershot_waterwheel")

    satin_oak = model.material("satin_oiled_oak", rgba=(0.55, 0.35, 0.18, 1.0))
    wet_oak = model.material("matte_wet_oak", rgba=(0.35, 0.22, 0.12, 1.0))
    dark_steel = model.material("matte_graphite_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    bronze = model.material("satin_bronze", rgba=(0.72, 0.50, 0.25, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.70, 0.72, 0.73, 1.0))
    slate = model.material("honed_slate", rgba=(0.19, 0.21, 0.22, 1.0))
    water = model.material("low_clear_water", rgba=(0.18, 0.42, 0.62, 0.48))
    seam_black = model.material("shadow_seam_black", rgba=(0.025, 0.027, 0.030, 1.0))

    frame = model.part("frame")
    # Refined shallow flume and footings: low water passes just below the lower paddles.
    frame.visual(Box((1.36, 1.40, 0.08)), origin=Origin(xyz=(0.0, 0.0, 0.04)), material=slate, name="stone_sill")
    frame.visual(Box((1.28, 1.16, 0.035)), origin=Origin(xyz=(0.0, 0.0, 0.115)), material=water, name="water_run")
    frame.visual(Box((1.34, 0.10, 0.18)), origin=Origin(xyz=(0.0, -0.66, 0.13)), material=slate, name="front_flume_lip")
    frame.visual(Box((1.34, 0.10, 0.18)), origin=Origin(xyz=(0.0, 0.66, 0.13)), material=slate, name="rear_flume_lip")
    for y, name in ((-0.36, "flow_mark_0"), (0.0, "flow_mark_1"), (0.36, "flow_mark_2")):
        frame.visual(Box((1.10, 0.012, 0.012)), origin=Origin(xyz=(0.0, y, 0.139)), material=water, name=name)

    # Two side frames tied through the sill; rods meet saddles that cradle the bearing collars.
    for x, side in ((-0.56, "side_0"), (0.56, "side_1")):
        bearing_ring_name = "side_0_bearing_ring" if side == "side_0" else "side_1_bearing_ring"
        frame.visual(Box((0.09, 1.20, 0.085)), origin=Origin(xyz=(x, 0.0, 0.165)), material=dark_steel, name=f"{side}_base_rail")
        frame.visual(Box((0.10, 0.22, 0.07)), origin=Origin(xyz=(x, 0.0, WHEEL_CENTER_Z - 0.145)), material=dark_steel, name=f"{side}_bearing_saddle")
        frame.visual(Box((0.095, 0.030, 0.34)), origin=Origin(xyz=(x, -0.135, WHEEL_CENTER_Z - 0.225)), material=dark_steel, name=f"{side}_front_cheek")
        frame.visual(Box((0.095, 0.030, 0.34)), origin=Origin(xyz=(x, 0.135, WHEEL_CENTER_Z - 0.225)), material=dark_steel, name=f"{side}_rear_cheek")
        _add_cylinder_between(
            frame,
            (x, -0.52, 0.205),
            (x, -0.082, WHEEL_CENTER_Z - 0.090),
            radius=0.026,
            material=dark_steel,
            name=f"{side}_front_strut",
        )
        _add_cylinder_between(
            frame,
            (x, 0.52, 0.205),
            (x, 0.082, WHEEL_CENTER_Z - 0.090),
            radius=0.026,
            material=dark_steel,
            name=f"{side}_rear_strut",
        )
        _add_cylinder_between(
            frame,
            (x, -0.48, 0.245),
            (x, 0.48, 0.245),
            radius=0.018,
            material=dark_steel,
            name=f"{side}_low_tie",
        )
        frame.visual(
            mesh_from_geometry(TorusGeometry(0.087, 0.023, radial_segments=28, tubular_segments=44), bearing_ring_name),
            origin=Origin(xyz=(x, 0.0, WHEEL_CENTER_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bronze,
            name=bearing_ring_name,
        )
        frame.visual(
            mesh_from_geometry(TorusGeometry(0.081, 0.012, radial_segments=24, tubular_segments=40), f"{side}_bearing_face"),
            origin=Origin(xyz=(x + (0.020 if x > 0 else -0.020), 0.0, WHEEL_CENTER_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bronze,
            name=f"{side}_bearing_face",
        )
        for y in (-0.080, 0.080):
            for z in (WHEEL_CENTER_Z - 0.070, WHEEL_CENTER_Z + 0.070):
                frame.visual(
                    Cylinder(radius=0.011, length=0.010),
                    origin=Origin(
                        xyz=(x + (0.017 if x > 0 else -0.017), y, z),
                        rpy=(0.0, math.pi / 2.0, 0.0),
                    ),
                    material=brushed_steel,
                    name=f"{side}_bolt_{int((y + 0.09) * 1000)}_{int((z - WHEEL_CENTER_Z + 0.08) * 1000)}",
                )

    for x in (-0.56, 0.56):
        for y in (-0.45, 0.45):
            frame.visual(
                Box((0.090, 0.160, 0.043)),
                origin=Origin(xyz=(x, y, 0.1015)),
                material=dark_steel,
                name=f"foot_{int((x + 0.7) * 100)}_{int((y + 0.6) * 100)}",
            )

    for y, name in ((-0.54, "front_cross_tie"), (0.54, "rear_cross_tie")):
        _add_cylinder_between(
            frame,
            (-0.61, y, 0.255),
            (0.61, y, 0.255),
            radius=0.020,
            material=dark_steel,
            name=name,
        )

    wheel = model.part("wheel")
    wheel.visual(
        mesh_from_geometry(
            WheelGeometry(
                WHEEL_RADIUS,
                0.62,
                rim=WheelRim(inner_radius=0.60, flange_height=0.030, flange_thickness=0.014, bead_seat_depth=0.010),
                hub=WheelHub(
                    radius=0.120,
                    width=0.240,
                    cap_style="domed",
                    bolt_pattern=BoltPattern(count=8, circle_diameter=0.155, hole_diameter=0.012),
                ),
                face=WheelFace(dish_depth=0.030, front_inset=0.012, rear_inset=0.012),
                spokes=WheelSpokes(style="straight", count=12, thickness=0.022, window_radius=0.035),
                bore=WheelBore(style="round", diameter=AXLE_RADIUS * 1.8),
            ),
            "waterwheel_rim",
        ),
        material=satin_oak,
        name="rim_spokes",
    )
    wheel.visual(
        Cylinder(radius=AXLE_RADIUS, length=AXLE_LENGTH),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="axle_spindle",
    )
    wheel.visual(
        Box((0.260, 0.036, 0.036)),
        origin=Origin(xyz=(0.0, 0.056, 0.0)),
        material=brushed_steel,
        name="shaft_key",
    )
    for index in range(8):
        theta = (2.0 * math.pi * index) / 8.0
        _add_cylinder_between(
            wheel,
            (0.0, 0.105 * math.sin(theta), 0.105 * math.cos(theta)),
            (0.0, 0.650 * math.sin(theta), 0.650 * math.cos(theta)),
            radius=0.018,
            material=satin_oak,
            name=f"drive_spoke_{index}",
        )
    for index in range(PADDLE_COUNT):
        paddle_name = "paddle_0" if index == 0 else f"paddle_{index}"
        phi = math.pi + (2.0 * math.pi * index / PADDLE_COUNT)
        y = PADDLE_CENTER_RADIUS * math.sin(phi)
        z = PADDLE_CENTER_RADIUS * math.cos(phi)
        wheel.visual(
            Box((PADDLE_WIDTH, 0.050, PADDLE_HEIGHT)),
            origin=Origin(xyz=(0.0, y, z), rpy=(-phi, 0.0, 0.0)),
            material=wet_oak,
            name=paddle_name,
        )
        # Thin dark seam plates leave crisp interface breaks between boards and rim.
        for sx, seam_name in ((-(PADDLE_WIDTH / 2.0 + 0.007), "seam_a"), (PADDLE_WIDTH / 2.0 + 0.007, "seam_b")):
            wheel.visual(
                Box((0.014, 0.056, PADDLE_HEIGHT * 0.86)),
                origin=Origin(xyz=(sx, y, z), rpy=(-phi, 0.0, 0.0)),
                material=seam_black,
                name=f"paddle_{index}_{seam_name}",
            )

    wheel.inertial = Inertial.from_geometry(Box((AXLE_LENGTH, WHEEL_RADIUS * 2.0, WHEEL_RADIUS * 2.0)), mass=185.0)

    model.articulation(
        "frame_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, WHEEL_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=420.0, velocity=1.2),
        motion_properties=MotionProperties(damping=0.08, friction=0.02),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    wheel = object_model.get_part("wheel")
    joint = object_model.get_articulation("frame_to_wheel")

    ctx.check("waterwheel_has_rotary_stage", joint is not None and wheel is not None and frame is not None)
    if frame is None or wheel is None or joint is None:
        return ctx.report()

    ctx.check("wheel_axis_is_crosswise", tuple(joint.axis) == (1.0, 0.0, 0.0), details=f"axis={joint.axis!r}")
    ctx.expect_overlap(wheel, frame, axes="x", elem_a="axle_spindle", elem_b="side_0_bearing_ring", min_overlap=0.020)
    ctx.expect_overlap(wheel, frame, axes="x", elem_a="axle_spindle", elem_b="side_1_bearing_ring", min_overlap=0.020)
    ctx.expect_gap(
        wheel,
        frame,
        axis="z",
        positive_elem="paddle_0",
        negative_elem="water_run",
        min_gap=0.010,
        max_gap=0.070,
        name="lower paddle rides just above shallow water",
    )

    closed_aabb = ctx.part_element_world_aabb(wheel, elem="paddle_0")
    with ctx.pose({joint: math.pi / 2.0}):
        rotated_aabb = ctx.part_element_world_aabb(wheel, elem="paddle_0")
    if closed_aabb is not None and rotated_aabb is not None:
        closed_center_y = (closed_aabb[0][1] + closed_aabb[1][1]) / 2.0
        rotated_center_y = (rotated_aabb[0][1] + rotated_aabb[1][1]) / 2.0
        ctx.check(
            "paddle_advances_with_rotation",
            rotated_center_y > closed_center_y + 0.45,
            details=f"closed_y={closed_center_y:.3f}, rotated_y={rotated_center_y:.3f}",
        )
    else:
        ctx.fail("paddle_pose_aabb_available", "Expected paddle_0 AABBs at rest and at a rotated pose.")

    return ctx.report()


object_model = build_object_model()
