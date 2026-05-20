from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


AXLE_Z = 1.25
FRAME_X = 0.98
WHEEL_RADIUS = 0.82
WHEEL_WIDTH = 1.15


def _radial_center(radius: float, theta: float) -> tuple[float, float, float]:
    """Point in the wheel's local YZ plane for a member whose local +Z is radial."""

    return (0.0, -radius * math.sin(theta), radius * math.cos(theta))


def _circle_profile(radius: float, segments: int = 48) -> list[tuple[float, float]]:
    return [
        (radius * math.cos(2.0 * math.pi * i / segments), radius * math.sin(2.0 * math.pi * i / segments))
        for i in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_service_undershot_waterwheel")

    orange = model.material("chipped_safety_orange", rgba=(0.92, 0.36, 0.10, 1.0))
    steel = model.material("dark_galvanized_steel", rgba=(0.18, 0.19, 0.18, 1.0))
    bronze = model.material("oiled_bronze_bearing", rgba=(0.58, 0.43, 0.18, 1.0))
    timber = model.material("worn_treated_timber", rgba=(0.46, 0.29, 0.13, 1.0))
    wear = model.material("replaceable_bright_wear_steel", rgba=(0.74, 0.72, 0.66, 1.0))
    deck = model.material("checked_service_deck", rgba=(0.28, 0.30, 0.28, 1.0))

    frame = model.part("frame")
    wheel = model.part("wheel")

    # Connected skid base and cross ties: chunky enough to read as a field-service
    # unit that can be dragged into a raceway and shimmed in place.
    for idx, x in enumerate((-1.08, 1.08)):
        frame.visual(
            Box((0.20, 2.80, 0.14)),
            origin=Origin(xyz=(x, 0.0, 0.07)),
            material=steel,
            name=f"skid_{idx}",
        )

    for name, y in (("front_cross_tie", -1.28), ("rear_cross_tie", 1.28)):
        frame.visual(
            Box((2.40, 0.20, 0.14)),
            origin=Origin(xyz=(0.0, y, 0.17)),
            material=steel,
            name=name,
        )

    frame.visual(
        Box((1.52, 1.08, 0.055)),
        origin=Origin(xyz=(0.0, -0.04, 0.255)),
        material=deck,
        name="replaceable_sill_plate",
    )
    for x in (-0.60, 0.0, 0.60):
        frame.visual(
            Box((0.10, 2.58, 0.12)),
            origin=Origin(xyz=(x, -0.04, 0.20)),
            material=steel,
            name=f"sill_rib_{x:+.1f}",
        )

    # Two outboard side frames with upright posts, crossed braces, and bearing
    # blocks. They sit outside the wheel envelope and carry the axle at center.
    side_specs = ((0, -FRAME_X), (1, FRAME_X))
    bearing_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(0.14, 56),
            [_circle_profile(0.088, 40)],
            0.18,
            cap=True,
            center=True,
        ).rotate_y(math.pi / 2.0),
        "split_bearing_bushing",
    )
    for idx, x in side_specs:
        for y in (-0.62, 0.62):
            frame.visual(
                Box((0.14, 0.14, 1.36)),
                origin=Origin(xyz=(x, y, 0.76)),
                material=orange,
                name=f"upright_{idx}_{'front' if y < 0 else 'rear'}",
            )

        frame.visual(
            Box((0.16, 1.46, 0.12)),
            origin=Origin(xyz=(x, 0.0, 1.47)),
            material=orange,
            name=f"top_rail_{idx}",
        )
        frame.visual(
            Box((0.34, 0.56, 0.20)),
            origin=Origin(xyz=(x, 0.0, 1.04)),
            material=orange,
            name=f"bearing_pedestal_{idx}",
        )
        frame.visual(
            bearing_mesh,
            origin=Origin(xyz=(x, 0.0, AXLE_Z)),
            material=bronze,
            name=f"bearing_shell_{idx}",
        )
        frame.visual(
            Box((0.31, 0.42, 0.085)),
            origin=Origin(xyz=(x, 0.0, 1.405)),
            material=orange,
            name=f"bearing_cap_{idx}",
        )

        for base_y, top_y in ((-0.92, -0.22), (0.92, 0.22)):
            dy = top_y - base_y
            dz = 1.12 - 0.20
            length = math.hypot(dy, dz)
            angle = math.atan2(dz, dy)
            frame.visual(
                Box((0.12, length, 0.12)),
                origin=Origin(xyz=(x, (base_y + top_y) / 2.0, 0.66), rpy=(angle, 0.0, 0.0)),
                material=orange,
                name=f"diagonal_{idx}_{'front' if base_y < 0 else 'rear'}",
            )

        outer_face = x + (0.17 if x > 0 else -0.17)
        for b, (y, z) in enumerate(((-0.15, 1.405), (0.15, 1.405), (-0.15, 1.12), (0.15, 1.12))):
            frame.visual(
                Cylinder(radius=0.025, length=0.032),
                origin=Origin(xyz=(outer_face, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=steel,
                name=f"cap_bolt_{idx}_{b}",
            )

    # Maintenance access: a bolted service deck and guard rails tied into the
    # front uprights. The open rear/front gap keeps the paddle wheel accessible.
    frame.visual(
        Box((2.42, 0.38, 0.08)),
        origin=Origin(xyz=(0.0, -1.36, 0.48)),
        material=deck,
        name="service_deck",
    )
    for idx, x in side_specs:
        frame.visual(
            Box((0.12, 0.78, 0.09)),
            origin=Origin(xyz=(x, -1.03, 0.44)),
            material=steel,
            name=f"deck_cantilever_{idx}",
        )
    for x in (-1.08, -0.36, 0.36, 1.08):
        frame.visual(
            Box((0.075, 0.075, 0.62)),
            origin=Origin(xyz=(x, -1.52, 0.82)),
            material=orange,
            name=f"rail_post_{x:+.2f}",
        )
    for z, name in ((1.15, "handrail"), (0.87, "midrail")):
        frame.visual(
            Box((2.34, 0.075, 0.075)),
            origin=Origin(xyz=(0.0, -1.52, z)),
            material=orange,
            name=name,
        )

    # Rotating waterwheel.  Two steel rims, stout radial spokes, a through axle,
    # timber paddles, and bolted wear strips make the serviceable construction
    # explicit.  The child part frame is exactly the axle centerline.
    rim_mesh = mesh_from_geometry(
        TorusGeometry(WHEEL_RADIUS, 0.035, radial_segments=18, tubular_segments=80).rotate_y(
            math.pi / 2.0
        ),
        "waterwheel_side_rim",
    )
    for idx, x in enumerate((-WHEEL_WIDTH / 2.0 + 0.035, WHEEL_WIDTH / 2.0 - 0.035)):
        wheel.visual(
            rim_mesh,
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=steel,
            name=f"side_rim_{idx}",
        )

    wheel.visual(
        Cylinder(radius=0.070, length=2.34),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="axle_shaft",
    )
    wheel.visual(
        Cylinder(radius=0.155, length=0.88),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hub_sleeve",
    )
    for idx, x in enumerate((-0.49, 0.49)):
        wheel.visual(
            Cylinder(radius=0.205, length=0.16),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"hub_flange_{idx}",
        )
        wheel.visual(
            Cylinder(radius=0.112, length=0.085),
            origin=Origin(xyz=(x * 1.55, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=wear,
            name=f"stop_collar_{idx}",
        )

    for side, x in enumerate((-0.49, 0.49)):
        for i in range(8):
            theta = 2.0 * math.pi * i / 8.0
            _, y, z = _radial_center(0.505, theta)
            wheel.visual(
                Box((0.10, 0.075, 0.66)),
                origin=Origin(xyz=(x, y, z), rpy=(theta, 0.0, 0.0)),
                material=steel,
                name=f"spoke_{side}_{i}",
            )

    for i in range(12):
        theta = 2.0 * math.pi * i / 12.0
        _, y, z = _radial_center(0.785, theta)
        wheel.visual(
            Box((WHEEL_WIDTH, 0.090, 0.255)),
            origin=Origin(xyz=(0.0, y, z), rpy=(theta, 0.0, 0.0)),
            material=timber,
            name=f"paddle_{i}",
        )
        _, wy, wz = _radial_center(0.925, theta)
        wheel.visual(
            Box((WHEEL_WIDTH + 0.06, 0.105, 0.055)),
            origin=Origin(xyz=(0.0, wy, wz), rpy=(theta, 0.0, 0.0)),
            material=wear,
            name=f"wear_strip_{i}",
        )
        # Through-bolts visually explain why the replaceable strip stays with
        # the timber blade; they also tie back to the heavy-duty service theme.
        for bx in (-0.42, 0.42):
            wheel.visual(
                Cylinder(radius=0.020, length=0.040),
                origin=Origin(xyz=(bx, wy, wz), rpy=(theta, math.pi / 2.0, 0.0)),
                material=steel,
                name=f"wear_bolt_{i}_{'a' if bx < 0 else 'b'}",
            )

    model.articulation(
        "frame_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, AXLE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1800.0, velocity=2.2),
        motion_properties=MotionProperties(damping=0.08, friction=0.04),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    wheel = object_model.get_part("wheel")
    joint = object_model.get_articulation("frame_to_wheel")

    ctx.allow_overlap(
        frame,
        wheel,
        elem_a="bearing_shell_0",
        elem_b="axle_shaft",
        reason="The axle is intentionally captured through the left replaceable bushing; mesh collision treats the ring bore conservatively.",
    )
    ctx.allow_overlap(
        frame,
        wheel,
        elem_a="bearing_shell_1",
        elem_b="axle_shaft",
        reason="The axle is intentionally captured through the right replaceable bushing; mesh collision treats the ring bore conservatively.",
    )

    ctx.check(
        "wheel uses centered continuous axle joint",
        joint.articulation_type == ArticulationType.CONTINUOUS and tuple(joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={joint.articulation_type}, axis={joint.axis}",
    )

    ctx.expect_within(
        wheel,
        frame,
        axes="yz",
        inner_elem="axle_shaft",
        outer_elem="bearing_shell_0",
        margin=0.002,
        name="axle centered in bearing 0",
    )
    ctx.expect_overlap(
        wheel,
        frame,
        axes="x",
        elem_a="axle_shaft",
        elem_b="bearing_shell_0",
        min_overlap=0.14,
        name="axle retained by bearing 0",
    )
    ctx.expect_within(
        wheel,
        frame,
        axes="yz",
        inner_elem="axle_shaft",
        outer_elem="bearing_shell_1",
        margin=0.002,
        name="axle centered in bearing 1",
    )
    ctx.expect_overlap(
        wheel,
        frame,
        axes="x",
        elem_a="axle_shaft",
        elem_b="bearing_shell_1",
        min_overlap=0.14,
        name="axle retained by bearing 1",
    )

    rest_pos = ctx.part_world_position(wheel)
    with ctx.pose({joint: math.pi / 2.0}):
        turned_pos = ctx.part_world_position(wheel)
        ctx.expect_within(
            wheel,
            frame,
            axes="yz",
            inner_elem="axle_shaft",
            outer_elem="bearing_shell_0",
            margin=0.002,
            name="rotated axle remains in bearing",
        )
    ctx.check(
        "rotation keeps wheel on fixed centerline",
        rest_pos is not None
        and turned_pos is not None
        and all(abs(rest_pos[i] - turned_pos[i]) < 1e-6 for i in range(3)),
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


object_model = build_object_model()
