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
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_arm_turnstile_gate")

    frame_gray = model.material("frame_gray", rgba=(0.48, 0.5, 0.53, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.76, 0.78, 0.8, 1.0))
    dark_cap = model.material("dark_cap", rgba=(0.14, 0.15, 0.16, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((1.12, 0.84, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=frame_gray,
        name="base_plinth",
    )
    frame.visual(
        Cylinder(radius=0.12, length=0.80),
        origin=Origin(xyz=(0.0, 0.0, 0.46)),
        material=frame_gray,
        name="center_column",
    )
    frame.visual(
        Cylinder(radius=0.155, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.92)),
        material=dark_cap,
        name="bearing_housing",
    )
    frame.visual(
        Cylinder(radius=0.18, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.99)),
        material=dark_cap,
        name="top_cap",
    )

    support_radius = 0.53
    support_angles = (math.pi / 3.0, math.pi, 5.0 * math.pi / 3.0)
    for index, angle in enumerate(support_angles):
        frame.visual(
            Cylinder(radius=0.03, length=0.86),
            origin=Origin(
                xyz=(support_radius * math.cos(angle), support_radius * math.sin(angle), 0.49)
            ),
            material=frame_gray,
            name=f"ring_support_{index}",
        )

    guard_ring_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.53, tube=0.018, radial_segments=18, tubular_segments=64),
        "turnstile_guard_ring",
    )
    frame.visual(
        guard_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.92)),
        material=brushed_steel,
        name="guard_ring",
    )
    frame.inertial = Inertial.from_geometry(
        Box((1.12, 0.84, 1.02)),
        mass=85.0,
        origin=Origin(xyz=(0.0, 0.0, 0.51)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.09, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=dark_cap,
        name="hub_shell",
    )
    rotor.visual(
        Cylinder(radius=0.06, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        material=brushed_steel,
        name="hub_capstan",
    )

    arm_radius = 0.022
    arm_length = 0.46
    arm_center_radius = arm_length * 0.5
    arm_height = 0.03
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        rotor.visual(
            Cylinder(radius=arm_radius, length=arm_length),
            origin=Origin(
                xyz=(
                    arm_center_radius * math.cos(angle),
                    arm_center_radius * math.sin(angle),
                    arm_height,
                ),
                rpy=(0.0, math.pi / 2.0, angle),
            ),
            material=brushed_steel,
            name=f"arm_{index}",
        )
        rotor.visual(
            Cylinder(radius=0.028, length=0.055),
            origin=Origin(
                xyz=(
                    (arm_length - 0.028) * math.cos(angle),
                    (arm_length - 0.028) * math.sin(angle),
                    arm_height,
                ),
                rpy=(0.0, math.pi / 2.0, angle),
            ),
            material=dark_cap,
            name=f"arm_end_{index}",
        )

    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.49, length=0.26),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, 0.13)),
    )

    model.articulation(
        "frame_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 1.00)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    rotor = object_model.get_part("rotor")
    spin = object_model.get_articulation("frame_to_rotor")

    ctx.expect_overlap(
        rotor,
        frame,
        axes="xy",
        elem_a="hub_shell",
        elem_b="bearing_housing",
        min_overlap=0.17,
        name="hub remains centered over the bearing housing",
    )
    ctx.expect_gap(
        rotor,
        frame,
        axis="z",
        positive_elem="arm_0",
        negative_elem="base_plinth",
        min_gap=0.88,
        max_gap=0.98,
        name="arms sit at waist height above the base plinth",
    )
    ctx.expect_within(
        rotor,
        frame,
        axes="xy",
        inner_elem="arm_0",
        outer_elem="guard_ring",
        margin=0.0,
        name="reference arm stays inside the fixed guard ring diameter",
    )
    ctx.expect_gap(
        rotor,
        frame,
        axis="z",
        positive_elem="arm_0",
        negative_elem="guard_ring",
        min_gap=0.01,
        max_gap=0.08,
        name="guard ring sits just below the arm sweep plane",
    )

    rest_aabb = ctx.part_element_world_aabb(rotor, elem="arm_0")
    with ctx.pose({spin: 2.0 * math.pi / 3.0}):
        ctx.expect_within(
            rotor,
            frame,
            axes="xy",
            inner_elem="arm_0",
            outer_elem="guard_ring",
            margin=0.0,
            name="rotated arm sweep remains inside the guard ring diameter",
        )
        rotated_aabb = ctx.part_element_world_aabb(rotor, elem="arm_0")

    def _center_xy(aabb):
        if aabb is None:
            return None
        (min_x, min_y, _), (max_x, max_y, _) = aabb
        return ((min_x + max_x) * 0.5, (min_y + max_y) * 0.5)

    rest_center = _center_xy(rest_aabb)
    rotated_center = _center_xy(rotated_aabb)
    ctx.check(
        "continuous joint rotates the arm hub around the center axis",
        rest_center is not None
        and rotated_center is not None
        and math.hypot(
            rotated_center[0] - rest_center[0],
            rotated_center[1] - rest_center[1],
        )
        > 0.35,
        details=f"rest_center={rest_center}, rotated_center={rotated_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
