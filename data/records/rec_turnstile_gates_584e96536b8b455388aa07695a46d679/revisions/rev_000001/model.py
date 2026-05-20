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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_arm_turnstile_gate")

    powder_coat = model.material("powder_coat", rgba=(0.22, 0.24, 0.27, 1.0))
    stainless = model.material("stainless", rgba=(0.76, 0.78, 0.80, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((1.28, 0.76, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=powder_coat,
        name="base_plinth",
    )
    frame.visual(
        Box((0.20, 0.76, 0.12)),
        origin=Origin(xyz=(-0.54, 0.0, 0.12)),
        material=powder_coat,
        name="left_side_pedestal",
    )
    frame.visual(
        Box((0.20, 0.76, 0.12)),
        origin=Origin(xyz=(0.54, 0.0, 0.12)),
        material=powder_coat,
        name="right_side_pedestal",
    )
    frame.visual(
        Cylinder(radius=0.17, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=powder_coat,
        name="column_skirt",
    )
    frame.visual(
        Cylinder(radius=0.11, length=0.62),
        origin=Origin(xyz=(0.0, 0.0, 0.37)),
        material=stainless,
        name="central_column",
    )
    frame.visual(
        Cylinder(radius=0.13, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.65)),
        material=powder_coat,
        name="bearing_cap",
    )
    frame.visual(
        Box((0.12, 0.12, 0.90)),
        origin=Origin(xyz=(-0.60, 0.0, 0.51)),
        material=stainless,
        name="left_frame_post",
    )
    frame.visual(
        Box((0.12, 0.12, 0.90)),
        origin=Origin(xyz=(0.60, 0.0, 0.51)),
        material=stainless,
        name="right_frame_post",
    )
    frame.visual(
        Box((1.32, 0.12, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.99)),
        material=stainless,
        name="top_bridge",
    )
    frame.visual(
        Cylinder(radius=0.022, length=0.62),
        origin=Origin(xyz=(-0.60, 0.0, 0.52), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="left_side_rail",
    )
    frame.visual(
        Cylinder(radius=0.022, length=0.62),
        origin=Origin(xyz=(0.60, 0.0, 0.52), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="right_side_rail",
    )
    frame.inertial = Inertial.from_geometry(
        Box((1.32, 0.76, 0.98)),
        mass=68.0,
        origin=Origin(xyz=(0.0, 0.0, 0.49)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.12, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=powder_coat,
        name="hub_lower_collar",
    )
    rotor.visual(
        Cylinder(radius=0.09, length=0.15),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=stainless,
        name="hub_sleeve",
    )
    rotor.visual(
        Cylinder(radius=0.10, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.215)),
        material=powder_coat,
        name="hub_top_cap",
    )

    arm_angles = (
        math.pi / 2.0,
        math.pi / 2.0 + 2.0 * math.pi / 3.0,
        math.pi / 2.0 + 4.0 * math.pi / 3.0,
    )
    for index, angle in enumerate(arm_angles):
        c = math.cos(angle)
        s = math.sin(angle)
        rotor.visual(
            Cylinder(radius=0.034, length=0.10),
            origin=Origin(
                xyz=(0.14 * c, 0.14 * s, 0.115),
                rpy=(0.0, math.pi / 2.0, angle),
            ),
            material=powder_coat,
            name=f"arm_socket_{index}",
        )
        rotor.visual(
            Cylinder(radius=0.022, length=0.32),
            origin=Origin(
                xyz=(0.29 * c, 0.29 * s, 0.115),
                rpy=(0.0, math.pi / 2.0, angle),
            ),
            material=stainless,
            name=f"arm_{index}",
        )
        rotor.visual(
            Cylinder(radius=0.028, length=0.08),
            origin=Origin(
                xyz=(0.48 * c, 0.48 * s, 0.115),
                rpy=(0.0, math.pi / 2.0, angle),
            ),
            material=dark_rubber,
            name=f"arm_tip_{index}",
        )

    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.48, length=0.26),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, 0.13)),
    )

    model.articulation(
        "hub_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.68)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    rotor = object_model.get_part("rotor")
    hub_spin = object_model.get_articulation("hub_spin")

    ctx.expect_gap(
        rotor,
        frame,
        axis="z",
        positive_elem="hub_lower_collar",
        negative_elem="bearing_cap",
        min_gap=0.0,
        max_gap=0.001,
        name="rotor seats directly on the bearing cap",
    )

    rest_arm = ctx.part_element_world_aabb(rotor, elem="arm_0")
    rest_tip = ctx.part_element_world_aabb(rotor, elem="arm_tip_0")
    with ctx.pose({hub_spin: 2.0 * math.pi / 3.0}):
        turned_arm = ctx.part_element_world_aabb(rotor, elem="arm_0")
        turned_tip = ctx.part_element_world_aabb(rotor, elem="arm_tip_0")

    rest_arm_center = None
    turned_arm_center = None
    if rest_arm is not None:
        rest_arm_center = tuple((lo + hi) * 0.5 for lo, hi in zip(rest_arm[0], rest_arm[1]))
    if turned_arm is not None:
        turned_arm_center = tuple((lo + hi) * 0.5 for lo, hi in zip(turned_arm[0], turned_arm[1]))

    ctx.check(
        "arm hub rotates about the vertical axis",
        rest_arm_center is not None
        and turned_arm_center is not None
        and rest_arm_center[1] > 0.20
        and abs(rest_arm_center[0]) < 0.03
        and turned_arm_center[0] < -0.18
        and turned_arm_center[1] < -0.06,
        details=f"rest={rest_arm_center}, turned={turned_arm_center}",
    )

    def radial_extent(aabb):
        if aabb is None:
            return None
        return max(abs(value) for corner in aabb for value in corner[:2])

    ctx.check(
        "arm sweep stays inside the fixed frame envelope",
        rest_tip is not None
        and turned_tip is not None
        and radial_extent(rest_tip) is not None
        and radial_extent(turned_tip) is not None
        and radial_extent(rest_tip) < 0.56
        and radial_extent(turned_tip) < 0.56,
        details=f"rest_tip={rest_tip}, turned_tip={turned_tip}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
