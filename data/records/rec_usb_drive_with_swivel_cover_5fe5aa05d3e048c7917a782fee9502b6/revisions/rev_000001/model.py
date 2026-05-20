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
    model = ArticulatedObject(name="usb_swivel_drive")

    dark_plastic = model.material("dark_plastic", rgba=(0.10, 0.10, 0.11, 1.0))
    mid_plastic = model.material("mid_plastic", rgba=(0.19, 0.20, 0.22, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.73, 0.75, 0.78, 1.0))
    connector_metal = model.material("connector_metal", rgba=(0.84, 0.85, 0.87, 1.0))

    body_width = 0.0162
    body_thickness = 0.0072
    core_width = 0.0156
    core_thickness = 0.0068
    body_front = 0.038
    body_rear = 0.006
    connector_length = 0.0135
    connector_width = 0.0124
    connector_thickness = 0.0046

    cover_thickness = 0.0009
    cover_side_clearance = 0.0007
    cover_top_clearance = 0.00035
    cover_front = 0.0565
    cover_rear = 0.0020
    cover_outer_width = body_width + (2.0 * cover_side_clearance) + (2.0 * cover_thickness)

    body = model.part("body")
    body.visual(
        Box((0.041, core_width, core_thickness)),
        origin=Origin(xyz=(0.0155, 0.0, 0.0)),
        material=dark_plastic,
        name="cartridge_core",
    )
    body.visual(
        Box((0.014, body_width, body_thickness)),
        origin=Origin(xyz=(0.001, 0.0, 0.0)),
        material=mid_plastic,
        name="pivot_block",
    )
    body.visual(
        Box((0.007, 0.0148, 0.0060)),
        origin=Origin(xyz=(0.0345, 0.0, 0.0)),
        material=mid_plastic,
        name="nose_shoulder",
    )
    body.visual(
        Box((connector_length, connector_width, connector_thickness)),
        origin=Origin(
            xyz=(
                body_front + (connector_length / 2.0),
                0.0,
                0.0,
            )
        ),
        material=connector_metal,
        name="usb_shell",
    )
    for side, sign in (("left", 1.0), ("right", -1.0)):
        body.visual(
            Cylinder(radius=0.0028, length=0.0004),
            origin=Origin(
                xyz=(0.0, sign * (body_width / 2.0 + 0.0002), 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=mid_plastic,
            name=f"{side}_pivot_boss",
        )
    body.inertial = Inertial.from_geometry(
        Box((body_front + body_rear + connector_length, body_width, body_thickness)),
        mass=0.018,
        origin=Origin(xyz=((body_front - body_rear + connector_length) / 2.0, 0.0, 0.0)),
    )

    cover = model.part("cover")
    cover.visual(
        Box((cover_front + cover_rear, cover_outer_width, cover_thickness)),
        origin=Origin(
            xyz=((cover_front - cover_rear) / 2.0, 0.0, body_thickness / 2.0 + cover_top_clearance + cover_thickness / 2.0)
        ),
        material=brushed_metal,
        name="top_plate",
    )
    side_arm_height = body_thickness + cover_top_clearance + 0.0011 + cover_thickness
    side_arm_z = -0.0001 + cover_top_clearance / 2.0
    side_arm_x = (cover_front - cover_rear - 0.0012) / 2.0
    side_arm_length = cover_front + cover_rear - 0.0012
    for side, sign in (("left", 1.0), ("right", -1.0)):
        cover.visual(
            Box((side_arm_length, cover_thickness, side_arm_height)),
            origin=Origin(
                xyz=(
                    side_arm_x,
                    sign * (body_width / 2.0 + cover_side_clearance + cover_thickness / 2.0),
                    side_arm_z,
                )
            ),
            material=brushed_metal,
            name=f"{side}_arm",
        )
        cover.visual(
            Cylinder(radius=0.0034, length=0.00035),
            origin=Origin(
                xyz=(0.0, sign * (body_width / 2.0 + 0.000575), 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=brushed_metal,
            name=f"{side}_pivot_collar",
        )
    cover.visual(
        Box((0.0014, cover_outer_width, body_thickness + cover_top_clearance + 0.0012)),
        origin=Origin(xyz=(cover_front - 0.0007, 0.0, -0.00005)),
        material=brushed_metal,
        name="front_bridge",
    )
    cover.inertial = Inertial.from_geometry(
        Box((cover_front + cover_rear, cover_outer_width, side_arm_height)),
        mass=0.008,
        origin=Origin(xyz=((cover_front - cover_rear) / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_cover",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cover,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    cover = object_model.get_part("cover")
    pivot = object_model.get_articulation("body_to_cover")

    with ctx.pose({pivot: 0.0}):
        ctx.expect_gap(
            cover,
            body,
            axis="z",
            positive_elem="top_plate",
            negative_elem="pivot_block",
            max_gap=0.0010,
            max_penetration=0.0,
            name="closed cover rides just above the body",
        )
        ctx.expect_gap(
            cover,
            body,
            axis="x",
            positive_elem="front_bridge",
            negative_elem="usb_shell",
            min_gap=0.002,
            max_gap=0.008,
            name="closed front bridge protects the connector tip",
        )
        ctx.expect_overlap(
            cover,
            body,
            axes="x",
            elem_a="top_plate",
            elem_b="cartridge_core",
            min_overlap=0.035,
            name="closed cover spans the cartridge length",
        )

    with ctx.pose({pivot: math.pi}):
        ctx.expect_gap(
            body,
            cover,
            axis="x",
            positive_elem="usb_shell",
            min_gap=0.025,
            name="rotated cover leaves the connector exposed",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
