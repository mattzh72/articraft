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

    plastic_black = model.material("plastic_black", rgba=(0.10, 0.10, 0.11, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.18, 0.19, 0.20, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    steel_shadow = model.material("steel_shadow", rgba=(0.56, 0.58, 0.61, 1.0))
    usb_blue = model.material("usb_blue", rgba=(0.16, 0.43, 0.82, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.032, 0.0168, 0.0072)),
        origin=Origin(xyz=(0.013, 0.0, 0.0)),
        material=plastic_black,
        name="body_shell",
    )
    body.visual(
        Box((0.027, 0.0128, 0.0022)),
        origin=Origin(xyz=(0.013, 0.0, 0.0025)),
        material=dark_gray,
        name="top_inlay",
    )
    body.visual(
        Cylinder(radius=0.00225, length=0.0076),
        origin=Origin(xyz=(0.0015, -0.0074, 0.0)),
        material=dark_gray,
        name="pivot_hub",
    )
    body.visual(
        Box((0.012, 0.0122, 0.00045)),
        origin=Origin(xyz=(0.035, 0.0, 0.00227)),
        material=brushed_steel,
        name="connector_top",
    )
    body.visual(
        Box((0.012, 0.0122, 0.00045)),
        origin=Origin(xyz=(0.035, 0.0, -0.00227)),
        material=brushed_steel,
        name="connector_bottom",
    )
    body.visual(
        Box((0.012, 0.00045, 0.0041)),
        origin=Origin(xyz=(0.035, 0.00588, 0.0)),
        material=steel_shadow,
        name="connector_side_right",
    )
    body.visual(
        Box((0.012, 0.00045, 0.0041)),
        origin=Origin(xyz=(0.035, -0.00588, 0.0)),
        material=steel_shadow,
        name="connector_side_left",
    )
    body.visual(
        Box((0.004, 0.0076, 0.00135)),
        origin=Origin(xyz=(0.029, 0.0, 0.0)),
        material=usb_blue,
        name="tongue_support",
    )
    body.visual(
        Box((0.008, 0.0076, 0.00135)),
        origin=Origin(xyz=(0.035, 0.0, 0.0)),
        material=usb_blue,
        name="connector_tongue",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.041, 0.0168, 0.0076)),
        mass=0.018,
        origin=Origin(xyz=(0.0165, 0.0, 0.0)),
    )

    cover = model.part("cover")
    cover.visual(
        Box((0.041, 0.0188, 0.0009)),
        origin=Origin(xyz=(0.0190, 0.0080, 0.00445)),
        material=brushed_steel,
        name="cover_top",
    )
    cover.visual(
        Box((0.041, 0.0188, 0.0009)),
        origin=Origin(xyz=(0.0190, 0.0080, -0.00445)),
        material=brushed_steel,
        name="cover_bottom",
    )
    cover.visual(
        Box((0.041, 0.0012, 0.0089)),
        origin=Origin(xyz=(0.0190, 0.0168, 0.0)),
        material=steel_shadow,
        name="cover_side_wall",
    )
    cover.visual(
        Box((0.0045, 0.0042, 0.0013)),
        origin=Origin(xyz=(0.0007, 0.0006, 0.00475)),
        material=steel_shadow,
        name="top_pivot_shoulder",
    )
    cover.visual(
        Box((0.0045, 0.0042, 0.0013)),
        origin=Origin(xyz=(0.0007, 0.0006, -0.00475)),
        material=steel_shadow,
        name="bottom_pivot_shoulder",
    )
    cover.visual(
        Cylinder(radius=0.0034, length=0.0018),
        origin=Origin(xyz=(0.0, 0.0, 0.0058)),
        material=brushed_steel,
        name="top_pivot_collar",
    )
    cover.visual(
        Cylinder(radius=0.0034, length=0.0018),
        origin=Origin(xyz=(0.0, 0.0, -0.0058)),
        material=brushed_steel,
        name="bottom_pivot_collar",
    )
    cover.visual(
        Cylinder(radius=0.00445, length=0.0188),
        origin=Origin(xyz=(0.0393, 0.0080, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="rounded_front_cap",
    )
    cover.inertial = Inertial.from_geometry(
        Box((0.041, 0.0188, 0.0106)),
        mass=0.012,
        origin=Origin(xyz=(0.0190, 0.0080, 0.0)),
    )

    model.articulation(
        "body_to_cover",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cover,
        origin=Origin(xyz=(0.0015, -0.0074, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    cover = object_model.get_part("cover")
    hinge = object_model.get_articulation("body_to_cover")

    ctx.expect_gap(
        cover,
        body,
        axis="z",
        positive_elem="cover_top",
        negative_elem="body_shell",
        min_gap=0.0002,
        max_gap=0.0012,
        name="top cover skin sits just above the body",
    )
    ctx.expect_gap(
        body,
        cover,
        axis="z",
        positive_elem="body_shell",
        negative_elem="cover_bottom",
        min_gap=0.0002,
        max_gap=0.0012,
        name="bottom cover skin sits just below the body",
    )
    ctx.expect_gap(
        cover,
        body,
        axis="y",
        positive_elem="cover_side_wall",
        negative_elem="body_shell",
        min_gap=0.0002,
        max_gap=0.0012,
        name="outer side wall clears the body flank",
    )
    ctx.expect_overlap(
        body,
        cover,
        axes="x",
        elem_a="body_shell",
        elem_b="cover_top",
        min_overlap=0.027,
        name="cover spans the body length in closed pose",
    )
    ctx.expect_contact(
        body,
        cover,
        elem_a="pivot_hub",
        elem_b="top_pivot_collar",
        contact_tol=0.0022,
        name="upper pivot hardware stays near the hub",
    )

    def _center_y(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> float | None:
        if aabb is None:
            return None
        return 0.5 * (aabb[0][1] + aabb[1][1])

    closed_aabb = ctx.part_world_aabb(cover)
    quarter_turn_aabb = None
    with ctx.pose({hinge: math.pi / 2.0}):
        quarter_turn_aabb = ctx.part_world_aabb(cover)
    with ctx.pose({hinge: math.pi}):
        open_aabb = ctx.part_world_aabb(cover)
        ctx.expect_gap(
            body,
            cover,
            axis="x",
            positive_elem="connector_top",
            negative_elem="cover_top",
            min_gap=0.002,
            name="connector clears the swung-away cover at 180 degrees",
        )

    def _center_x(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> float | None:
        if aabb is None:
            return None
        return 0.5 * (aabb[0][0] + aabb[1][0])

    quarter_turn_center_y = _center_y(quarter_turn_aabb)
    closed_center_x = _center_x(closed_aabb)
    open_center_x = _center_x(open_aabb)
    ctx.check(
        "cover rotates around the side pivot",
        quarter_turn_center_y is not None
        and closed_center_x is not None
        and open_center_x is not None
        and quarter_turn_center_y > 0.010
        and closed_center_x > 0.010
        and open_center_x < -0.010,
        details=(
            f"quarter_turn_center_y={quarter_turn_center_y}, "
            f"closed_center_x={closed_center_x}, open_center_x={open_center_x}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
