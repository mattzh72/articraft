from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="split_cheek_yaw_stage")

    base_length = 0.19
    base_width = 0.17
    base_thickness = 0.012

    cheek_length = 0.068
    cheek_thickness = 0.016
    cheek_height = 0.028
    cheek_y = 0.067

    pedestal_radius = 0.017
    pedestal_height = 0.006

    top_length = 0.10
    top_width = 0.05
    top_thickness = 0.010
    collar_radius = pedestal_radius
    collar_height = 0.010

    dark_anodized = model.material("dark_anodized", color=(0.20, 0.22, 0.25, 1.0))
    machined_aluminum = model.material("machined_aluminum", color=(0.73, 0.76, 0.80, 1.0))

    def rounded_box(length: float, width: float, height: float, radius: float) -> cq.Workplane:
        return (
            cq.Workplane("XY")
            .box(length, width, height, centered=(True, True, False))
            .edges("|Z")
            .fillet(radius)
        )

    base_plate_shape = rounded_box(base_length, base_width, base_thickness, 0.006)

    left_cheek_shape = (
        rounded_box(cheek_length, cheek_thickness, cheek_height, 0.0025)
        .translate((0.0, cheek_y, base_thickness))
    )
    right_cheek_shape = (
        rounded_box(cheek_length, cheek_thickness, cheek_height, 0.0025)
        .translate((0.0, -cheek_y, base_thickness))
    )

    hub_pedestal_shape = (
        cq.Workplane("XY")
        .circle(pedestal_radius)
        .extrude(pedestal_height)
        .edges(">Z")
        .fillet(0.0015)
        .translate((0.0, 0.0, base_thickness))
    )

    top_plate_shape = rounded_box(top_length, top_width, top_thickness, 0.003)
    hub_collar_shape = (
        cq.Workplane("XY")
        .circle(collar_radius)
        .extrude(collar_height)
        .edges(">Z")
        .fillet(0.0015)
    )

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(base_plate_shape, "base_plate"),
        name="base_plate",
        material=dark_anodized,
    )
    base.visual(
        mesh_from_cadquery(left_cheek_shape, "left_cheek"),
        name="left_cheek",
        material=dark_anodized,
    )
    base.visual(
        mesh_from_cadquery(right_cheek_shape, "right_cheek"),
        name="right_cheek",
        material=dark_anodized,
    )
    base.visual(
        mesh_from_cadquery(hub_pedestal_shape, "hub_pedestal"),
        name="hub_pedestal",
        material=machined_aluminum,
    )

    top = model.part("top")
    top.visual(
        mesh_from_cadquery(hub_collar_shape, "hub_collar"),
        name="hub_collar",
        material=machined_aluminum,
    )
    top.visual(
        mesh_from_cadquery(top_plate_shape.translate((0.0, 0.0, collar_height)), "top_plate"),
        name="top_plate",
        material=machined_aluminum,
    )

    model.articulation(
        "base_to_top",
        ArticulationType.REVOLUTE,
        parent=base,
        child=top,
        origin=Origin(xyz=(0.0, 0.0, base_thickness + pedestal_height)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=4.0,
            lower=-3.14159,
            upper=3.14159,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    base = object_model.get_part("base")
    top = object_model.get_part("top")
    yaw = object_model.get_articulation("base_to_top")

    ctx.check("base part exists", base is not None)
    ctx.check("top part exists", top is not None)
    ctx.check("yaw articulation exists", yaw is not None)

    limits = yaw.motion_limits
    ctx.check(
        "yaw axis is vertical",
        tuple(round(v, 6) for v in yaw.axis) == (0.0, 0.0, 1.0),
        details=f"axis={yaw.axis}",
    )
    ctx.check(
        "yaw limits permit broad rotation",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower <= -3.0
        and limits.upper >= 3.0,
        details=f"limits={limits}",
    )

    ctx.expect_contact(
        top,
        base,
        elem_a="hub_collar",
        elem_b="hub_pedestal",
        name="hub collar seats on pedestal",
    )
    ctx.expect_overlap(
        top,
        base,
        axes="xy",
        elem_a="hub_collar",
        elem_b="hub_pedestal",
        min_overlap=0.03,
        name="hub collar stays centered over pedestal",
    )

    def spans(aabb):
        return tuple(aabb[1][i] - aabb[0][i] for i in range(3))

    rest_plate_aabb = ctx.part_element_world_aabb(top, elem="top_plate")

    with ctx.pose({yaw: 1.0}):
        ctx.expect_contact(
            top,
            base,
            elem_a="hub_collar",
            elem_b="hub_pedestal",
            name="hub support remains engaged while yawed",
        )
        ctx.expect_gap(
            base,
            top,
            axis="y",
            positive_elem="left_cheek",
            negative_elem="top_plate",
            min_gap=0.001,
            name="left cheek clears turned top plate",
        )
        ctx.expect_gap(
            top,
            base,
            axis="y",
            positive_elem="top_plate",
            negative_elem="right_cheek",
            min_gap=0.001,
            name="right cheek clears turned top plate",
        )

        turned_plate_aabb = ctx.part_element_world_aabb(top, elem="top_plate")

    rest_spans = spans(rest_plate_aabb) if rest_plate_aabb is not None else None
    turned_spans = spans(turned_plate_aabb) if turned_plate_aabb is not None else None
    ctx.check(
        "top plate visibly rotates about hub",
        rest_spans is not None
        and turned_spans is not None
        and abs(rest_spans[1] - turned_spans[1]) > 0.03
        and turned_spans[1] > rest_spans[1] + 0.03,
        details=f"rest_spans={rest_spans}, turned_spans={turned_spans}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
