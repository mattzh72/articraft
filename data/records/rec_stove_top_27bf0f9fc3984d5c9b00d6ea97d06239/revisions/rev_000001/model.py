from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


COUNTERTOP_LENGTH = 1.45
COUNTERTOP_DEPTH = 0.85
COUNTERTOP_THICKNESS = 0.060
COOKTOP_LENGTH = 0.92
COOKTOP_DEPTH = 0.62
GLASS_LENGTH = 0.925
GLASS_DEPTH = 0.625
GLASS_THICKNESS = 0.007
TOP_Z = COUNTERTOP_THICKNESS
POD_CENTER = (0.315, -0.235)
POD_SIZE = (0.175, 0.120, 0.006)
BUTTON_SIZE = 0.026
BUTTON_HEIGHT = 0.006
BUTTON_TRAVEL = 0.004
BUTTON_SPACING = 0.044


def _countertop_shape():
    slab = cq.Workplane("XY").box(
        COUNTERTOP_LENGTH, COUNTERTOP_DEPTH, COUNTERTOP_THICKNESS
    )
    slab = slab.translate((0.0, 0.0, COUNTERTOP_THICKNESS / 2.0))
    pocket = cq.Workplane("XY").box(COOKTOP_LENGTH, COOKTOP_DEPTH, 0.012)
    pocket = pocket.translate((0.0, 0.0, TOP_Z - 0.006))
    return slab.cut(pocket)


def _glass_shape():
    glass = cq.Workplane("XY").box(GLASS_LENGTH, GLASS_DEPTH, GLASS_THICKNESS)
    glass = glass.translate((0.0, 0.0, TOP_Z - GLASS_THICKNESS / 2.0))
    return glass.edges("|Z").fillet(0.025)


def _ring_shape(radius: float, line_width: float = 0.004, height: float = 0.0005):
    return (
        cq.Workplane("XY")
        .circle(radius)
        .circle(radius - line_width)
        .extrude(height)
    )


def _pod_shape():
    pod_w, pod_d, pod_h = POD_SIZE
    pod = cq.Workplane("XY").box(pod_w, pod_d, pod_h)
    pod = pod.translate((0.0, 0.0, pod_h / 2.0))
    pod = pod.edges("|Z").fillet(0.010)

    # The moving cap is guided by a close square opening, so the plunger reads
    # as captured by the pod face while still sliding straight down.
    hole = BUTTON_SIZE
    for x in (-BUTTON_SPACING / 2.0, BUTTON_SPACING / 2.0):
        for y in (-BUTTON_SPACING / 2.0, BUTTON_SPACING / 2.0):
            cutter = cq.Workplane("XY").box(hole, hole, pod_h * 3.0)
            cutter = cutter.translate((x, y, pod_h / 2.0))
            pod = pod.cut(cutter)
    return pod


def _button_shape():
    cap = cq.Workplane("XY").box(BUTTON_SIZE, BUTTON_SIZE, BUTTON_HEIGHT)
    cap = cap.translate((0.0, 0.0, BUTTON_HEIGHT / 2.0))
    return cap.edges("|Z").fillet(0.003)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flush_induction_island_cooktop")

    stone = model.material("warm_stone", rgba=(0.62, 0.58, 0.51, 1.0))
    glass = model.material("black_glass", rgba=(0.015, 0.017, 0.020, 0.92))
    zone_mark = model.material("soft_gray_print", rgba=(0.68, 0.70, 0.72, 1.0))
    pod_mat = model.material("satin_graphite", rgba=(0.055, 0.057, 0.062, 1.0))
    button_mat = model.material("button_black", rgba=(0.015, 0.015, 0.017, 1.0))

    island_top = model.part("island_top")
    island_top.visual(
        mesh_from_cadquery(_countertop_shape(), "countertop_slab"),
        material=stone,
        name="countertop_slab",
    )
    island_top.visual(
        mesh_from_cadquery(_glass_shape(), "glass_panel"),
        material=glass,
        name="glass_panel",
    )

    zones = (
        ("zone_0_0", -0.245, 0.145, 0.115),
        ("zone_0_1", 0.155, 0.145, 0.105),
        ("zone_1_0", -0.245, -0.145, 0.105),
        ("zone_1_1", 0.120, -0.145, 0.095),
    )
    for zone_name, x, y, radius in zones:
        island_top.visual(
            mesh_from_cadquery(_ring_shape(radius), zone_name),
            # Printed zone graphics are modeled as hairline inlays just into the
            # glass surface so they read flush and remain part of the fixed top.
            origin=Origin(xyz=(x, y, TOP_Z - 0.00025)),
            material=zone_mark,
            name=zone_name,
        )

    island_top.visual(
        mesh_from_cadquery(_pod_shape(), "control_pod"),
        origin=Origin(xyz=(POD_CENTER[0], POD_CENTER[1], TOP_Z)),
        material=pod_mat,
        name="control_pod",
    )

    for row, y_offset in enumerate((BUTTON_SPACING / 2.0, -BUTTON_SPACING / 2.0)):
        for col, x_offset in enumerate((-BUTTON_SPACING / 2.0, BUTTON_SPACING / 2.0)):
            button_name = f"button_{row}_{col}"
            button = model.part(button_name)
            button.visual(
                mesh_from_cadquery(_button_shape(), button_name),
                material=button_mat,
                name="cap",
            )
            model.articulation(
                f"{button_name}_slide",
                ArticulationType.PRISMATIC,
                parent=island_top,
                child=button,
                origin=Origin(
                    xyz=(
                        POD_CENTER[0] + x_offset,
                        POD_CENTER[1] + y_offset,
                        TOP_Z + POD_SIZE[2],
                    )
                ),
                axis=(0.0, 0.0, -1.0),
                motion_limits=MotionLimits(
                    effort=8.0,
                    velocity=0.12,
                    lower=0.0,
                    upper=BUTTON_TRAVEL,
                ),
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    island_top = object_model.get_part("island_top")

    button_joints = [
        object_model.get_articulation(f"button_{row}_{col}_slide")
        for row in range(2)
        for col in range(2)
    ]
    ctx.check(
        "exactly four movable buttons",
        len(button_joints) == 4
        and all(j.articulation_type == ArticulationType.PRISMATIC for j in button_joints),
        details=f"joints={[j.name for j in button_joints]}",
    )
    ctx.check(
        "button plungers move normal to panel",
        all(tuple(j.axis) == (0.0, 0.0, -1.0) for j in button_joints),
        details=f"axes={[j.axis for j in button_joints]}",
    )

    for row in range(2):
        for col in range(2):
            button = object_model.get_part(f"button_{row}_{col}")
            ctx.expect_gap(
                button,
                island_top,
                axis="z",
                max_gap=0.001,
                max_penetration=0.0,
                elem_a="cap",
                elem_b="control_pod",
                name=f"button_{row}_{col} rests on pod plane",
            )
            ctx.expect_overlap(
                button,
                island_top,
                axes="xy",
                min_overlap=0.020,
                elem_a="cap",
                elem_b="control_pod",
                name=f"button_{row}_{col} lies in control pod footprint",
            )

    first_button = object_model.get_part("button_0_0")
    first_joint = object_model.get_articulation("button_0_0_slide")
    rest_pos = ctx.part_world_position(first_button)
    with ctx.pose({first_joint: BUTTON_TRAVEL}):
        pressed_pos = ctx.part_world_position(first_button)
    ctx.check(
        "button press travels downward",
        rest_pos is not None
        and pressed_pos is not None
        and pressed_pos[2] < rest_pos[2] - BUTTON_TRAVEL * 0.8,
        details=f"rest={rest_pos}, pressed={pressed_pos}",
    )

    ctx.expect_within(
        island_top,
        island_top,
        axes="xy",
        inner_elem="glass_panel",
        outer_elem="countertop_slab",
        margin=0.006,
        name="glass cooktop fits in countertop recess",
    )

    return ctx.report()


object_model = build_object_model()
