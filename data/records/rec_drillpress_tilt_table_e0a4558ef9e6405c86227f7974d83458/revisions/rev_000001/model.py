from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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
    model = ArticulatedObject(name="gang_drill_press")

    machine_paint = model.material("machine_paint", rgba=(0.31, 0.43, 0.36, 1.0))
    cast_iron = model.material("cast_iron", rgba=(0.27, 0.28, 0.30, 1.0))
    motor_black = model.material("motor_black", rgba=(0.16, 0.17, 0.18, 1.0))
    steel = model.material("steel", rgba=(0.73, 0.75, 0.78, 1.0))
    tool_steel = model.material("tool_steel", rgba=(0.55, 0.57, 0.60, 1.0))

    beam_length = 1.56
    beam_depth = 0.28
    beam_height = 0.18
    beam_center_z = 1.48
    column_x = 0.70
    head_x_positions = (-0.54, -0.18, 0.18, 0.54)

    body = model.part("body")
    body.visual(
        Box((0.36, 0.54, 0.64)),
        origin=Origin(xyz=(-column_x, 0.0, 0.32)),
        material=machine_paint,
        name="left_pedestal",
    )
    body.visual(
        Box((0.36, 0.54, 0.64)),
        origin=Origin(xyz=(column_x, 0.0, 0.32)),
        material=machine_paint,
        name="right_pedestal",
    )
    body.visual(
        Box((1.46, 0.24, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.67)),
        material=cast_iron,
        name="bed_casting",
    )
    body.visual(
        Box((1.42, 0.05, 0.04)),
        origin=Origin(xyz=(0.0, 0.09, 0.72)),
        material=steel,
        name="left_way",
    )
    body.visual(
        Box((1.42, 0.05, 0.04)),
        origin=Origin(xyz=(0.0, -0.09, 0.72)),
        material=steel,
        name="right_way",
    )
    body.visual(
        Box((0.14, 0.22, 0.75)),
        origin=Origin(xyz=(-column_x, 0.0, 1.015)),
        material=machine_paint,
        name="left_column",
    )
    body.visual(
        Box((0.14, 0.22, 0.75)),
        origin=Origin(xyz=(column_x, 0.0, 1.015)),
        material=machine_paint,
        name="right_column",
    )
    body.visual(
        Box((beam_length, beam_depth, beam_height)),
        origin=Origin(xyz=(0.0, 0.0, beam_center_z)),
        material=machine_paint,
        name="beam",
    )
    body.visual(
        Box((beam_length, 0.09, 0.08)),
        origin=Origin(xyz=(0.0, 0.095, 1.35)),
        material=cast_iron,
        name="beam_front_lip",
    )

    for index, x_pos in enumerate(head_x_positions, start=1):
        body.visual(
            Box((0.18, 0.20, 0.11)),
            origin=Origin(xyz=(x_pos, 0.0, 1.335)),
            material=machine_paint,
            name=f"head_{index}_housing",
        )
        body.visual(
            Box((0.07, 0.038, 0.11)),
            origin=Origin(xyz=(x_pos, 0.051, 1.225)),
            material=cast_iron,
            name=f"head_{index}_front_guide",
        )
        body.visual(
            Box((0.07, 0.038, 0.11)),
            origin=Origin(xyz=(x_pos, -0.051, 1.225)),
            material=cast_iron,
            name=f"head_{index}_rear_guide",
        )
        body.visual(
            Cylinder(radius=0.05, length=0.16),
            origin=Origin(xyz=(x_pos, 0.0, 1.615), rpy=(1.5708, 0.0, 0.0)),
            material=motor_black,
            name=f"head_{index}_motor",
        )
        body.visual(
            Box((0.05, 0.06, 0.03)),
            origin=Origin(xyz=(x_pos, 0.0, 1.678)),
            material=motor_black,
            name=f"head_{index}_terminal_box",
        )

    body.inertial = Inertial.from_geometry(
        Box((1.56, 0.54, 1.65)),
        mass=340.0,
        origin=Origin(xyz=(0.0, 0.0, 0.825)),
    )

    table = model.part("sliding_table")
    table.visual(
        Box((1.18, 0.34, 0.07)),
        origin=Origin(xyz=(0.0, 0.0, 0.865)),
        material=cast_iron,
        name="table_top",
    )
    table.visual(
        Box((0.76, 0.22, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.8025)),
        material=machine_paint,
        name="table_saddle",
    )
    table.visual(
        Box((0.80, 0.045, 0.035)),
        origin=Origin(xyz=(0.0, 0.09, 0.7575)),
        material=steel,
        name="left_runner",
    )
    table.visual(
        Box((0.80, 0.045, 0.035)),
        origin=Origin(xyz=(0.0, -0.09, 0.7575)),
        material=steel,
        name="right_runner",
    )
    table.inertial = Inertial.from_geometry(
        Box((1.18, 0.34, 0.16)),
        mass=58.0,
        origin=Origin(xyz=(0.0, 0.0, 0.82)),
    )

    model.articulation(
        "table_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=table,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=320.0,
            velocity=0.35,
            lower=-0.25,
            upper=0.25,
        ),
    )

    for index, x_pos in enumerate(head_x_positions, start=1):
        quill = model.part(f"quill_{index}")
        quill.visual(
            Cylinder(radius=0.032, length=0.03),
            origin=Origin(xyz=(0.0, 0.0, -0.015)),
            material=steel,
            name="collar",
        )
        quill.visual(
            Cylinder(radius=0.026, length=0.055),
            origin=Origin(xyz=(0.0, 0.0, -0.0575)),
            material=steel,
            name="quill_body",
        )
        quill.visual(
            Cylinder(radius=0.018, length=0.03),
            origin=Origin(xyz=(0.0, 0.0, -0.10)),
            material=tool_steel,
            name="chuck",
        )
        quill.visual(
            Cylinder(radius=0.004, length=0.07),
            origin=Origin(xyz=(0.0, 0.0, -0.15)),
            material=tool_steel,
            name="bit",
        )
        quill.inertial = Inertial.from_geometry(
            Cylinder(radius=0.03, length=0.18),
            mass=2.8,
            origin=Origin(xyz=(0.0, 0.0, -0.09)),
        )

        model.articulation(
            f"head_{index}_drop",
            ArticulationType.PRISMATIC,
            parent=body,
            child=quill,
            origin=Origin(xyz=(x_pos, 0.0, 1.18)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=90.0,
                velocity=0.18,
                lower=0.0,
                upper=0.09,
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
    body = object_model.get_part("body")
    table = object_model.get_part("sliding_table")
    table_slide = object_model.get_articulation("table_slide")
    quill_parts = [object_model.get_part(f"quill_{index}") for index in range(1, 5)]
    quill_joints = [
        object_model.get_articulation(f"head_{index}_drop") for index in range(1, 5)
    ]

    housing_centers_x = []
    for index in range(1, 5):
        aabb = ctx.part_element_world_aabb(body, elem=f"head_{index}_housing")
        if aabb is None:
            ctx.fail(
                f"head {index} housing exists",
                details=f"missing element head_{index}_housing on body",
            )
            continue
        housing_centers_x.append((aabb[0][0] + aabb[1][0]) * 0.5)
    if len(housing_centers_x) == 4:
        spacings = [
            housing_centers_x[i + 1] - housing_centers_x[i] for i in range(3)
        ]
        ctx.check(
            "spindle heads are equally spaced on the beam",
            max(spacings) - min(spacings) <= 0.001,
            details=f"head x centers={housing_centers_x}, spacings={spacings}",
        )

    rest_table_pos = ctx.part_world_position(table)
    with ctx.pose({table_slide: table_slide.motion_limits.upper}):
        right_slide_pos = ctx.part_world_position(table)
        ctx.check(
            "table slides along the beam axis",
            rest_table_pos is not None
            and right_slide_pos is not None
            and right_slide_pos[0] > rest_table_pos[0] + 0.18
            and abs(right_slide_pos[1] - rest_table_pos[1]) <= 1e-6
            and abs(right_slide_pos[2] - rest_table_pos[2]) <= 1e-6,
            details=f"rest={rest_table_pos}, right_slide={right_slide_pos}",
        )
        ctx.expect_gap(
            table,
            body,
            axis="z",
            positive_elem="left_runner",
            negative_elem="left_way",
            max_gap=0.001,
            max_penetration=0.0,
            name="left runner stays seated on the left way",
        )
        ctx.expect_gap(
            table,
            body,
            axis="z",
            positive_elem="right_runner",
            negative_elem="right_way",
            max_gap=0.001,
            max_penetration=0.0,
            name="right runner stays seated on the right way",
        )
        ctx.expect_overlap(
            table,
            body,
            axes="x",
            elem_a="left_runner",
            elem_b="left_way",
            min_overlap=0.60,
            name="left runner retains engagement at full right slide",
        )
        ctx.expect_overlap(
            table,
            body,
            axes="x",
            elem_a="right_runner",
            elem_b="right_way",
            min_overlap=0.60,
            name="right runner retains engagement at full right slide",
        )

    rest_quill_positions = {
        part.name: ctx.part_world_position(part) for part in quill_parts
    }
    for index, (quill_part, quill_joint) in enumerate(
        zip(quill_parts, quill_joints),
        start=1,
    ):
        with ctx.pose({quill_joint: quill_joint.motion_limits.upper}):
            dropped_pos = ctx.part_world_position(quill_part)
            ctx.check(
                f"quill {index} drops vertically",
                rest_quill_positions[quill_part.name] is not None
                and dropped_pos is not None
                and dropped_pos[2] < rest_quill_positions[quill_part.name][2] - 0.06,
                details=(
                    f"rest={rest_quill_positions[quill_part.name]}, "
                    f"dropped={dropped_pos}"
                ),
            )
            unchanged_neighbors = []
            for other_part in quill_parts:
                if other_part.name == quill_part.name:
                    continue
                other_rest = rest_quill_positions[other_part.name]
                other_now = ctx.part_world_position(other_part)
                unchanged_neighbors.append(
                    other_rest is not None
                    and other_now is not None
                    and abs(other_now[2] - other_rest[2]) <= 1e-6
                )
            ctx.check(
                f"quill {index} drops independently",
                all(unchanged_neighbors),
                details=f"rest={rest_quill_positions}",
            )
            ctx.expect_gap(
                quill_part,
                table,
                axis="z",
                positive_elem="bit",
                negative_elem="table_top",
                min_gap=0.004,
                max_gap=0.02,
                name=f"quill {index} stops just above the shared table",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
