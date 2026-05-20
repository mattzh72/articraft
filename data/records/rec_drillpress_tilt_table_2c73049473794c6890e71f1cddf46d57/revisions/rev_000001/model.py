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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="radial_arm_drill_press")

    cast_iron = model.material("cast_iron", rgba=(0.34, 0.38, 0.42, 1.0))
    machine_gray = model.material("machine_gray", rgba=(0.60, 0.62, 0.64, 1.0))
    steel = model.material("steel", rgba=(0.73, 0.75, 0.78, 1.0))
    black = model.material("black", rgba=(0.12, 0.12, 0.13, 1.0))

    def shell_mesh(*, outer_radius: float, inner_radius: float, length: float, name: str):
        half = length / 2.0
        geom = LatheGeometry.from_shell_profiles(
            [(outer_radius, -half), (outer_radius, half)],
            [(inner_radius, -half), (inner_radius, half)],
            segments=56,
            start_cap="flat",
            end_cap="flat",
        )
        return mesh_from_geometry(geom, name)

    frame = model.part("frame")
    frame.visual(
        Box((0.92, 0.68, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=cast_iron,
        name="base_plate",
    )
    frame.visual(
        Box((0.34, 0.28, 0.42)),
        origin=Origin(xyz=(0.0, 0.0, 0.29)),
        material=cast_iron,
        name="column_plinth",
    )
    frame.visual(
        Cylinder(radius=0.07, length=1.46),
        origin=Origin(xyz=(0.0, 0.0, 0.81)),
        material=machine_gray,
        name="round_column",
    )
    frame.visual(
        Cylinder(radius=0.09, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 1.565)),
        material=cast_iron,
        name="column_cap",
    )
    frame.visual(
        Box((0.58, 0.40, 0.05)),
        origin=Origin(xyz=(0.29, 0.0, 0.62)),
        material=machine_gray,
        name="work_table",
    )
    frame.visual(
        Box((0.20, 0.16, 0.19)),
        origin=Origin(xyz=(0.10, 0.0, 0.50)),
        material=cast_iron,
        name="table_brace",
    )

    arm = model.part("arm")
    arm.visual(
        shell_mesh(
            outer_radius=0.16,
            inner_radius=0.105,
            length=0.24,
            name="arm_column_sleeve_mesh",
        ),
        material=cast_iron,
        name="column_sleeve",
    )
    for name, xyz in (
        ("front_bearing_roller", (0.095, 0.0, 0.0)),
        ("rear_bearing_roller", (-0.095, 0.0, 0.0)),
        ("left_bearing_roller", (0.0, 0.095, 0.0)),
        ("right_bearing_roller", (0.0, -0.095, 0.0)),
    ):
        arm.visual(
            Cylinder(radius=0.025, length=0.20),
            origin=Origin(xyz=xyz),
            material=steel,
            name=name,
        )
    arm.visual(
        Box((1.02, 0.18, 0.14)),
        origin=Origin(xyz=(0.59, 0.0, 0.0)),
        material=cast_iron,
        name="arm_beam",
    )

    head = model.part("head")
    head.visual(
        Box((0.34, 0.32, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.101)),
        material=cast_iron,
        name="carriage_top",
    )
    head.visual(
        Box((0.34, 0.024, 0.22)),
        origin=Origin(xyz=(0.0, 0.107, -0.02)),
        material=cast_iron,
        name="carriage_left_wall",
    )
    head.visual(
        Box((0.34, 0.024, 0.22)),
        origin=Origin(xyz=(0.0, -0.107, -0.02)),
        material=cast_iron,
        name="carriage_right_wall",
    )
    head.visual(
        Box((0.28, 0.20, 0.18)),
        origin=Origin(xyz=(-0.03, 0.0, 0.205)),
        material=cast_iron,
        name="motor_housing",
    )
    head.visual(
        Box((0.20, 0.06, 0.18)),
        origin=Origin(xyz=(0.0, 0.125, -0.18)),
        material=cast_iron,
        name="gearbox_left",
    )
    head.visual(
        Box((0.20, 0.06, 0.18)),
        origin=Origin(xyz=(0.0, -0.125, -0.18)),
        material=cast_iron,
        name="gearbox_right",
    )
    head.visual(
        Box((0.05, 0.20, 0.08)),
        origin=Origin(xyz=(0.11, 0.0, -0.15)),
        material=cast_iron,
        name="spindle_front_rib",
    )
    head.visual(
        Box((0.05, 0.20, 0.08)),
        origin=Origin(xyz=(-0.11, 0.0, -0.15)),
        material=cast_iron,
        name="spindle_rear_rib",
    )
    head.visual(
        Box((0.22, 0.012, 0.10)),
        origin=Origin(xyz=(0.0, 0.096, 0.0)),
        material=steel,
        name="left_guide_pad",
    )
    head.visual(
        Box((0.22, 0.012, 0.10)),
        origin=Origin(xyz=(0.0, -0.096, 0.0)),
        material=steel,
        name="right_guide_pad",
    )
    head.visual(
        shell_mesh(
            outer_radius=0.09,
            inner_radius=0.062,
            length=0.26,
            name="spindle_sleeve_mesh",
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.30)),
        material=machine_gray,
        name="spindle_sleeve",
    )
    head.visual(
        Box((0.08, 0.07, 0.03)),
        origin=Origin(xyz=(0.0, 0.09, -0.415)),
        material=steel,
        name="left_quill_shoe",
    )
    head.visual(
        Box((0.08, 0.07, 0.03)),
        origin=Origin(xyz=(0.0, -0.09, -0.415)),
        material=steel,
        name="right_quill_shoe",
    )
    head.visual(
        Box((0.04, 0.03, 0.18)),
        origin=Origin(xyz=(0.0, 0.11, -0.34)),
        material=cast_iron,
        name="left_shoe_post",
    )
    head.visual(
        Box((0.04, 0.03, 0.18)),
        origin=Origin(xyz=(0.0, -0.11, -0.34)),
        material=cast_iron,
        name="right_shoe_post",
    )

    quill = model.part("quill")
    quill.visual(
        Cylinder(radius=0.055, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, -0.11)),
        material=steel,
        name="quill_tube",
    )
    quill.visual(
        Cylinder(radius=0.032, length=0.07),
        origin=Origin(xyz=(0.0, 0.0, -0.255)),
        material=black,
        name="chuck_body",
    )
    quill.visual(
        Cylinder(radius=0.007, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, -0.38)),
        material=steel,
        name="drill_bit",
    )

    model.articulation(
        "column_to_arm",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 1.30)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=350.0, velocity=1.2),
    )
    model.articulation(
        "arm_to_head",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=head,
        origin=Origin(xyz=(0.40, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=500.0,
            velocity=0.20,
            lower=0.0,
            upper=0.45,
        ),
    )
    model.articulation(
        "head_to_quill",
        ArticulationType.PRISMATIC,
        parent=head,
        child=quill,
        origin=Origin(xyz=(0.0, 0.0, -0.18)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=1800.0,
            velocity=0.18,
            lower=0.0,
            upper=0.18,
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

    frame = object_model.get_part("frame")
    arm = object_model.get_part("arm")
    head = object_model.get_part("head")
    quill = object_model.get_part("quill")

    arm_swing = object_model.get_articulation("column_to_arm")
    head_slide = object_model.get_articulation("arm_to_head")
    quill_drop = object_model.get_articulation("head_to_quill")

    arm_beam = arm.get_visual("arm_beam")
    carriage_top = head.get_visual("carriage_top")
    spindle_sleeve = head.get_visual("spindle_sleeve")
    left_quill_shoe = head.get_visual("left_quill_shoe")
    quill_tube = quill.get_visual("quill_tube")

    ctx.expect_gap(
        head,
        arm,
        axis="z",
        positive_elem=carriage_top,
        negative_elem=arm_beam,
        min_gap=0.006,
        max_gap=0.03,
        name="carriage roof clears the arm beam",
    )
    ctx.expect_overlap(
        head,
        arm,
        axes="xy",
        elem_a=carriage_top,
        elem_b=arm_beam,
        min_overlap=0.17,
        name="head carriage stays centered over the radial arm",
    )
    ctx.expect_overlap(
        quill,
        head,
        axes="z",
        elem_a=quill_tube,
        elem_b=spindle_sleeve,
        min_overlap=0.20,
        name="retracted quill remains guided inside the spindle sleeve",
    )
    ctx.expect_contact(
        quill,
        head,
        elem_a=quill_tube,
        elem_b=left_quill_shoe,
        name="quill bears against the lower guide shoe when retracted",
    )

    with ctx.pose({head_slide: head_slide.motion_limits.upper}):
        ctx.expect_overlap(
            head,
            arm,
            axes="xy",
            elem_a=carriage_top,
            elem_b=arm_beam,
            min_overlap=0.17,
            name="head carriage remains captured on the arm at full reach",
        )

    with ctx.pose({quill_drop: quill_drop.motion_limits.upper}):
        ctx.expect_overlap(
            quill,
            head,
            axes="z",
            elem_a=quill_tube,
            elem_b=spindle_sleeve,
            min_overlap=0.04,
            name="extended quill still retains insertion in the spindle sleeve",
        )

    with ctx.pose({arm_swing: 0.0, head_slide: 0.0, quill_drop: 0.0}):
        rest_head = ctx.part_world_position(head)
        rest_quill = ctx.part_world_position(quill)
        rest_arm = ctx.part_world_position(arm)

    with ctx.pose({arm_swing: math.pi / 2.0, head_slide: 0.0, quill_drop: 0.0}):
        swung_head = ctx.part_world_position(head)

    with ctx.pose({arm_swing: 0.0, head_slide: head_slide.motion_limits.upper, quill_drop: 0.0}):
        extended_head = ctx.part_world_position(head)

    with ctx.pose({arm_swing: 0.0, head_slide: 0.0, quill_drop: quill_drop.motion_limits.upper}):
        dropped_quill = ctx.part_world_position(quill)

    rest_radius = None
    swung_radius = None
    if rest_head is not None:
        rest_radius = math.hypot(rest_head[0], rest_head[1])
    if swung_head is not None:
        swung_radius = math.hypot(swung_head[0], swung_head[1])

    ctx.check(
        "arm sweeps around the round column",
        (
            rest_head is not None
            and swung_head is not None
            and rest_radius is not None
            and swung_radius is not None
            and abs(rest_radius - swung_radius) < 0.02
            and swung_head[1] > 0.30
            and abs(swung_head[0]) < 0.05
        ),
        details=f"rest_head={rest_head}, swung_head={swung_head}",
    )
    ctx.check(
        "drill head slides outward along the arm",
        (
            rest_head is not None
            and extended_head is not None
            and extended_head[0] > rest_head[0] + 0.35
            and abs(extended_head[1] - rest_head[1]) < 0.01
            and abs(extended_head[2] - rest_head[2]) < 0.01
        ),
        details=f"rest_head={rest_head}, extended_head={extended_head}",
    )
    ctx.check(
        "quill drops vertically from the head",
        (
            rest_quill is not None
            and dropped_quill is not None
            and dropped_quill[2] < rest_quill[2] - 0.10
            and abs(dropped_quill[0] - rest_quill[0]) < 0.002
            and abs(dropped_quill[1] - rest_quill[1]) < 0.002
        ),
        details=f"rest_quill={rest_quill}, dropped_quill={dropped_quill}",
    )
    ctx.check(
        "arm articulation stays on the column axis",
        rest_arm is not None and abs(rest_arm[0]) < 1e-6 and abs(rest_arm[1]) < 1e-6,
        details=f"arm_origin={rest_arm}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
