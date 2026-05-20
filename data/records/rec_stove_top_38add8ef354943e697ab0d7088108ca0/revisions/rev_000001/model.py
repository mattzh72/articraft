from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BODY_DEPTH = 0.33
BODY_WIDTH = 0.52
BODY_HEIGHT = 0.082
TOP_Z = BODY_HEIGHT
HALF_DEPTH = BODY_DEPTH * 0.5

LID_PANEL_DEPTH = 0.27
LID_PANEL_WIDTH = 0.50
LID_GLASS_THICKNESS = 0.006
LID_HINGE_X = HALF_DEPTH - 0.018
LID_HINGE_Z = TOP_Z + 0.013
LID_OPEN_ANGLE = math.radians(105.0)

KNOB_Y_POSITIONS = (-0.078, 0.078)


def _add_burner(stove, y_pos: float, *, prefix: str, grate_material, burner_material) -> None:
    burner_x = 0.010
    stove.visual(
        Cylinder(radius=0.058, length=0.008),
        origin=Origin(xyz=(burner_x, y_pos, TOP_Z + 0.004)),
        material=burner_material,
        name=f"{prefix}_bowl",
    )
    stove.visual(
        Cylinder(radius=0.025, length=0.010),
        origin=Origin(xyz=(burner_x, y_pos, TOP_Z + 0.013)),
        material=burner_material,
        name=f"{prefix}_cap",
    )
    stove.visual(
        Box((0.108, 0.010, 0.008)),
        origin=Origin(xyz=(burner_x, y_pos, TOP_Z + 0.018)),
        material=grate_material,
        name=f"{prefix}_grate_cross_x",
    )
    stove.visual(
        Box((0.010, 0.108, 0.008)),
        origin=Origin(xyz=(burner_x, y_pos, TOP_Z + 0.018)),
        material=grate_material,
        name=f"{prefix}_grate_cross_y",
    )
    for frame_name, frame_xyz, frame_size in (
        ("front", (burner_x, y_pos - 0.045, TOP_Z + 0.018), (0.108, 0.010, 0.008)),
        ("rear", (burner_x, y_pos + 0.045, TOP_Z + 0.018), (0.108, 0.010, 0.008)),
        ("left", (burner_x - 0.045, y_pos, TOP_Z + 0.018), (0.010, 0.108, 0.008)),
        ("right", (burner_x + 0.045, y_pos, TOP_Z + 0.018), (0.010, 0.108, 0.008)),
    ):
        stove.visual(
            Box(frame_size),
            origin=Origin(xyz=frame_xyz),
            material=grate_material,
            name=f"{prefix}_grate_frame_{frame_name}",
        )


def _add_knob(model: ArticulatedObject, stove, index: int, y_pos: float, *, knob_material, trim_material) -> None:
    knob = model.part(f"knob_{index}")
    knob.visual(
        Cylinder(radius=0.004, length=0.010),
        origin=Origin(xyz=(-0.005, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_material,
        name="shaft",
    )
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.040,
                0.022,
                body_style="skirted",
                top_diameter=0.032,
                skirt=KnobSkirt(0.046, 0.004, flare=0.04),
                grip=KnobGrip(style="fluted", count=14, depth=0.0011),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0006, angle_deg=0.0),
                center=False,
            ),
            f"stove_knob_{index}",
        ),
        origin=Origin(rpy=(0.0, -math.pi / 2.0, 0.0)),
        material=knob_material,
        name="knob_cap",
    )

    model.articulation(
        f"stove_to_knob_{index}",
        ArticulationType.CONTINUOUS,
        parent=stove,
        child=knob,
        origin=Origin(xyz=(-HALF_DEPTH, y_pos, 0.040)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.35, velocity=10.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="galley_stove_top")

    stainless = model.material("stainless", rgba=(0.72, 0.74, 0.76, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.18, 0.19, 1.0))
    burner_black = model.material("burner_black", rgba=(0.10, 0.10, 0.11, 1.0))
    knob_black = model.material("knob_black", rgba=(0.09, 0.09, 0.10, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.42, 0.51, 0.56, 0.35))

    stove = model.part("stove")
    stove.visual(
        Box((BODY_DEPTH, BODY_WIDTH, BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT * 0.5)),
        material=stainless,
        name="body_shell",
    )
    stove.visual(
        Box((0.295, 0.480, 0.004)),
        origin=Origin(xyz=(0.004, 0.0, TOP_Z + 0.002)),
        material=dark_steel,
        name="deck_plate",
    )
    stove.visual(
        Box((0.018, 0.360, 0.016)),
        origin=Origin(xyz=(LID_HINGE_X + 0.014, 0.0, TOP_Z + 0.006)),
        material=stainless,
        name="hinge_rail",
    )
    for support_index, support_y in enumerate((-0.175, 0.175)):
        stove.visual(
            Box((0.010, 0.040, 0.016)),
            origin=Origin(xyz=(LID_HINGE_X + 0.020, support_y, TOP_Z + 0.008)),
            material=stainless,
            name=f"hinge_support_{support_index}",
        )
    stove.visual(
        Box((0.018, 0.300, 0.038)),
        origin=Origin(xyz=(-HALF_DEPTH + 0.009, 0.0, 0.040)),
        material=dark_steel,
        name="control_panel",
    )
    for bezel_index, bezel_y in enumerate(KNOB_Y_POSITIONS):
        stove.visual(
            Cylinder(radius=0.026, length=0.003),
            origin=Origin(
                xyz=(-HALF_DEPTH - 0.0015, bezel_y, 0.040),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=dark_steel,
            name=f"knob_bezel_{bezel_index}",
        )

    _add_burner(
        stove,
        -0.108,
        prefix="burner_0",
        grate_material=dark_steel,
        burner_material=burner_black,
    )
    _add_burner(
        stove,
        0.108,
        prefix="burner_1",
        grate_material=dark_steel,
        burner_material=burner_black,
    )

    for knob_index, knob_y in enumerate(KNOB_Y_POSITIONS):
        _add_knob(
            model,
            stove,
            knob_index,
            knob_y,
            knob_material=knob_black,
            trim_material=dark_steel,
        )

    cover = model.part("cover")
    cover.visual(
        Box((LID_PANEL_DEPTH, LID_PANEL_WIDTH, LID_GLASS_THICKNESS)),
        origin=Origin(xyz=(-LID_PANEL_DEPTH * 0.5, 0.0, 0.0145)),
        material=smoked_glass,
        name="glass_panel",
    )
    cover.visual(
        Box((0.018, 0.448, 0.012)),
        origin=Origin(xyz=(-0.009, 0.0, 0.006)),
        material=stainless,
        name="rear_frame",
    )
    for arm_index, arm_y in enumerate((-0.190, 0.190)):
        cover.visual(
            Box((0.018, 0.030, 0.014)),
            origin=Origin(xyz=(0.002, arm_y, 0.007)),
            material=stainless,
            name=f"hinge_arm_{arm_index}",
        )
        cover.visual(
            Cylinder(radius=0.006, length=0.022),
            origin=Origin(xyz=(0.007, arm_y, 0.006), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=stainless,
            name=f"hinge_barrel_{arm_index}",
        )

    model.articulation(
        "stove_to_cover",
        ArticulationType.REVOLUTE,
        parent=stove,
        child=cover,
        origin=Origin(xyz=(LID_HINGE_X, 0.0, LID_HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.0,
            lower=0.0,
            upper=LID_OPEN_ANGLE,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stove = object_model.get_part("stove")
    cover = object_model.get_part("cover")
    knob_0 = object_model.get_part("knob_0")
    knob_1 = object_model.get_part("knob_1")
    cover_hinge = object_model.get_articulation("stove_to_cover")
    knob_joint_0 = object_model.get_articulation("stove_to_knob_0")
    knob_joint_1 = object_model.get_articulation("stove_to_knob_1")

    ctx.expect_gap(
        cover,
        stove,
        axis="z",
        positive_elem="glass_panel",
        negative_elem="burner_0_grate_cross_x",
        min_gap=0.002,
        max_gap=0.020,
        name="closed cover clears left grate",
    )
    ctx.expect_gap(
        cover,
        stove,
        axis="z",
        positive_elem="glass_panel",
        negative_elem="burner_1_grate_cross_x",
        min_gap=0.002,
        max_gap=0.020,
        name="closed cover clears right grate",
    )
    ctx.expect_overlap(
        cover,
        stove,
        axes="xy",
        elem_a="glass_panel",
        elem_b="deck_plate",
        min_overlap=0.24,
        name="closed cover spans the cooktop deck",
    )
    ctx.expect_contact(
        knob_0,
        stove,
        elem_a="shaft",
        elem_b="control_panel",
        name="first knob shaft seats on the front panel",
    )
    ctx.expect_contact(
        knob_1,
        stove,
        elem_a="shaft",
        elem_b="control_panel",
        name="second knob shaft seats on the front panel",
    )

    for joint, label in ((knob_joint_0, "first"), (knob_joint_1, "second")):
        limits = joint.motion_limits
        ctx.check(
            f"{label}_knob_is_continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and limits is not None
            and limits.lower is None
            and limits.upper is None,
            details=f"type={joint.articulation_type!r}, limits={limits!r}",
        )

    closed_glass = ctx.part_element_world_aabb(cover, elem="glass_panel")
    with ctx.pose({cover_hinge: LID_OPEN_ANGLE}):
        open_glass = ctx.part_element_world_aabb(cover, elem="glass_panel")

    open_ok = False
    details = f"closed={closed_glass!r}, open={open_glass!r}"
    if closed_glass is not None and open_glass is not None:
        closed_min, closed_max = closed_glass
        open_min, open_max = open_glass
        open_dx = float(open_max[0] - open_min[0])
        open_dz = float(open_max[2] - open_min[2])
        open_ok = (
            float(open_min[0]) > 0.12
            and open_dx < 0.08
            and open_dz > 0.22
            and float(open_max[2]) > 0.34
            and float(open_max[0]) > float(closed_max[0]) + 0.06
        )
    ctx.check("open cover stands behind the burners", open_ok, details=details)

    return ctx.report()


object_model = build_object_model()
