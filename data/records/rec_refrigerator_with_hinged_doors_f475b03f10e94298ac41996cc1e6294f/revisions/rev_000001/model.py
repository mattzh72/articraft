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


CABINET_W = 1.20
CABINET_D = 0.72
CABINET_H = 2.08
FRONT_Y = -CABINET_D / 2.0

BASE_H = 0.10
FASCIA_H = 0.22
DOOR_BOTTOM = BASE_H
DOOR_H = CABINET_H - FASCIA_H - BASE_H
DOOR_T = 0.070
DOOR_W = 0.560
HINGE_X = 0.635
HINGE_Y = FRONT_Y - DOOR_T / 2.0 - 0.006

BARREL_LEN = 0.280
BARREL_RADIUS = 0.022
HINGE_PIN_RADIUS = 0.010
BARREL_LOCAL_Z = {
    "lower": 0.240,
    "middle": 0.880,
    "upper": 1.520,
}
CLIP_WORLD_Z = (0.550, 1.220, 1.820)


def _add_door_visuals(door, *, direction: float, metal, dark, handle_mat) -> None:
    """Add one side-by-side refrigerator door in a hinge-line child frame."""

    start_from_hinge = 0.065
    slab_center_x = direction * (start_from_hinge + DOOR_W / 2.0)
    front_y = -DOOR_T / 2.0

    door.visual(
        Box((DOOR_W, DOOR_T, DOOR_H)),
        origin=Origin(xyz=(slab_center_x, 0.0, DOOR_H / 2.0)),
        material=metal,
        name="door_slab",
    )

    # Dark raised gaskets and stainless trim make the split-center commercial
    # door faces read as two separate insulated doors rather than one panel.
    trim_y = front_y - 0.004
    trim_thick_y = 0.010
    door.visual(
        Box((DOOR_W, trim_thick_y, 0.035)),
        origin=Origin(xyz=(slab_center_x, trim_y, DOOR_H - 0.0175)),
        material=dark,
        name="top_gasket",
    )
    door.visual(
        Box((DOOR_W, trim_thick_y, 0.035)),
        origin=Origin(xyz=(slab_center_x, trim_y, 0.0175)),
        material=dark,
        name="bottom_gasket",
    )
    door.visual(
        Box((0.020, trim_thick_y, DOOR_H - 0.070)),
        origin=Origin(
            xyz=(
                direction * (start_from_hinge + DOOR_W - 0.010),
                trim_y,
                DOOR_H / 2.0,
            )
        ),
        material=dark,
        name="center_gasket",
    )
    door.visual(
        Box((0.018, trim_thick_y, DOOR_H - 0.090)),
        origin=Origin(xyz=(direction * (start_from_hinge + 0.012), trim_y, DOOR_H / 2.0)),
        material=dark,
        name="outer_gasket",
    )

    # Hinge leaf: a full-height metal strap that physically ties the alternating
    # hinge barrels to the insulated door slab.
    door.visual(
        Box((0.085, 0.018, DOOR_H)),
        origin=Origin(xyz=(direction * 0.0425, -0.030, DOOR_H / 2.0)),
        material=handle_mat,
        name="hinge_leaf",
    )
    door.visual(
        Cylinder(radius=BARREL_RADIUS, length=BARREL_LEN),
        origin=Origin(xyz=(0.0, 0.0, BARREL_LOCAL_Z["lower"])),
        material=handle_mat,
        name="hinge_barrel_lower",
    )
    door.visual(
        Cylinder(radius=BARREL_RADIUS, length=BARREL_LEN),
        origin=Origin(xyz=(0.0, 0.0, BARREL_LOCAL_Z["middle"])),
        material=handle_mat,
        name="hinge_barrel_middle",
    )
    door.visual(
        Cylinder(radius=BARREL_RADIUS, length=BARREL_LEN),
        origin=Origin(xyz=(0.0, 0.0, BARREL_LOCAL_Z["upper"])),
        material=handle_mat,
        name="hinge_barrel_upper",
    )

    # Tall pull handle near the meeting seam, with three standoffs so the handle
    # is visibly mounted instead of floating in front of the door.
    handle_x = direction * (start_from_hinge + DOOR_W - 0.125)
    handle_y = front_y - 0.055
    door.visual(
        Cylinder(radius=0.018, length=1.250),
        origin=Origin(xyz=(handle_x, handle_y, DOOR_H / 2.0)),
        material=handle_mat,
        name="vertical_handle",
    )
    for i, zc in enumerate((0.360, DOOR_H / 2.0, DOOR_H - 0.360)):
        door.visual(
            Box((0.060, 0.062, 0.045)),
            origin=Origin(xyz=(handle_x, front_y - 0.027, zc)),
            material=handle_mat,
            name=f"handle_standoff_{i}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_side_by_side_refrigerator")

    painted = model.material("painted_cabinet", rgba=(0.78, 0.80, 0.78, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.68, 0.72, 0.74, 1.0))
    dark = model.material("black_gasket", rgba=(0.02, 0.025, 0.025, 1.0))
    shadow = model.material("dark_interior", rgba=(0.05, 0.06, 0.065, 1.0))
    handle_mat = model.material("polished_hardware", rgba=(0.86, 0.88, 0.86, 1.0))
    knob_mat = model.material("black_control", rgba=(0.015, 0.015, 0.018, 1.0))
    pointer_mat = model.material("white_pointer", rgba=(0.94, 0.92, 0.82, 1.0))

    cabinet = model.part("cabinet")

    # A tall commercial cabinet shell: back, side walls, top cap, base plinth,
    # upper fascia, and a central mullion behind the meeting seam.
    cabinet.visual(
        Box((CABINET_W, 0.045, CABINET_H)),
        origin=Origin(xyz=(0.0, CABINET_D / 2.0 - 0.0225, CABINET_H / 2.0)),
        material=painted,
        name="rear_panel",
    )
    for side, x in (("0", -CABINET_W / 2.0 + 0.025), ("1", CABINET_W / 2.0 - 0.025)):
        cabinet.visual(
            Box((0.050, CABINET_D, CABINET_H)),
            origin=Origin(xyz=(x, 0.0, CABINET_H / 2.0)),
            material=painted,
            name=f"side_wall_{side}",
        )
    cabinet.visual(
        Box((CABINET_W, CABINET_D, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, CABINET_H - 0.040)),
        material=painted,
        name="top_cap",
    )
    cabinet.visual(
        Box((CABINET_W, CABINET_D, BASE_H)),
        origin=Origin(xyz=(0.0, 0.0, BASE_H / 2.0)),
        material=painted,
        name="base_plinth",
    )
    cabinet.visual(
        Box((CABINET_W, 0.065, FASCIA_H)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.0325, CABINET_H - FASCIA_H / 2.0)),
        material=painted,
        name="fascia_panel",
    )
    cabinet.visual(
        Box((0.040, 0.045, DOOR_H)),
        origin=Origin(xyz=(0.0, FRONT_Y + 0.026, DOOR_BOTTOM + DOOR_H / 2.0)),
        material=shadow,
        name="center_mullion",
    )
    cabinet.visual(
        Box((CABINET_W - 0.10, 0.030, DOOR_H)),
        origin=Origin(xyz=(0.0, FRONT_Y + 0.050, DOOR_BOTTOM + DOOR_H / 2.0)),
        material=shadow,
        name="dark_door_pocket",
    )

    # Cabinet-side hinge pins and alternating clips.  The door barrels are
    # captured on these pins, while the clips tie the pins back into the side
    # walls so the doors visibly stay seated around their hinge axes.
    cabinet.visual(
        Cylinder(radius=HINGE_PIN_RADIUS, length=DOOR_H + 0.020),
        origin=Origin(xyz=(-HINGE_X, HINGE_Y, DOOR_BOTTOM + DOOR_H / 2.0)),
        material=handle_mat,
        name="hinge_pin_0",
    )
    cabinet.visual(
        Cylinder(radius=HINGE_PIN_RADIUS, length=DOOR_H + 0.020),
        origin=Origin(xyz=(HINGE_X, HINGE_Y, DOOR_BOTTOM + DOOR_H / 2.0)),
        material=handle_mat,
        name="hinge_pin_1",
    )
    for index, sign in (("0", -1.0), ("1", 1.0)):
        clip_x = sign * (CABINET_W / 2.0 + 0.017)
        for clip_i, zc in enumerate(CLIP_WORLD_Z):
            cabinet.visual(
                Box((0.070, 0.060, 0.120)),
                origin=Origin(xyz=(clip_x, HINGE_Y + 0.025, zc)),
                material=handle_mat,
                name=f"hinge_clip_{index}_{clip_i}",
            )

    door_0 = model.part("door_0")
    _add_door_visuals(door_0, direction=1.0, metal=stainless, dark=dark, handle_mat=handle_mat)
    door_1 = model.part("door_1")
    _add_door_visuals(door_1, direction=-1.0, metal=stainless, dark=dark, handle_mat=handle_mat)

    model.articulation(
        "cabinet_to_door_0",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door_0,
        origin=Origin(xyz=(-HINGE_X, HINGE_Y, DOOR_BOTTOM)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.4, lower=0.0, upper=1.75),
    )
    model.articulation(
        "cabinet_to_door_1",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door_1,
        origin=Origin(xyz=(HINGE_X, HINGE_Y, DOOR_BOTTOM)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.4, lower=0.0, upper=1.75),
    )

    control_knob = model.part("control_knob")
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.085,
            0.045,
            body_style="skirted",
            top_diameter=0.068,
            skirt=KnobSkirt(0.096, 0.010, flare=0.06, chamfer=0.001),
            grip=KnobGrip(style="fluted", count=20, depth=0.0014),
            indicator=KnobIndicator(style="line", mode="raised", depth=0.0010),
            center=False,
        ),
        "top_control_knob",
    )
    control_knob.visual(knob_mesh, material=knob_mat, name="knob_cap")
    control_knob.visual(
        Cylinder(radius=0.015, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, -0.0225)),
        material=knob_mat,
        name="control_stem",
    )
    control_knob.visual(
        Box((0.008, 0.036, 0.003)),
        origin=Origin(xyz=(0.0, 0.020, 0.046)),
        material=pointer_mat,
        name="pointer_mark",
    )
    model.articulation(
        "fascia_to_knob",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=control_knob,
        origin=Origin(
            xyz=(0.0, FRONT_Y - 0.065, CABINET_H - FASCIA_H / 2.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=6.0, lower=-2.35, upper=2.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    door_0 = object_model.get_part("door_0")
    door_1 = object_model.get_part("door_1")
    knob = object_model.get_part("control_knob")
    hinge_0 = object_model.get_articulation("cabinet_to_door_0")
    hinge_1 = object_model.get_articulation("cabinet_to_door_1")
    knob_joint = object_model.get_articulation("fascia_to_knob")

    for door, index in ((door_0, "0"), (door_1, "1")):
        for key in BARREL_LOCAL_Z:
            ctx.allow_overlap(
                cabinet,
                door,
                elem_a=f"hinge_pin_{index}",
                elem_b=f"hinge_barrel_{key}",
                reason=(
                    "The cabinet-side hinge pin is intentionally captured inside "
                    "the door barrel proxy so the commercial door remains seated."
                ),
            )
            ctx.expect_within(
                cabinet,
                door,
                axes="xy",
                inner_elem=f"hinge_pin_{index}",
                outer_elem=f"hinge_barrel_{key}",
                margin=0.001,
                name=f"hinge pin {index} centered in {key} barrel",
            )
            ctx.expect_overlap(
                cabinet,
                door,
                axes="z",
                elem_a=f"hinge_pin_{index}",
                elem_b=f"hinge_barrel_{key}",
                min_overlap=0.240,
                name=f"hinge pin {index} captured by {key} barrel",
            )

    ctx.allow_overlap(
        cabinet,
        knob,
        elem_a="fascia_panel",
        elem_b="control_stem",
        reason="The rotary control stem is intentionally seated through the fascia panel.",
    )
    ctx.expect_within(
        knob,
        cabinet,
        axes="xz",
        inner_elem="control_stem",
        outer_elem="fascia_panel",
        margin=0.002,
        name="control stem lies within fascia opening projection",
    )
    ctx.expect_overlap(
        knob,
        cabinet,
        axes="y",
        elem_a="control_stem",
        elem_b="fascia_panel",
        min_overlap=0.020,
        name="control stem penetrates fascia as a seated shaft",
    )

    # Closed doors sit just proud of the cabinet opening and overlap the cabinet
    # footprint without being welded into the front frame.
    ctx.expect_gap(
        cabinet,
        door_0,
        axis="y",
        positive_elem="center_mullion",
        negative_elem="door_slab",
        min_gap=0.004,
        max_gap=0.040,
        name="door 0 seated proud of center mullion",
    )
    ctx.expect_gap(
        cabinet,
        door_1,
        axis="y",
        positive_elem="center_mullion",
        negative_elem="door_slab",
        min_gap=0.004,
        max_gap=0.040,
        name="door 1 seated proud of center mullion",
    )
    ctx.expect_gap(
        door_1,
        door_0,
        axis="x",
        positive_elem="center_gasket",
        negative_elem="center_gasket",
        min_gap=0.006,
        max_gap=0.040,
        name="visible split between side by side doors",
    )

    # At an opened pose, the hinge pins remain captured on the vertical axes.
    with ctx.pose({hinge_0: 1.20, hinge_1: 1.20, knob_joint: 1.00}):
        ctx.expect_within(
            cabinet,
            door_0,
            axes="xy",
            inner_elem="hinge_pin_0",
            outer_elem="hinge_barrel_middle",
            margin=0.001,
            name="door 0 stays clipped while open",
        )
        ctx.expect_within(
            cabinet,
            door_1,
            axes="xy",
            inner_elem="hinge_pin_1",
            outer_elem="hinge_barrel_middle",
            margin=0.001,
            name="door 1 stays clipped while open",
        )

    return ctx.report()


object_model = build_object_model()
