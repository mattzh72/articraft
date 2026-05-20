from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """Centered rounded rectangular appliance shell."""
    return cq.Workplane("XY").box(*size).edges().fillet(radius)


def _waffle_plate(
    length: float,
    width: float,
    slab_h: float,
    *,
    rib_h: float,
    rib_w: float,
    rib_count: int,
    center: tuple[float, float, float],
    direction: int,
) -> cq.Workplane:
    """One cast cooking plate with a raised square waffle grid."""
    cx, cy, cz = center
    plate = cq.Workplane("XY").box(length, width, slab_h).translate((cx, cy, cz))

    # Low perimeter lip keeps batter visually contained.
    lip_h = rib_h * 0.85
    lip_t = rib_w * 1.35
    rib_z = cz + direction * (slab_h * 0.5 + rib_h * 0.5 - 0.0006)
    lip_z = cz + direction * (slab_h * 0.5 + lip_h * 0.5 - 0.0006)
    for y in (-width * 0.5 + lip_t * 0.5, width * 0.5 - lip_t * 0.5):
        plate = plate.union(
            cq.Workplane("XY")
            .box(length, lip_t, lip_h)
            .translate((cx, cy + y, lip_z))
        )
    for x in (-length * 0.5 + lip_t * 0.5, length * 0.5 - lip_t * 0.5):
        plate = plate.union(
            cq.Workplane("XY")
            .box(lip_t, width, lip_h)
            .translate((cx + x, cy, lip_z))
        )

    usable_x = length - 2.0 * lip_t
    usable_y = width - 2.0 * lip_t
    if rib_count > 1:
        pitch_x = usable_x / rib_count
        pitch_y = usable_y / rib_count
        for i in range(1, rib_count):
            x = -usable_x * 0.5 + i * pitch_x
            plate = plate.union(
                cq.Workplane("XY")
                .box(rib_w, usable_y, rib_h)
                .translate((cx + x, cy, rib_z))
            )
            y = -usable_y * 0.5 + i * pitch_y
            plate = plate.union(
                cq.Workplane("XY")
                .box(usable_x, rib_w, rib_h)
                .translate((cx, cy + y, rib_z))
            )

    return plate


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_waffle_maker")

    black_plastic = model.material("satin_black_plastic", rgba=(0.015, 0.014, 0.013, 1.0))
    dark_metal = model.material("seasoned_cast_aluminum", rgba=(0.025, 0.024, 0.022, 1.0))
    warm_steel = model.material("brushed_warm_steel", rgba=(0.62, 0.57, 0.50, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.005, 0.005, 0.005, 1.0))
    amber = model.material("amber_indicator_lens", rgba=(1.0, 0.46, 0.05, 1.0))
    white = model.material("white_marking", rgba=(0.96, 0.94, 0.86, 1.0))

    base = model.part("base")
    base_shell = _rounded_box((0.320, 0.240, 0.065), 0.014).translate((0.0, 0.0, 0.040))
    base.visual(
        mesh_from_cadquery(base_shell, "base_shell", tolerance=0.0008),
        material=black_plastic,
        name="base_shell",
    )

    lower_plate_shape = _waffle_plate(
        0.232,
        0.172,
        0.012,
        rib_h=0.006,
        rib_w=0.0065,
        rib_count=4,
        center=(-0.020, 0.0, 0.077),
        direction=1,
    )
    base.visual(
        mesh_from_cadquery(lower_plate_shape, "lower_plate", tolerance=0.0007),
        material=dark_metal,
        name="lower_plate",
    )

    # Side control escutcheon and small indicator lamp are rigidly mounted to the base.
    base.visual(
        Box((0.098, 0.010, 0.058)),
        origin=Origin(xyz=(-0.035, -0.1245, 0.055)),
        material=warm_steel,
        name="side_panel",
    )
    base.visual(
        Cylinder(radius=0.006, length=0.0025),
        origin=Origin(xyz=(0.004, -0.1305, 0.072), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=amber,
        name="indicator_lamp",
    )

    # Rear hinge: fixed outer knuckles on the base, with brackets tied into the rear shell.
    for idx, y in enumerate((-0.081, 0.081)):
        base.visual(
            Box((0.026, 0.047, 0.042)),
            origin=Origin(xyz=(0.164, y, 0.092)),
            material=black_plastic,
            name=f"hinge_bracket_{idx}",
        )
        base.visual(
            Cylinder(radius=0.010, length=0.052),
            origin=Origin(xyz=(0.145, y, 0.118), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=warm_steel,
            name=f"fixed_hinge_knuckle_{idx}",
        )

    for idx, (x, y) in enumerate(
        ((-0.115, -0.075), (-0.115, 0.075), (0.105, -0.075), (0.105, 0.075))
    ):
        base.visual(
            Cylinder(radius=0.017, length=0.012),
            origin=Origin(xyz=(x, y, 0.0025)),
            material=rubber,
            name=f"foot_{idx}",
        )

    upper = model.part("upper_plate")
    upper_shell = _rounded_box((0.282, 0.232, 0.052), 0.014).translate((-0.153, 0.0, 0.022))
    upper.visual(
        mesh_from_cadquery(upper_shell, "upper_shell", tolerance=0.0008),
        material=black_plastic,
        name="upper_shell",
    )
    upper_cooking_shape = _waffle_plate(
        0.222,
        0.166,
        0.012,
        rib_h=0.006,
        rib_w=0.0065,
        rib_count=4,
        center=(-0.150, 0.0, -0.008),
        direction=-1,
    )
    upper.visual(
        mesh_from_cadquery(upper_cooking_shape, "upper_plate_grid", tolerance=0.0007),
        material=dark_metal,
        name="upper_plate_grid",
    )
    upper.visual(
        Cylinder(radius=0.010, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=warm_steel,
        name="moving_hinge_knuckle",
    )
    upper.visual(
        Box((0.018, 0.070, 0.024)),
        origin=Origin(xyz=(-0.010, 0.0, 0.006)),
        material=black_plastic,
        name="hinge_leaf",
    )
    handle_shape = _rounded_box((0.038, 0.162, 0.026), 0.007).translate((-0.305, 0.0, 0.014))
    upper.visual(
        mesh_from_cadquery(handle_shape, "front_handle", tolerance=0.0008),
        material=black_plastic,
        name="front_handle",
    )

    knob = model.part("browning_knob")
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.048,
            0.028,
            body_style="skirted",
            top_diameter=0.038,
            skirt=KnobSkirt(0.054, 0.006, flare=0.06, chamfer=0.001),
            grip=KnobGrip(style="fluted", count=18, depth=0.0014),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0006),
            center=False,
        ),
        "browning_knob",
    )
    knob.visual(
        knob_mesh,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_plastic,
        name="knob_cap",
    )
    knob.visual(
        Box((0.004, 0.0012, 0.021)),
        origin=Origin(xyz=(0.0, -0.0284, 0.010)),
        material=white,
        name="knob_pointer",
    )

    model.articulation(
        "base_to_upper",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper,
        origin=Origin(xyz=(0.145, 0.0, 0.118)),
        # The closed upper plate extends forward along child -X; +Y opens it upward.
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.35),
    )
    model.articulation(
        "base_to_knob",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=knob,
        origin=Origin(xyz=(-0.035, -0.1295, 0.055)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=6.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    upper = object_model.get_part("upper_plate")
    knob = object_model.get_part("browning_knob")
    hinge = object_model.get_articulation("base_to_upper")
    knob_joint = object_model.get_articulation("base_to_knob")

    ctx.check(
        "browning knob is continuous",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint type is {knob_joint.articulation_type}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            upper,
            base,
            axis="z",
            positive_elem="upper_plate_grid",
            negative_elem="lower_plate",
            min_gap=0.003,
            max_gap=0.018,
            name="closed plates have cooking clearance",
        )
        ctx.expect_overlap(
            upper,
            base,
            axes="xy",
            elem_a="upper_plate_grid",
            elem_b="lower_plate",
            min_overlap=0.14,
            name="upper and lower waffle grids align",
        )
        ctx.expect_gap(
            base,
            knob,
            axis="y",
            positive_elem="side_panel",
            negative_elem="knob_cap",
            max_gap=0.003,
            max_penetration=0.001,
            name="knob face is mounted on side panel",
        )
        closed_handle = ctx.part_element_world_aabb(upper, elem="front_handle")

    with ctx.pose({hinge: 1.15, knob_joint: 2.4}):
        open_handle = ctx.part_element_world_aabb(upper, elem="front_handle")
        ctx.check(
            "upper plate opens upward",
            closed_handle is not None
            and open_handle is not None
            and open_handle[0][2] > closed_handle[0][2] + 0.10,
            details=f"closed_handle={closed_handle}, open_handle={open_handle}",
        )

    return ctx.report()


object_model = build_object_model()
