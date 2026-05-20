from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _rail_origin(start: tuple[float, float, float], end: tuple[float, float, float]) -> tuple[Origin, float]:
    """Origin for a box whose local +Z axis runs from start to end."""
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if abs(dy) > 1e-6:
        raise ValueError("This ladder helper expects rails in the XZ plane.")
    pitch = math.atan2(dx, dz)
    return Origin(xyz=((sx + ex) / 2.0, (sy + ey) / 2.0, (sz + ez) / 2.0), rpy=(0.0, pitch, 0.0)), length


def _add_rail(part, name: str, start, end, *, size=(0.050, 0.055), material=None) -> None:
    origin, length = _rail_origin(start, end)
    part.visual(Box((size[0], size[1], length)), origin=origin, material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="apex_hinged_step_ladder")

    aluminum = model.material("brushed_aluminum", rgba=(0.78, 0.80, 0.78, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.05, 0.055, 0.06, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    orange_lock = model.material("orange_lock_bar", rgba=(0.95, 0.42, 0.08, 1.0))
    pin_metal = model.material("hinge_pin_metal", rgba=(0.55, 0.56, 0.55, 1.0))

    hinge_xyz = (0.09, 0.0, 1.33)
    front_y = 0.265

    front = model.part("front_section")

    # Fixed plastic top cap: broad, slightly proud, and part of the front assembly.
    front.visual(
        Box((0.30, 0.64, 0.070)),
        origin=Origin(xyz=(-0.115, 0.0, 1.350)),
        material=dark_plastic,
        name="top_cap",
    )
    front.visual(
        Box((0.20, 0.42, 0.010)),
        origin=Origin(xyz=(-0.125, 0.0, 1.390)),
        material=black_rubber,
        name="cap_tray_recess",
    )

    # Two front stiles lean forward from the fixed cap down to the front feet.
    for side, y in (("stile_0", -front_y), ("stile_1", front_y)):
        _add_rail(
            front,
            side,
            start=(-0.525, y, 0.035),
            end=(-0.105, y, 1.325),
            size=(0.052, 0.058),
            material=aluminum,
        )
        front.visual(
            Box((0.17, 0.12, 0.040)),
            origin=Origin(xyz=(-0.540, y, 0.020)),
            material=black_rubber,
            name=f"front_foot_{side[-1]}",
        )

    # Horizontal steps span between the two stiles and visibly tie them together.
    step_specs = (
        ("step_0", 0.40, -0.405, 0.64, 0.215),
        ("step_1", 0.73, -0.295, 0.60, 0.205),
        ("step_2", 1.05, -0.190, 0.56, 0.190),
    )
    for step_name, z, x, width, depth in step_specs:
        front.visual(
            Box((depth, width, 0.045)),
            origin=Origin(xyz=(x, 0.0, z)),
            material=aluminum,
            name=step_name,
        )
        for groove_i, gy in enumerate((-0.18, 0.0, 0.18)):
            front.visual(
                Box((depth * 0.78, 0.018, 0.006)),
                origin=Origin(xyz=(x, gy, z + 0.024)),
                material=dark_plastic,
                name=f"{step_name}_grip_{groove_i}",
            )

    # A lower cross tie and a small central fork carry the folding spreader bar.
    front.visual(
        Box((0.060, 0.54, 0.050)),
        origin=Origin(xyz=(-0.335, 0.0, 0.650)),
        material=aluminum,
        name="spreader_cross_tie",
    )
    front.visual(
        Box((0.040, 0.085, 0.105)),
        origin=Origin(xyz=(-0.280, 0.160, 0.735)),
        material=aluminum,
        name="spreader_hinge_web",
    )
    for i, y in enumerate((0.113, 0.207)):
        front.visual(
            Box((0.080, 0.014, 0.065)),
            origin=Origin(xyz=(-0.280, y, 0.650)),
            material=pin_metal,
            name=f"spreader_fork_{i}",
        )

    # Exposed apex hinge knuckles on the front frame; the rear brace owns the middle knuckle.
    for i, y in enumerate((-0.185, 0.185)):
        front.visual(
            Box((0.075, 0.070, 0.050)),
            origin=Origin(xyz=(0.055, y, 1.330)),
            material=aluminum,
            name=f"hinge_leaf_{i}",
        )
        front.visual(
            Cylinder(radius=0.026, length=0.150),
            origin=Origin(xyz=hinge_xyz[0:1] + (y,) + hinge_xyz[2:3], rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=pin_metal,
            name=f"front_hinge_knuckle_{i}",
        )

    rear = model.part("rear_brace")
    # Single rear brace leg, hinged at the apex and splayed back in the open pose.
    rear_foot = (0.56, 0.0, -1.300)
    _add_rail(
        rear,
        "single_leg",
        start=rear_foot,
        end=(0.0, 0.0, 0.0),
        size=(0.055, 0.070),
        material=aluminum,
    )
    rear.visual(
        Cylinder(radius=0.023, length=0.220),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=pin_metal,
        name="rear_hinge_knuckle",
    )
    rear.visual(
        Box((0.20, 0.22, 0.040)),
        origin=Origin(xyz=(0.565, 0.0, -1.310)),
        material=black_rubber,
        name="rear_foot",
    )
    # Lock receiver mounted to the rear brace at the same height as the open spreader bar.
    rear.visual(
        Box((0.060, 0.270, 0.070)),
        origin=Origin(xyz=(0.295, 0.095, -0.680)),
        material=aluminum,
        name="lock_mount",
    )
    for i, y in enumerate((0.110, 0.210)):
        rear.visual(
            Box((0.055, 0.012, 0.075)),
            origin=Origin(xyz=(0.300, y, -0.680)),
            material=pin_metal,
            name=f"lock_fork_{i}",
        )
    rear.visual(
        Box((0.018, 0.082, 0.050)),
        origin=Origin(xyz=(0.292, 0.160, -0.680)),
        material=pin_metal,
        name="lock_stop",
    )

    spreader = model.part("spreader_bar")
    # Child frame is the front hinge axis.  At q=0 the bar is the locked, nearly
    # horizontal link between the front section and the rear receiver.
    spreader.visual(
        Cylinder(radius=0.015, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=pin_metal,
        name="hinge_eye",
    )
    spreader.visual(
        Box((0.575, 0.026, 0.024)),
        origin=Origin(xyz=(0.302, 0.0, 0.0)),
        material=orange_lock,
        name="flat_bar",
    )
    spreader.visual(
        Box((0.040, 0.050, 0.035)),
        origin=Origin(xyz=(0.607, 0.0, 0.0)),
        material=pin_metal,
        name="end_latch",
    )

    rear_hinge = model.articulation(
        "apex_hinge",
        ArticulationType.REVOLUTE,
        parent=front,
        child=rear,
        origin=Origin(xyz=hinge_xyz),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=1.0, lower=0.0, upper=0.46),
    )
    rear_hinge.meta["description"] = "Rear single-leg brace folds about the ladder apex."

    spreader_hinge = model.articulation(
        "spreader_hinge",
        ArticulationType.REVOLUTE,
        parent=front,
        child=spreader,
        origin=Origin(xyz=(-0.280, 0.160, 0.650)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=-1.15, upper=0.0),
    )
    spreader_hinge.meta["description"] = "Folding spreader bar rotates up from its locked open position."

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front = object_model.get_part("front_section")
    rear = object_model.get_part("rear_brace")
    spreader = object_model.get_part("spreader_bar")
    apex_hinge = object_model.get_articulation("apex_hinge")
    spreader_hinge = object_model.get_articulation("spreader_hinge")

    ctx.check(
        "front section carries two stiles",
        front.get_visual("stile_0") is not None and front.get_visual("stile_1") is not None,
    )
    ctx.check(
        "top cap is fixed to front section",
        front.get_visual("top_cap") is not None and all(a.child != "front_section" for a in object_model.articulations),
    )
    ctx.check(
        "rear brace uses apex revolute hinge",
        apex_hinge.articulation_type == ArticulationType.REVOLUTE and apex_hinge.child == "rear_brace",
    )
    ctx.check(
        "spreader bar uses revolute hinge",
        spreader_hinge.articulation_type == ArticulationType.REVOLUTE and spreader_hinge.child == "spreader_bar",
    )

    # In the open, locked pose the latch sits just short of the rear stop, so it
    # reads as a real spreader lock without relying on broad interpenetration.
    ctx.expect_gap(
        rear,
        spreader,
        axis="x",
        positive_elem="lock_stop",
        negative_elem="end_latch",
        min_gap=0.0,
        max_gap=0.035,
        name="spreader latch reaches rear lock stop",
    )
    ctx.expect_overlap(
        spreader,
        rear,
        axes="z",
        elem_a="end_latch",
        elem_b="lock_stop",
        min_overlap=0.020,
        name="spreader latch aligns vertically with rear lock",
    )

    rear_open = ctx.part_element_world_aabb(rear, elem="rear_foot")
    with ctx.pose({apex_hinge: 0.42}):
        rear_folded = ctx.part_element_world_aabb(rear, elem="rear_foot")
    ctx.check(
        "rear brace folds toward front section",
        rear_open is not None
        and rear_folded is not None
        and rear_folded[0][0] < rear_open[0][0] - 0.18,
        details=f"open={rear_open}, folded={rear_folded}",
    )

    bar_open = ctx.part_element_world_aabb(spreader, elem="end_latch")
    with ctx.pose({spreader_hinge: -0.85}):
        bar_folded = ctx.part_element_world_aabb(spreader, elem="end_latch")
    ctx.check(
        "spreader bar folds upward from locked position",
        bar_open is not None
        and bar_folded is not None
        and bar_folded[0][2] > bar_open[0][2] + 0.25,
        details=f"open={bar_open}, folded={bar_folded}",
    )

    return ctx.report()


object_model = build_object_model()
