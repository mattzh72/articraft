from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _rail_origin(start: tuple[float, float, float], end: tuple[float, float, float]) -> tuple[Origin, float]:
    """Return an origin for a box whose long local Z axis runs from start to end."""
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if abs(dx) > 1.0e-9:
        raise ValueError("This simple ladder helper expects rails in a Y-Z plane.")
    roll = math.atan2(-dy, dz)
    return Origin(xyz=((sx + ex) / 2.0, (sy + ey) / 2.0, (sz + ez) / 2.0), rpy=(roll, 0.0, 0.0)), length


def _add_y_bolt(part, name: str, xyz: tuple[float, float, float], material: Material) -> None:
    part.visual(
        Cylinder(radius=0.011, length=0.010),
        origin=Origin(xyz=xyz, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_a_frame_step_ladder")

    aluminum = Material("brushed_aluminum", color=(0.78, 0.80, 0.78, 1.0))
    dark_rubber = Material("black_rubber", color=(0.02, 0.02, 0.018, 1.0))
    hinge_steel = Material("hinge_steel", color=(0.50, 0.52, 0.54, 1.0))
    safety_label = Material("yellow_warning_label", color=(1.0, 0.78, 0.10, 1.0))

    front = model.part("front_frame")
    rear = model.part("rear_frame")

    # Coordinate convention: X is ladder width, Y is depth, Z is up.
    # The hinge line runs along X through the exposed barrels at the rear edge
    # of the top cap.  The front frame is the fixed/root link.
    hinge_xyz = (0.0, 0.105, 1.42)
    rail_thick_x = 0.055
    rail_thick_y = 0.045

    # Front climbing rails lean forward from the top cap to the front feet.
    for idx, x in enumerate((-0.315, 0.315)):
        origin, length = _rail_origin((x, -0.075, 1.39), (x, -0.525, 0.045))
        front.visual(
            Box((rail_thick_x, rail_thick_y, length)),
            origin=origin,
            material=aluminum,
            name=f"front_rail_{idx}",
        )

    # Wide step treads, with black anti-slip ribs on the visible top face.
    tread_specs = (
        ("lower_tread", -0.430, 0.32, 0.66),
        ("middle_tread", -0.325, 0.64, 0.64),
        ("upper_tread", -0.220, 0.96, 0.62),
        ("top_tread", -0.130, 1.20, 0.58),
    )
    for tread_name, y, z, width in tread_specs:
        front.visual(
            Box((width, 0.245, 0.045)),
            origin=Origin(xyz=(0.0, y + 0.020, z)),
            material=aluminum,
            name=tread_name,
        )
        for rib_i, rib_y in enumerate((-0.065, 0.0, 0.065)):
            front.visual(
                Box((width - 0.070, 0.012, 0.008)),
                origin=Origin(xyz=(0.0, y + 0.020 + rib_y, z + 0.026)),
                material=dark_rubber,
                name=f"{tread_name}_grip_{rib_i}",
            )

    # Top cap/platform bridges the side rails and visibly carries the hinge.
    front.visual(
        Box((0.78, 0.250, 0.080)),
        origin=Origin(xyz=(0.0, -0.060, 1.43)),
        material=aluminum,
        name="top_cap",
    )
    front.visual(
        Box((0.34, 0.055, 0.006)),
        origin=Origin(xyz=(0.0, -0.072, 1.4725)),
        material=safety_label,
        name="top_label",
    )

    # Broad rubber foot pads overlap the bottom of the rails.
    for idx, x in enumerate((-0.315, 0.315)):
        front.visual(
            Box((0.17, 0.135, 0.045)),
            origin=Origin(xyz=(x, -0.545, 0.025)),
            material=dark_rubber,
            name=f"front_foot_{idx}",
        )

    # Exposed hinge leaves and outer barrels mounted to the top cap.
    for idx, x, leaf_name, barrel_name in (
        (0, -0.260, "front_hinge_leaf_0", "front_hinge_barrel_0"),
        (1, 0.260, "front_hinge_leaf_1", "front_hinge_barrel_1"),
    ):
        front.visual(
            Box((0.155, 0.032, 0.105)),
            origin=Origin(xyz=(x, 0.075, 1.405)),
            material=hinge_steel,
            name=leaf_name,
        )
        front.visual(
            Cylinder(radius=0.022, length=0.145),
            origin=Origin(xyz=(x, hinge_xyz[1], hinge_xyz[2]), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hinge_steel,
            name=barrel_name,
        )
        _add_y_bolt(front, f"front_hinge_bolt_{idx}_0", (x - 0.045, 0.056, 1.385), hinge_steel)
        _add_y_bolt(front, f"front_hinge_bolt_{idx}_1", (x + 0.045, 0.056, 1.385), hinge_steel)
    front.visual(
        Cylinder(radius=0.010, length=0.700),
        origin=Origin(xyz=hinge_xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_steel,
        name="hinge_pin",
    )

    # Rear support frame is authored in the hinge frame.  At q=0 it is opened
    # backward, forming the A-frame; positive joint motion folds it toward the
    # front frame.
    for idx, x in enumerate((-0.315, 0.315)):
        origin, length = _rail_origin((x, 0.075, -0.105), (x, 0.700, -1.380))
        rear.visual(
            Box((rail_thick_x, rail_thick_y, length)),
            origin=origin,
            material=aluminum,
            name=f"rear_rail_{idx}",
        )

    # Cross members tie the rear rails together so the support reads as a single
    # braced ladder frame rather than two loose props.
    for name, y, z, sx in (
        ("rear_top_spreader", 0.080, -0.105, 0.700),
        ("rear_middle_spreader", 0.330, -0.610, 0.680),
        ("rear_lower_spreader", 0.565, -1.090, 0.660),
    ):
        rear.visual(
            Box((sx, 0.050, 0.050)),
            origin=Origin(xyz=(0.0, y, z)),
            material=aluminum,
            name=name,
        )

    for x, foot_name in ((-0.315, "rear_foot_0"), (0.315, "rear_foot_1")):
        rear.visual(
            Box((0.17, 0.135, 0.045)),
            origin=Origin(xyz=(x, 0.720, -1.395)),
            material=dark_rubber,
            name=foot_name,
        )

    # Central rear hinge leaf and barrel, interleaved between the two front
    # barrels so the hinge construction is clear without broad part overlap.
    rear.visual(
        Box((0.330, 0.055, 0.105)),
        origin=Origin(xyz=(0.0, 0.030, -0.065)),
        material=hinge_steel,
        name="rear_hinge_leaf",
    )
    rear.visual(
        Cylinder(radius=0.018, length=0.220),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_steel,
        name="rear_hinge_barrel",
    )
    _add_y_bolt(rear, "rear_hinge_bolt_0", (-0.075, 0.060, -0.045), hinge_steel)
    _add_y_bolt(rear, "rear_hinge_bolt_1", (0.075, 0.060, -0.045), hinge_steel)

    model.articulation(
        "top_hinge",
        ArticulationType.REVOLUTE,
        parent=front,
        child=rear,
        origin=Origin(xyz=hinge_xyz),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.2, lower=0.0, upper=0.62),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front = object_model.get_part("front_frame")
    rear = object_model.get_part("rear_frame")
    hinge = object_model.get_articulation("top_hinge")

    ctx.check(
        "single rear-frame hinge",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and hinge.parent == "front_frame"
        and hinge.child == "rear_frame",
        details=f"type={hinge.articulation_type}, parent={hinge.parent}, child={hinge.child}",
    )
    limits = hinge.motion_limits
    ctx.check(
        "hinge has folding travel",
        limits is not None and limits.lower == 0.0 and limits.upper is not None and limits.upper > 0.55,
        details=f"limits={limits}",
    )
    ctx.expect_overlap(
        front,
        rear,
        axes="yz",
        elem_a="front_hinge_barrel_0",
        elem_b="rear_hinge_barrel",
        min_overlap=0.030,
        name="hinge barrels share one hinge line",
    )
    ctx.expect_overlap(
        front,
        rear,
        axes="yz",
        elem_a="front_hinge_barrel_1",
        elem_b="rear_hinge_barrel",
        min_overlap=0.030,
        name="opposite hinge barrel shares hinge line",
    )
    ctx.allow_overlap(
        front,
        rear,
        elem_a="hinge_pin",
        elem_b="rear_hinge_barrel",
        reason="The continuous hinge pin is intentionally captured inside the rear hinge barrel.",
    )
    ctx.expect_within(
        front,
        rear,
        axes="yz",
        inner_elem="hinge_pin",
        outer_elem="rear_hinge_barrel",
        margin=0.001,
        name="hinge pin is centered in rear barrel",
    )
    ctx.expect_overlap(
        front,
        rear,
        axes="x",
        elem_a="hinge_pin",
        elem_b="rear_hinge_barrel",
        min_overlap=0.18,
        name="hinge pin passes through rear barrel",
    )

    rest_aabb = ctx.part_element_world_aabb(rear, elem="rear_foot_0")
    rest_center_y = None
    if rest_aabb is not None:
        rest_center_y = (rest_aabb[0][1] + rest_aabb[1][1]) / 2.0
    with ctx.pose({hinge: limits.upper if limits is not None and limits.upper is not None else 0.62}):
        folded_aabb = ctx.part_element_world_aabb(rear, elem="rear_foot_0")
        folded_center_y = None
        if folded_aabb is not None:
            folded_center_y = (folded_aabb[0][1] + folded_aabb[1][1]) / 2.0
    ctx.check(
        "rear frame folds toward climbing frame",
        rest_center_y is not None
        and folded_center_y is not None
        and folded_center_y < rest_center_y - 0.45,
        details=f"rest_y={rest_center_y}, folded_y={folded_center_y}",
    )

    return ctx.report()


object_model = build_object_model()
