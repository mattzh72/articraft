from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


PLATE_THICKNESS = 0.004
PIN_RADIUS = 0.0065
PIN_HOLE_RADIUS = 0.010
PIN_SPAN = 0.088

PROXIMAL_LENGTH = 0.105
MIDDLE_LENGTH = 0.078
DISTAL_LENGTH = 0.060

INNER_PLATE_Y = 0.020
OUTER_PLATE_Y = 0.032


def _circle_profile(cx: float, cz: float, radius: float, segments: int = 28) -> list[tuple[float, float]]:
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * i / segments),
            cz + radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _side_plate_mesh(
    *,
    length: float,
    proximal_height: float,
    distal_height: float,
    distal_hole: bool,
    name: str,
):
    """Transparent X-Z side plate, extruded to a thin Y thickness."""
    proximal_pad = max(0.020, PIN_HOLE_RADIUS + 0.009)
    distal_pad = max(0.018, PIN_HOLE_RADIUS + 0.008)
    neck = 0.022

    outer = [
        (-proximal_pad, -0.5 * proximal_height),
        (neck, -0.5 * proximal_height),
        (length - neck, -0.5 * distal_height),
        (length + distal_pad, -0.5 * distal_height),
        (length + distal_pad, 0.5 * distal_height),
        (length - neck, 0.5 * distal_height),
        (neck, 0.5 * proximal_height),
        (-proximal_pad, 0.5 * proximal_height),
    ]
    holes = [_circle_profile(0.0, 0.0, PIN_HOLE_RADIUS)]
    if distal_hole:
        holes.append(_circle_profile(length, 0.0, PIN_HOLE_RADIUS))

    mesh = ExtrudeWithHolesGeometry(outer, holes, PLATE_THICKNESS, center=True)
    # The extrusion is along mesh-local Z; rotate so thickness becomes local Y
    # and the profile's second coordinate becomes the link's Z height.
    mesh.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(mesh, name)


def _add_pin(part, *, plate_y: float, material: Material) -> None:
    pin_axis = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))
    part.visual(
        Cylinder(radius=PIN_RADIUS, length=PIN_SPAN),
        origin=pin_axis,
        material=material,
        name="proximal_pin",
    )
    for side, y in (("upper", plate_y), ("lower", -plate_y)):
        part.visual(
            Cylinder(radius=0.012, length=0.006),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=material,
            name=f"{side}_bushing",
        )
    for side, y in (("upper", 0.043), ("lower", -0.043)):
        part.visual(
            Cylinder(radius=0.011, length=0.005),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=material,
            name=f"{side}_pin_head",
        )


def _add_phalanx(
    part,
    *,
    length: float,
    proximal_height: float,
    distal_height: float,
    plate_y: float,
    distal_hole: bool,
    plate_mesh,
    plate_material: Material,
    metal: Material,
    elastomer: Material | None = None,
) -> None:
    for side, y in (("side_0", plate_y), ("side_1", -plate_y)):
        part.visual(
            plate_mesh,
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=plate_material,
            name=f"{side}_plate",
        )

    _add_pin(part, plate_y=plate_y, material=metal)

    rail_length = max(0.020, length - 0.046)
    part.visual(
        Box((rail_length, plate_y * 2.05, 0.010)),
        origin=Origin(xyz=(length * 0.5, 0.0, -0.004)),
        material=metal,
        name="center_spine",
    )

    # Two cross spacers hold the transparent side plates as one rigid phalanx.
    for index, x in enumerate((length * 0.36, length * 0.52)):
        part.visual(
            Cylinder(radius=0.0045, length=plate_y * 2.35),
            origin=Origin(xyz=(x, 0.0, -0.014), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name=f"spacer_{index}",
        )

    if elastomer is not None:
        part.visual(
            Box((0.020, plate_y * 2.30, max(0.012, distal_height * 0.50))),
            origin=Origin(xyz=(length + 0.010, 0.0, -0.002)),
            material=elastomer,
            name="tip_pad",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="finger_phalanx_chain")

    clear_acrylic = model.material("clear_acrylic", rgba=(0.58, 0.86, 1.0, 0.38))
    smoke_acrylic = model.material("smoke_acrylic", rgba=(0.45, 0.63, 0.78, 0.45))
    brushed_metal = model.material("brushed_metal", rgba=(0.72, 0.73, 0.70, 1.0))
    dark_polymer = model.material("dark_polymer", rgba=(0.035, 0.038, 0.045, 1.0))
    fingertip_rubber = model.material("fingertip_rubber", rgba=(0.03, 0.03, 0.035, 1.0))

    base_plate_mesh = _side_plate_mesh(
        length=0.030,
        proximal_height=0.082,
        distal_height=0.068,
        distal_hole=False,
        name="base_side_plate",
    )
    proximal_plate_mesh = _side_plate_mesh(
        length=PROXIMAL_LENGTH,
        proximal_height=0.046,
        distal_height=0.038,
        distal_hole=True,
        name="proximal_side_plate",
    )
    middle_plate_mesh = _side_plate_mesh(
        length=MIDDLE_LENGTH,
        proximal_height=0.039,
        distal_height=0.032,
        distal_hole=True,
        name="middle_side_plate",
    )
    distal_plate_mesh = _side_plate_mesh(
        length=DISTAL_LENGTH,
        proximal_height=0.032,
        distal_height=0.024,
        distal_hole=False,
        name="distal_side_plate",
    )

    base = model.part("base_knuckle")
    base.visual(
        Box((0.135, 0.092, 0.030)),
        origin=Origin(xyz=(-0.044, 0.0, -0.060)),
        material=dark_polymer,
        name="palm_mount",
    )
    base.visual(
        Box((0.052, 0.086, 0.018)),
        origin=Origin(xyz=(-0.004, 0.0, -0.043)),
        material=dark_polymer,
        name="knuckle_bridge",
    )
    for side, y in (("side_0", OUTER_PLATE_Y + 0.002), ("side_1", -OUTER_PLATE_Y - 0.002)):
        base.visual(
            base_plate_mesh,
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=smoke_acrylic,
            name=f"{side}_cheek",
        )

    proximal = model.part("proximal_phalanx")
    _add_phalanx(
        proximal,
        length=PROXIMAL_LENGTH,
        proximal_height=0.046,
        distal_height=0.038,
        plate_y=INNER_PLATE_Y,
        distal_hole=True,
        plate_mesh=proximal_plate_mesh,
        plate_material=clear_acrylic,
        metal=brushed_metal,
    )

    middle = model.part("middle_phalanx")
    _add_phalanx(
        middle,
        length=MIDDLE_LENGTH,
        proximal_height=0.039,
        distal_height=0.032,
        plate_y=OUTER_PLATE_Y,
        distal_hole=True,
        plate_mesh=middle_plate_mesh,
        plate_material=clear_acrylic,
        metal=brushed_metal,
    )

    distal = model.part("distal_phalanx")
    _add_phalanx(
        distal,
        length=DISTAL_LENGTH,
        proximal_height=0.032,
        distal_height=0.024,
        plate_y=INNER_PLATE_Y,
        distal_hole=False,
        plate_mesh=distal_plate_mesh,
        plate_material=clear_acrylic,
        metal=brushed_metal,
        elastomer=fingertip_rubber,
    )

    flex_axis = (0.0, -1.0, 0.0)
    model.articulation(
        "base_joint",
        ArticulationType.REVOLUTE,
        parent=base,
        child=proximal,
        origin=Origin(),
        axis=flex_axis,
        motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=0.0, upper=math.pi / 2.0),
    )
    model.articulation(
        "middle_joint",
        ArticulationType.REVOLUTE,
        parent=proximal,
        child=middle,
        origin=Origin(xyz=(PROXIMAL_LENGTH, 0.0, 0.0)),
        axis=flex_axis,
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=math.radians(75.0)),
    )
    model.articulation(
        "distal_joint",
        ArticulationType.REVOLUTE,
        parent=middle,
        child=distal,
        origin=Origin(xyz=(MIDDLE_LENGTH, 0.0, 0.0)),
        axis=flex_axis,
        motion_limits=MotionLimits(effort=5.0, velocity=2.5, lower=0.0, upper=math.radians(75.0)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_joint = object_model.get_articulation("base_joint")
    middle_joint = object_model.get_articulation("middle_joint")
    distal_joint = object_model.get_articulation("distal_joint")
    proximal = object_model.get_part("proximal_phalanx")
    middle = object_model.get_part("middle_phalanx")
    distal = object_model.get_part("distal_phalanx")

    for parent, child, plates, joint_label in (
        ("base_knuckle", "proximal_phalanx", ("side_0_cheek", "side_1_cheek"), "base"),
        ("proximal_phalanx", "middle_phalanx", ("side_0_plate", "side_1_plate"), "middle"),
        ("middle_phalanx", "distal_phalanx", ("side_0_plate", "side_1_plate"), "distal"),
    ):
        for plate_name in plates:
            ctx.allow_overlap(
                parent,
                child,
                elem_a=plate_name,
                elem_b="proximal_pin",
                reason="The metal hinge pin is intentionally captured through the clear side-plate bore.",
            )
            ctx.expect_overlap(
                child,
                parent,
                axes="xz",
                elem_a="proximal_pin",
                elem_b=plate_name,
                min_overlap=0.010,
                name=f"{joint_label} pin crosses {plate_name}",
            )

    ctx.check(
        "three planar flexion joints",
        len(object_model.articulations) == 3
        and all(tuple(j.axis) == (0.0, -1.0, 0.0) for j in object_model.articulations),
        details="The chain should have exactly three revolute joints sharing a common Y-axis bend plane.",
    )
    ctx.check(
        "base joint bends ninety degrees",
        math.isclose(base_joint.motion_limits.lower, 0.0)
        and math.isclose(base_joint.motion_limits.upper, math.pi / 2.0),
        details=f"limits={base_joint.motion_limits}",
    )
    ctx.check(
        "distal joints bend seventy five degrees",
        all(
            math.isclose(j.motion_limits.lower, 0.0)
            and math.isclose(j.motion_limits.upper, math.radians(75.0))
            for j in (middle_joint, distal_joint)
        ),
        details=f"middle={middle_joint.motion_limits}, distal={distal_joint.motion_limits}",
    )

    with ctx.pose({base_joint: 0.0, middle_joint: 0.0, distal_joint: 0.0}):
        ctx.expect_origin_gap(
            middle,
            proximal,
            axis="x",
            min_gap=PROXIMAL_LENGTH - 0.001,
            max_gap=PROXIMAL_LENGTH + 0.001,
            name="straight chain keeps second joint at proximal length",
        )
        straight_tip = ctx.part_world_position(distal)

    with ctx.pose({base_joint: math.pi / 2.0, middle_joint: math.radians(75.0), distal_joint: math.radians(75.0)}):
        flexed_tip = ctx.part_world_position(distal)

    ctx.check(
        "positive joint motion curls in one vertical plane",
        straight_tip is not None
        and flexed_tip is not None
        and flexed_tip[2] > straight_tip[2] + 0.08
        and abs(flexed_tip[1] - straight_tip[1]) < 0.002,
        details=f"straight={straight_tip}, flexed={flexed_tip}",
    )

    return ctx.report()


object_model = build_object_model()
