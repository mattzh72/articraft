from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


PALM_WIDTH = 0.100
PALM_DEPTH = 0.105
PALM_THICKNESS = 0.026
PALM_FRONT_Y = PALM_DEPTH / 2.0
FINGER_ROOT_Y = PALM_FRONT_Y + 0.0075
FINGER_ROOT_Z = PALM_THICKNESS + 0.0055
FINGER_ROOT_XS = (-0.036, -0.012, 0.012, 0.036)

FINGER_PROXIMAL_LENGTH = 0.052
FINGER_MIDDLE_LENGTH = 0.036
FINGER_DISTAL_LENGTH = 0.027
FINGER_WIDTHS = (0.013, 0.015, 0.015, 0.013)
FINGER_HEIGHT = 0.011

THUMB_ROOT_X = -PALM_WIDTH / 2.0 - 0.0075
THUMB_ROOT_Y = -0.010
THUMB_ROOT_Z = PALM_THICKNESS * 0.58
THUMB_YAW = 0.74


def _x_axis_cylinder_origin(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0))


def _add_finger_segment(
    part,
    *,
    length: float,
    width: float,
    height: float,
    link_material,
    pin_material,
    distal_barrel: bool,
    tip: bool = False,
) -> None:
    root_radius = height * 0.62
    distal_radius = height * 0.58 if distal_barrel else 0.0
    body_start = root_radius
    body_end = max(body_start + 0.001, length - distal_radius)
    body_length = body_end - body_start
    part.visual(
        Cylinder(radius=root_radius, length=width + 0.004),
        origin=_x_axis_cylinder_origin((0.0, 0.0, 0.0)),
        material=pin_material,
        name="root_pin",
    )
    part.visual(
        Box((width, body_length, height)),
        origin=Origin(xyz=(0.0, body_start + body_length * 0.5, 0.0)),
        material=link_material,
        name="link_body",
    )
    part.visual(
        Box((width * 0.62, body_length, height * 0.28)),
        origin=Origin(xyz=(0.0, body_start + body_length * 0.5, height * 0.60)),
        material=pin_material,
        name="top_rib",
    )
    if distal_barrel:
        part.visual(
            Cylinder(radius=distal_radius, length=width + 0.002),
            origin=_x_axis_cylinder_origin((0.0, length, 0.0)),
            material=pin_material,
            name="distal_barrel",
        )
    if tip:
        part.visual(
            Sphere(radius=height * 0.58),
            origin=Origin(xyz=(0.0, length + height * 0.35, 0.0)),
            material=link_material,
            name="rounded_tip",
        )


def _add_thumb_segment(
    part,
    *,
    length: float,
    width: float,
    height: float,
    link_material,
    pin_material,
    distal_barrel: bool,
    tip: bool = False,
) -> None:
    root_radius = height * 0.62
    distal_radius = height * 0.56 if distal_barrel else 0.0
    body_start = root_radius
    body_end = max(body_start + 0.001, length - distal_radius)
    body_length = body_end - body_start
    part.visual(
        Cylinder(radius=root_radius, length=height * 1.8),
        origin=Origin(),
        material=pin_material,
        name="root_pin",
    )
    part.visual(
        Box((width, body_length, height)),
        origin=Origin(xyz=(0.0, body_start + body_length * 0.5, 0.0)),
        material=link_material,
        name="link_body",
    )
    if distal_barrel:
        part.visual(
            Cylinder(radius=distal_radius, length=width + 0.003),
            origin=_x_axis_cylinder_origin((0.0, length, 0.0)),
            material=pin_material,
            name="distal_barrel",
        )
    if tip:
        part.visual(
            Sphere(radius=height * 0.62),
            origin=Origin(xyz=(0.0, length + height * 0.30, 0.0)),
            material=link_material,
            name="rounded_tip",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="robotic_palm")

    palm_shell = model.material("brushed_aluminum", rgba=(0.64, 0.66, 0.68, 1.0))
    dark_hardware = model.material("dark_hardware", rgba=(0.08, 0.09, 0.10, 1.0))
    graphite_link = model.material("graphite_link", rgba=(0.17, 0.18, 0.20, 1.0))
    bearing_steel = model.material("bearing_steel", rgba=(0.50, 0.53, 0.56, 1.0))
    rubber_pad = model.material("black_rubber", rgba=(0.025, 0.025, 0.027, 1.0))

    palm = model.part("palm")
    palm.visual(
        Box((PALM_WIDTH, PALM_DEPTH, PALM_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, PALM_THICKNESS / 2.0)),
        material=palm_shell,
        name="palm_block",
    )
    palm.visual(
        Box((PALM_WIDTH * 0.84, PALM_DEPTH * 0.56, 0.003)),
        origin=Origin(xyz=(0.0, -0.006, PALM_THICKNESS + 0.0015)),
        material=dark_hardware,
        name="top_cover",
    )
    palm.visual(
        Box((PALM_WIDTH * 0.78, 0.006, 0.006)),
        origin=Origin(xyz=(0.0, PALM_FRONT_Y + 0.002, PALM_THICKNESS + 0.003)),
        material=dark_hardware,
        name="front_bus_bar",
    )

    for index, root_x in enumerate(FINGER_ROOT_XS):
        palm.visual(
            Box((0.019, 0.017, 0.014)),
            origin=Origin(xyz=(root_x, PALM_FRONT_Y + 0.004, FINGER_ROOT_Z - 0.0126)),
            material=palm_shell,
            name=f"finger_{index}_saddle",
        )
        palm.visual(
            Cylinder(radius=0.0068, length=0.019),
            origin=_x_axis_cylinder_origin((root_x, FINGER_ROOT_Y, FINGER_ROOT_Z)),
            material=bearing_steel,
            name=f"finger_{index}_socket",
        )

    palm.visual(
        Box((0.016, 0.034, 0.018)),
        origin=Origin(xyz=(-PALM_WIDTH / 2.0 - 0.002, THUMB_ROOT_Y - 0.024, THUMB_ROOT_Z)),
        material=palm_shell,
        name="thumb_side_mount",
    )
    palm.visual(
        Cylinder(radius=0.0072, length=0.021),
        origin=Origin(xyz=(THUMB_ROOT_X, THUMB_ROOT_Y, THUMB_ROOT_Z)),
        material=bearing_steel,
        name="thumb_socket",
    )

    for corner_x in (-0.036, 0.036):
        for corner_y in (-0.030, 0.026):
            palm.visual(
                Cylinder(radius=0.0033, length=0.0022),
                origin=Origin(xyz=(corner_x, corner_y, PALM_THICKNESS + 0.0032)),
                material=dark_hardware,
                name=f"cover_screw_{corner_x}_{corner_y}",
            )

    for index, root_x in enumerate(FINGER_ROOT_XS):
        width = FINGER_WIDTHS[index]
        proximal = model.part(f"finger_{index}_proximal")
        _add_finger_segment(
            proximal,
            length=FINGER_PROXIMAL_LENGTH,
            width=width,
            height=FINGER_HEIGHT,
            link_material=graphite_link,
            pin_material=bearing_steel,
            distal_barrel=True,
        )
        model.articulation(
            f"finger_{index}_root",
            ArticulationType.REVOLUTE,
            parent=palm,
            child=proximal,
            origin=Origin(xyz=(root_x, FINGER_ROOT_Y, FINGER_ROOT_Z)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=0.0, upper=1.25),
        )

        middle = model.part(f"finger_{index}_middle")
        _add_finger_segment(
            middle,
            length=FINGER_MIDDLE_LENGTH,
            width=width * 0.88,
            height=FINGER_HEIGHT * 0.92,
            link_material=graphite_link,
            pin_material=bearing_steel,
            distal_barrel=True,
        )
        model.articulation(
            f"finger_{index}_middle_joint",
            ArticulationType.REVOLUTE,
            parent=proximal,
            child=middle,
            origin=Origin(xyz=(0.0, FINGER_PROXIMAL_LENGTH, 0.0)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.2, velocity=3.0, lower=0.0, upper=1.05),
        )

        distal = model.part(f"finger_{index}_distal")
        _add_finger_segment(
            distal,
            length=FINGER_DISTAL_LENGTH,
            width=width * 0.76,
            height=FINGER_HEIGHT * 0.84,
            link_material=graphite_link,
            pin_material=bearing_steel,
            distal_barrel=False,
            tip=True,
        )
        distal.visual(
            Box((width * 0.62, 0.010, 0.0025)),
            origin=Origin(xyz=(0.0, FINGER_DISTAL_LENGTH + 0.002, -FINGER_HEIGHT * 0.46)),
            material=rubber_pad,
            name="finger_pad",
        )
        model.articulation(
            f"finger_{index}_tip_joint",
            ArticulationType.REVOLUTE,
            parent=middle,
            child=distal,
            origin=Origin(xyz=(0.0, FINGER_MIDDLE_LENGTH, 0.0)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.5, velocity=3.0, lower=0.0, upper=0.85),
        )

    thumb_base = model.part("thumb_base")
    _add_thumb_segment(
        thumb_base,
        length=0.040,
        width=0.015,
        height=0.012,
        link_material=graphite_link,
        pin_material=bearing_steel,
        distal_barrel=True,
    )
    model.articulation(
        "thumb_base_joint",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=thumb_base,
        origin=Origin(xyz=(THUMB_ROOT_X, THUMB_ROOT_Y, THUMB_ROOT_Z), rpy=(0.0, 0.0, THUMB_YAW)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.2, lower=-0.75, upper=0.75),
    )

    thumb_middle = model.part("thumb_middle")
    _add_finger_segment(
        thumb_middle,
        length=0.032,
        width=0.013,
        height=0.010,
        link_material=graphite_link,
        pin_material=bearing_steel,
        distal_barrel=True,
    )
    model.articulation(
        "thumb_middle_joint",
        ArticulationType.REVOLUTE,
        parent=thumb_base,
        child=thumb_middle,
        origin=Origin(xyz=(0.0, 0.040, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.8, velocity=3.0, lower=0.0, upper=1.0),
    )

    thumb_tip = model.part("thumb_tip")
    _add_finger_segment(
        thumb_tip,
        length=0.024,
        width=0.011,
        height=0.009,
        link_material=graphite_link,
        pin_material=bearing_steel,
        distal_barrel=False,
        tip=True,
    )
    thumb_tip.visual(
        Box((0.008, 0.010, 0.0025)),
        origin=Origin(xyz=(0.0, 0.024, -0.004)),
        material=rubber_pad,
        name="thumb_pad",
    )
    model.articulation(
        "thumb_tip_joint",
        ArticulationType.REVOLUTE,
        parent=thumb_middle,
        child=thumb_tip,
        origin=Origin(xyz=(0.0, 0.032, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=3.0, lower=0.0, upper=0.85),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    palm = object_model.get_part("palm")

    finger_roots = [object_model.get_articulation(f"finger_{index}_root") for index in range(4)]
    ctx.check(
        "four independent finger roots",
        all(joint is not None and joint.articulation_type == ArticulationType.REVOLUTE for joint in finger_roots),
        details=f"roots={finger_roots!r}",
    )
    ctx.check(
        "thumb base has side revolute",
        object_model.get_articulation("thumb_base_joint") is not None
        and object_model.get_articulation("thumb_base_joint").articulation_type == ArticulationType.REVOLUTE,
        details="Expected a separate side-mounted thumb_base_joint.",
    )

    root_positions = [ctx.part_world_position(object_model.get_part(f"finger_{index}_proximal")) for index in range(4)]
    if all(position is not None for position in root_positions):
        xs = [position[0] for position in root_positions]
        spacings = [xs[index + 1] - xs[index] for index in range(3)]
        ctx.check(
            "finger roots evenly spaced",
            max(spacings) - min(spacings) < 0.0005,
            details=f"xs={xs!r}, spacings={spacings!r}",
        )
        ctx.check(
            "finger roots on front edge",
            all(abs(position[1] - FINGER_ROOT_Y) < 0.001 for position in root_positions),
            details=f"positions={root_positions!r}",
        )
    else:
        ctx.fail("finger root positions available", f"positions={root_positions!r}")

    for index in range(4):
        proximal = object_model.get_part(f"finger_{index}_proximal")
        middle = object_model.get_part(f"finger_{index}_middle")
        distal = object_model.get_part(f"finger_{index}_distal")
        ctx.allow_overlap(
            palm,
            proximal,
            elem_a=f"finger_{index}_socket",
            elem_b="root_pin",
            reason="Each finger root pin is intentionally captured inside its palm-front bearing socket.",
        )
        ctx.expect_within(
            proximal,
            palm,
            axes="x",
            inner_elem="root_pin",
            outer_elem=f"finger_{index}_socket",
            margin=0.001,
            name=f"finger_{index}_root_pin_seated",
        )
        ctx.expect_overlap(
            proximal,
            palm,
            axes="x",
            elem_a="root_pin",
            elem_b=f"finger_{index}_socket",
            min_overlap=0.012,
            name=f"finger_{index}_root_pin_retained",
        )

        ctx.allow_overlap(
            proximal,
            middle,
            elem_a="distal_barrel",
            elem_b="root_pin",
            reason="The middle phalanx hinge pin is intentionally captured in the proximal barrel.",
        )
        ctx.expect_overlap(
            proximal,
            middle,
            axes="x",
            elem_a="distal_barrel",
            elem_b="root_pin",
            min_overlap=0.010,
            name=f"finger_{index}_middle_pin_retained",
        )

        ctx.allow_overlap(
            middle,
            distal,
            elem_a="distal_barrel",
            elem_b="root_pin",
            reason="The distal phalanx hinge pin is intentionally captured in the middle barrel.",
        )
        ctx.expect_overlap(
            middle,
            distal,
            axes="x",
            elem_a="distal_barrel",
            elem_b="root_pin",
            min_overlap=0.009,
            name=f"finger_{index}_tip_pin_retained",
        )

    thumb_base = object_model.get_part("thumb_base")
    thumb_middle = object_model.get_part("thumb_middle")
    thumb_tip = object_model.get_part("thumb_tip")
    ctx.allow_overlap(
        palm,
        thumb_base,
        elem_a="thumb_socket",
        elem_b="root_pin",
        reason="The side thumb pivot pin is intentionally captured in the palm-side bearing socket.",
    )
    ctx.expect_overlap(
        thumb_base,
        palm,
        axes="z",
        elem_a="root_pin",
        elem_b="thumb_socket",
        min_overlap=0.015,
        name="thumb_base_pin_retained",
    )
    ctx.allow_overlap(
        thumb_base,
        thumb_middle,
        elem_a="distal_barrel",
        elem_b="root_pin",
        reason="The thumb middle hinge pin is intentionally captured in the thumb base barrel.",
    )
    ctx.allow_overlap(
        thumb_base,
        thumb_middle,
        elem_a="distal_barrel",
        elem_b="link_body",
        reason="The rectangular thumb link root seats locally into the simplified hinge barrel cheek.",
    )
    ctx.expect_overlap(
        thumb_base,
        thumb_middle,
        axes="x",
        elem_a="distal_barrel",
        elem_b="root_pin",
        min_overlap=0.009,
        name="thumb_middle_pin_retained",
    )
    ctx.expect_gap(
        thumb_middle,
        thumb_base,
        axis="y",
        positive_elem="link_body",
        negative_elem="distal_barrel",
        max_penetration=0.012,
        name="thumb_middle_barrel_seat_is_local",
    )
    ctx.allow_overlap(
        thumb_middle,
        thumb_tip,
        elem_a="distal_barrel",
        elem_b="root_pin",
        reason="The thumb tip hinge pin is intentionally captured in the thumb middle barrel.",
    )
    ctx.allow_overlap(
        thumb_middle,
        thumb_tip,
        elem_a="distal_barrel",
        elem_b="link_body",
        reason="The thumb tip link root seats locally into the simplified hinge barrel cheek.",
    )
    ctx.expect_overlap(
        thumb_middle,
        thumb_tip,
        axes="x",
        elem_a="distal_barrel",
        elem_b="root_pin",
        min_overlap=0.008,
        name="thumb_tip_pin_retained",
    )
    ctx.expect_gap(
        thumb_tip,
        thumb_middle,
        axis="y",
        positive_elem="link_body",
        negative_elem="distal_barrel",
        max_penetration=0.010,
        name="thumb_tip_barrel_seat_is_local",
    )

    first_root = object_model.get_articulation("finger_0_root")
    thumb_joint = object_model.get_articulation("thumb_base_joint")
    rest_first = ctx.part_world_position(object_model.get_part("finger_0_distal"))
    rest_thumb = ctx.part_world_position(thumb_tip)
    with ctx.pose({first_root: 0.55, thumb_joint: 0.55}):
        curled_first = ctx.part_world_position(object_model.get_part("finger_0_distal"))
        rotated_thumb = ctx.part_world_position(thumb_tip)
    ctx.check(
        "finger root flexes its chain",
        rest_first is not None and curled_first is not None and curled_first[2] < rest_first[2] - 0.010,
        details=f"rest={rest_first!r}, curled={curled_first!r}",
    )
    ctx.check(
        "thumb base rotates sideways",
        rest_thumb is not None and rotated_thumb is not None and abs(rotated_thumb[0] - rest_thumb[0]) > 0.015,
        details=f"rest={rest_thumb!r}, rotated={rotated_thumb!r}",
    )

    return ctx.report()


object_model = build_object_model()
