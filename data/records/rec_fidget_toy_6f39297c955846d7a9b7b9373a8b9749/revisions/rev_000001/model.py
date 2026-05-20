from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


SEGMENT_COUNT = 8
PITCH = 0.042
SECTION = 0.026
BODY_GAP = 0.0085
BARREL_RADIUS = 0.0050
PIN_RADIUS = 0.0026
PIN_INTERFERENCE = 0.00035
PIN_TOTAL_LENGTH = 0.034
CENTER_KNUCKLE_LENGTH = 0.012
KNUCKLE_CLEARANCE = 0.001
OUTER_KNUCKLE_LENGTH = (PIN_TOTAL_LENGTH - CENTER_KNUCKLE_LENGTH - 2.0 * KNUCKLE_CLEARANCE) / 2.0
OUTER_KNUCKLE_OFFSET = CENTER_KNUCKLE_LENGTH / 2.0 + KNUCKLE_CLEARANCE + OUTER_KNUCKLE_LENGTH / 2.0
PIN_HEAD_THICKNESS = 0.0018
PIN_HEAD_RADIUS = 0.0039


def _axis_tuple(axis_name: str) -> tuple[float, float, float]:
    if axis_name == "z":
        return (0.0, 0.0, 1.0)
    if axis_name == "y":
        return (0.0, 1.0, 0.0)
    raise ValueError(f"unsupported axis {axis_name!r}")


def _axis_rpy(axis_name: str) -> tuple[float, float, float]:
    # SDK/URDF cylinders are local-Z aligned; rotate local +Z onto +Y when needed.
    if axis_name == "z":
        return (0.0, 0.0, 0.0)
    if axis_name == "y":
        return (-math.pi / 2.0, 0.0, 0.0)
    raise ValueError(f"unsupported axis {axis_name!r}")


def _box_solid(center: tuple[float, float, float], size: tuple[float, float, float]) -> cq.Shape:
    return cq.Workplane("XY").box(size[0], size[1], size[2]).translate(center).val()


def _cylinder_solid(
    center: tuple[float, float, float],
    axis_name: str,
    radius: float,
    length: float,
) -> cq.Shape:
    axis = _axis_tuple(axis_name)
    start = (
        center[0] - axis[0] * length / 2.0,
        center[1] - axis[1] * length / 2.0,
        center[2] - axis[2] * length / 2.0,
    )
    return cq.Solid.makeCylinder(radius, length, cq.Vector(*start), cq.Vector(*axis))


def _fuse_all(solids: list[cq.Shape]) -> cq.Shape:
    fused = solids[0]
    for solid in solids[1:]:
        fused = fused.fuse(solid)
    return fused.clean()


def _web_box(
    x_center: float,
    x_length: float,
    axis_name: str,
    axis_center: float,
    axis_length: float,
) -> cq.Shape:
    web_width = BARREL_RADIUS * 2.15
    if axis_name == "z":
        return _box_solid((x_center, 0.0, axis_center), (x_length, web_width, axis_length))
    return _box_solid((x_center, axis_center, 0.0), (x_length, axis_length, web_width))


def _barrel_center(x: float, axis_name: str, axis_center: float) -> tuple[float, float, float]:
    if axis_name == "z":
        return (x, 0.0, axis_center)
    return (x, axis_center, 0.0)


def _segment_geometry(input_axis: str | None, output_axis: str | None) -> cq.Shape:
    """Build one square-section plastic segment with hollow interleaved hinge knuckles."""

    x_min = 0.0 if input_axis is None else BODY_GAP
    x_max = PITCH if output_axis is None else PITCH - BODY_GAP
    solids: list[cq.Shape] = [
        _box_solid(
            ((x_min + x_max) / 2.0, 0.0, 0.0),
            (x_max - x_min, SECTION, SECTION),
        )
    ]

    web_length = BODY_GAP + 0.001
    if input_axis is not None:
        solids.append(
            _cylinder_solid(
                _barrel_center(0.0, input_axis, 0.0),
                input_axis,
                BARREL_RADIUS,
                CENTER_KNUCKLE_LENGTH,
            )
        )
        solids.append(_web_box(BODY_GAP / 2.0, web_length, input_axis, 0.0, CENTER_KNUCKLE_LENGTH))

    if output_axis is not None:
        for sign in (-1.0, 1.0):
            axis_center = sign * OUTER_KNUCKLE_OFFSET
            solids.append(
                _cylinder_solid(
                    _barrel_center(PITCH, output_axis, axis_center),
                    output_axis,
                    BARREL_RADIUS,
                    OUTER_KNUCKLE_LENGTH,
                )
            )
            solids.append(
                _web_box(
                    PITCH - BODY_GAP / 2.0,
                    web_length,
                    output_axis,
                    axis_center,
                    OUTER_KNUCKLE_LENGTH,
                )
            )

    shape = _fuse_all(solids)
    # The visible pin is a separate primitive seated into this bore.  A very
    # small interference keeps the modeled pin mechanically captured rather
    # than appearing to float inside an oversized mesh hole.
    hole_radius = PIN_RADIUS - PIN_INTERFERENCE
    hole_length = PIN_TOTAL_LENGTH + 0.006
    if input_axis is not None:
        shape = shape.cut(_cylinder_solid(_barrel_center(0.0, input_axis, 0.0), input_axis, hole_radius, hole_length))
    if output_axis is not None:
        shape = shape.cut(_cylinder_solid(_barrel_center(PITCH, output_axis, 0.0), output_axis, hole_radius, hole_length))
    return shape.clean()


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="eight_segment_snake_fidget_chain")

    pin_material = model.material("dark_steel_pin", rgba=(0.08, 0.08, 0.075, 1.0))
    colors = [
        model.material("red_plastic", rgba=(0.88, 0.08, 0.08, 1.0)),
        model.material("orange_plastic", rgba=(0.98, 0.38, 0.06, 1.0)),
        model.material("yellow_plastic", rgba=(0.96, 0.78, 0.08, 1.0)),
        model.material("green_plastic", rgba=(0.08, 0.62, 0.24, 1.0)),
        model.material("cyan_plastic", rgba=(0.05, 0.55, 0.80, 1.0)),
        model.material("blue_plastic", rgba=(0.08, 0.18, 0.78, 1.0)),
        model.material("violet_plastic", rgba=(0.45, 0.12, 0.72, 1.0)),
        model.material("magenta_plastic", rgba=(0.85, 0.10, 0.58, 1.0)),
    ]

    hinge_axes = ["z" if i % 2 == 0 else "y" for i in range(SEGMENT_COUNT - 1)]

    segments = []
    for i in range(SEGMENT_COUNT):
        segment = model.part(f"segment_{i}")
        input_axis = hinge_axes[i - 1] if i > 0 else None
        output_axis = hinge_axes[i] if i < SEGMENT_COUNT - 1 else None
        segment.visual(
            mesh_from_cadquery(
                _segment_geometry(input_axis, output_axis),
                f"segment_{i}_plastic",
                tolerance=0.00045,
                angular_tolerance=0.08,
            ),
            material=colors[i],
            name="plastic_shell",
        )
        segments.append(segment)

    for i, axis_name in enumerate(hinge_axes):
        parent = segments[i]
        child = segments[i + 1]

        shaft_length = PIN_TOTAL_LENGTH + 2.0 * PIN_HEAD_THICKNESS + 0.0008
        parent.visual(
            Cylinder(radius=PIN_RADIUS, length=shaft_length),
            origin=Origin(xyz=(PITCH, 0.0, 0.0), rpy=_axis_rpy(axis_name)),
            material=pin_material,
            name="output_pin",
        )
        for sign, cap_name in ((-1.0, "head_0"), (1.0, "head_1")):
            offset = sign * (PIN_TOTAL_LENGTH / 2.0 + PIN_HEAD_THICKNESS / 2.0 + 0.0002)
            xyz = (PITCH, 0.0, offset) if axis_name == "z" else (PITCH, offset, 0.0)
            parent.visual(
                Cylinder(radius=PIN_HEAD_RADIUS, length=PIN_HEAD_THICKNESS),
                origin=Origin(xyz=xyz, rpy=_axis_rpy(axis_name)),
                material=pin_material,
                name=cap_name,
            )

        joint_axis = (0.0, 0.0, 1.0) if axis_name == "z" else (0.0, -1.0, 0.0)
        model.articulation(
            f"hinge_{i}",
            ArticulationType.REVOLUTE,
            parent=parent,
            child=child,
            origin=Origin(xyz=(PITCH, 0.0, 0.0)),
            axis=joint_axis,
            motion_limits=MotionLimits(
                effort=0.6,
                velocity=6.0,
                lower=-math.pi / 2.0,
                upper=math.pi / 2.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    segments = [object_model.get_part(f"segment_{i}") for i in range(SEGMENT_COUNT)]
    hinges = [object_model.get_articulation(f"hinge_{i}") for i in range(SEGMENT_COUNT - 1)]

    ctx.check("eight square-section segments", len(segments) == 8)
    ctx.check(
        "seven revolute hinge joints",
        len(hinges) == 7 and all(hinge.articulation_type == ArticulationType.REVOLUTE for hinge in hinges),
    )

    for i, hinge in enumerate(hinges):
        axis_name = "z" if i % 2 == 0 else "y"
        radial_axes = "xy" if axis_name == "z" else "xz"
        ctx.allow_overlap(
            segments[i],
            segments[i + 1],
            elem_a="output_pin",
            elem_b="plastic_shell",
            reason=(
                "The dark hinge pin is intentionally modeled with a tiny seated "
                "interference through the child knuckle bore so the chain reads "
                "as physically captured rather than floating."
            ),
        )
        ctx.expect_overlap(
            segments[i],
            segments[i + 1],
            axes="x",
            min_overlap=BARREL_RADIUS * 1.4,
            name=f"interleaved knuckles share hinge station {i}",
        )
        ctx.expect_within(
            segments[i],
            segments[i + 1],
            axes=radial_axes,
            inner_elem="output_pin",
            outer_elem="plastic_shell",
            margin=0.001,
            name=f"pin {i} runs through child knuckle bore",
        )
        ctx.expect_overlap(
            segments[i],
            segments[i + 1],
            axes=axis_name,
            elem_a="output_pin",
            elem_b="plastic_shell",
            min_overlap=CENTER_KNUCKLE_LENGTH * 0.75,
            name=f"pin {i} is retained along hinge axis",
        )
        ctx.check(
            f"hinge {i} has quarter-turn fidget limits",
            hinge.motion_limits is not None
            and hinge.motion_limits.lower is not None
            and hinge.motion_limits.upper is not None
            and hinge.motion_limits.lower <= -math.pi / 2.0
            and hinge.motion_limits.upper >= math.pi / 2.0,
        )

    segment_2 = object_model.get_part("segment_2")
    rest_pos_2 = ctx.part_world_position(segment_2)
    with ctx.pose({hinges[0]: math.pi / 2.0}):
        folded_pos_2 = ctx.part_world_position(segment_2)
    ctx.check(
        "first hinge folds the chain sideways",
        rest_pos_2 is not None and folded_pos_2 is not None and folded_pos_2[1] > rest_pos_2[1] + PITCH * 0.75,
        details=f"rest={rest_pos_2}, folded={folded_pos_2}",
    )

    segment_3 = object_model.get_part("segment_3")
    rest_pos_3 = ctx.part_world_position(segment_3)
    with ctx.pose({hinges[1]: math.pi / 2.0}):
        twisted_pos_3 = ctx.part_world_position(segment_3)
    ctx.check(
        "second hinge twists the chain upward",
        rest_pos_3 is not None and twisted_pos_3 is not None and twisted_pos_3[2] > rest_pos_3[2] + PITCH * 0.75,
        details=f"rest={rest_pos_3}, twisted={twisted_pos_3}",
    )

    return ctx.report()


object_model = build_object_model()
