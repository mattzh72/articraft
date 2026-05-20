from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


LINK_SPACING = 0.32
LINK_EYE_RADIUS = 0.034
LINK_HOLE_RADIUS = 0.010
LINK_THICKNESS = 0.012
LOWER_LAYER_Z = -0.006
UPPER_LAYER_Z = 0.006


def _circle_profile(
    center: tuple[float, float],
    radius: float,
    *,
    segments: int = 32,
    clockwise: bool = False,
) -> list[tuple[float, float]]:
    angles = range(segments - 1, -1, -1) if clockwise else range(segments)
    cx, cy = center
    return [
        (
            cx + radius * cos(2.0 * pi * index / segments),
            cy + radius * sin(2.0 * pi * index / segments),
        )
        for index in angles
    ]


def _capsule_profile(length: float, radius: float, *, segments_per_end: int = 18) -> list[tuple[float, float]]:
    """Rounded two-eye flat bar outline with hinge centers at x=0 and x=length."""
    points: list[tuple[float, float]] = [(0.0, -radius), (length, -radius)]
    for index in range(segments_per_end + 1):
        theta = -pi / 2.0 + pi * index / segments_per_end
        points.append((length + radius * cos(theta), radius * sin(theta)))
    points.append((0.0, radius))
    for index in range(segments_per_end + 1):
        theta = pi / 2.0 + pi * index / segments_per_end
        points.append((radius * cos(theta), radius * sin(theta)))
    return points


def _link_plate_mesh(name: str):
    outer = _capsule_profile(LINK_SPACING, LINK_EYE_RADIUS)
    holes = [
        _circle_profile((0.0, 0.0), LINK_HOLE_RADIUS),
        _circle_profile((LINK_SPACING, 0.0), LINK_HOLE_RADIUS),
    ]
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(outer, holes, LINK_THICKNESS, center=True),
        name,
    )


def _pin_visuals(part, *, joint_x: float, layer_z: float, steel, prefix: str) -> None:
    """A shouldered pivot pin fixed to the parent link and ending in the inter-plate clearance."""
    head_thickness = 0.006
    head_radius = 0.018
    shaft_radius = 0.006
    shaft_length = 0.026
    head_sign = -1.0 if layer_z < 0.0 else 1.0
    plate_outer_face = layer_z + head_sign * (LINK_THICKNESS * 0.5)
    head_center_z = plate_outer_face + head_sign * (head_thickness * 0.5 - 0.0005)

    part.visual(
        Cylinder(radius=shaft_radius, length=shaft_length),
        origin=Origin(xyz=(joint_x, 0.0, head_sign * shaft_length * 0.5)),
        material=steel,
        name=f"{prefix}_pin_shaft",
    )
    part.visual(
        Cylinder(radius=head_radius, length=head_thickness),
        origin=Origin(xyz=(joint_x, 0.0, head_center_z)),
        material=steel,
        name=f"{prefix}_pin_head",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="five_bar_serial_linkage")

    painted_steel = model.material("painted_steel", rgba=(0.18, 0.36, 0.68, 1.0))
    second_paint = model.material("second_paint", rgba=(0.86, 0.38, 0.14, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    bright_steel = model.material("bright_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    base_finish = model.material("base_finish", rgba=(0.10, 0.11, 0.12, 1.0))

    link_meshes = [_link_plate_mesh(f"bar_{index}_plate") for index in range(5)]
    layers = [LOWER_LAYER_Z, UPPER_LAYER_Z, LOWER_LAYER_Z, UPPER_LAYER_Z, LOWER_LAYER_Z]

    root_bar = model.part("root_bar")
    root_bar.visual(
        Box((0.23, 0.17, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, -0.058)),
        material=base_finish,
        name="base_foot",
    )
    root_bar.visual(
        Cylinder(radius=0.028, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, -0.032)),
        material=dark_steel,
        name="base_pedestal",
    )
    root_bar.visual(
        Cylinder(radius=0.024, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, LOWER_LAYER_Z - LINK_THICKNESS * 0.5 - 0.002)),
        material=bright_steel,
        name="base_clamp_washer",
    )
    root_bar.visual(
        link_meshes[0],
        origin=Origin(xyz=(0.0, 0.0, layers[0])),
        material=painted_steel,
        name="bar_plate",
    )
    _pin_visuals(root_bar, joint_x=LINK_SPACING, layer_z=layers[0], steel=bright_steel, prefix="joint_0")

    bars = [root_bar]
    for index in range(1, 5):
        bar = model.part(f"bar_{index}")
        bar.visual(
            link_meshes[index],
            origin=Origin(xyz=(0.0, 0.0, layers[index])),
            material=painted_steel if index % 2 == 0 else second_paint,
            name="bar_plate",
        )
        if index < 4:
            _pin_visuals(
                bar,
                joint_x=LINK_SPACING,
                layer_z=layers[index],
                steel=bright_steel,
                prefix=f"joint_{index}",
            )
        else:
            bar.visual(
                Cylinder(radius=0.017, length=0.006),
                origin=Origin(
                    xyz=(
                        LINK_SPACING,
                        0.0,
                        layers[index] + (-1.0 if layers[index] < 0.0 else 1.0) * 0.009,
                    )
                ),
                material=bright_steel,
                name="free_end_washer",
            )
        bars.append(bar)

    for index in range(4):
        model.articulation(
            f"hinge_{index}",
            ArticulationType.REVOLUTE,
            parent=bars[index],
            child=bars[index + 1],
            origin=Origin(xyz=(LINK_SPACING, 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=-1.35, upper=1.35),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bars = [object_model.get_part("root_bar")] + [object_model.get_part(f"bar_{index}") for index in range(1, 5)]
    hinges = [object_model.get_articulation(f"hinge_{index}") for index in range(4)]

    ctx.check("five rigid bars", len(bars) == 5, f"bar_count={len(bars)}")
    ctx.check("four revolute hinges", len(hinges) == 4, f"hinge_count={len(hinges)}")
    for hinge in hinges:
        ctx.check(
            f"{hinge.name} axis parallel",
            tuple(round(float(v), 6) for v in hinge.axis) == (0.0, 0.0, 1.0),
            f"axis={hinge.axis!r}",
        )

    rest_origins = [ctx.part_world_position(bar) for bar in bars]
    for index in range(1, 5):
        prev_origin = rest_origins[index - 1]
        this_origin = rest_origins[index]
        ctx.check(
            f"bar_{index} hinge spacing",
            prev_origin is not None
            and this_origin is not None
            and abs((this_origin[0] - prev_origin[0]) - LINK_SPACING) < 1.0e-6
            and abs(this_origin[1] - prev_origin[1]) < 1.0e-6
            and abs(this_origin[2] - prev_origin[2]) < 1.0e-6,
            f"prev={prev_origin}, this={this_origin}",
        )

    for index in range(4):
        ctx.expect_overlap(
            bars[index],
            bars[index + 1],
            axes="xy",
            elem_a="bar_plate",
            elem_b="bar_plate",
            min_overlap=0.040,
            name=f"hinge_{index} plates share pivot footprint",
        )
        upper_bar, lower_bar = (bars[index + 1], bars[index]) if index % 2 == 0 else (bars[index], bars[index + 1])
        ctx.expect_gap(
            upper_bar,
            lower_bar,
            axis="z",
            positive_elem="bar_plate",
            negative_elem="bar_plate",
            max_penetration=0.0,
            name=f"hinge_{index} plates are vertically clear",
        )

    rest_tip = ctx.part_world_position(bars[-1])
    with ctx.pose({hinges[0]: 0.65, hinges[1]: -0.45, hinges[2]: 0.35, hinges[3]: -0.25}):
        moved_tip = ctx.part_world_position(bars[-1])
    ctx.check(
        "serial chain tip moves in plane",
        rest_tip is not None
        and moved_tip is not None
        and abs(moved_tip[2] - rest_tip[2]) < 1.0e-6
        and abs(moved_tip[1] - rest_tip[1]) > 0.04,
        f"rest_tip={rest_tip}, moved_tip={moved_tip}",
    )

    return ctx.report()


object_model = build_object_model()
