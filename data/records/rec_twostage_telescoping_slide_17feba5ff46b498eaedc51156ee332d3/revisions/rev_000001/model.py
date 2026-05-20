from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


OUTER_LENGTH = 0.460
OUTER_DEPTH = 0.016
OUTER_HEIGHT = 0.045
OUTER_WALL = 0.0015
OUTER_LIP_DEPTH = 0.0018
OUTER_LIP_HEIGHT = 0.008

MIDDLE_LENGTH = 0.380
MIDDLE_DEPTH = 0.0112
MIDDLE_HEIGHT = 0.036
MIDDLE_WALL = 0.0014
MIDDLE_LIP_DEPTH = 0.0015
MIDDLE_LIP_HEIGHT = 0.007

OUTPUT_LENGTH = 0.300
OUTPUT_DEPTH = 0.0070
OUTPUT_HEIGHT = 0.026
OUTPUT_WALL = 0.0012
OUTPUT_LIP_DEPTH = 0.0013
OUTPUT_LIP_HEIGHT = 0.005

OUTER_TO_MIDDLE_INSERT = 0.035
OUTER_TO_MIDDLE_TRAVEL = 0.180
MIDDLE_TO_OUTPUT_INSERT = 0.030
MIDDLE_TO_OUTPUT_TRAVEL = 0.150

OUTER_TO_MIDDLE_Z = 0.0045
MIDDLE_TO_OUTPUT_Z = 0.0050
def _box_spec(
    name: str,
    size_xyz: tuple[float, float, float],
    center_xyz: tuple[float, float, float],
) -> tuple[str, tuple[float, float, float], tuple[float, float, float]]:
    return name, size_xyz, center_xyz


def _rail_spec(
    *,
    name: str,
    x_start: float,
    length: float,
    y_min: float,
    y_max: float,
    z_center: float,
    height: float,
) -> tuple[str, tuple[float, float, float], tuple[float, float, float]]:
    return _box_spec(
        name,
        (length, y_max - y_min, height),
        (x_start + length / 2.0, (y_min + y_max) / 2.0, z_center),
    )


def _channel_specs(
    *,
    prefix: str,
    length: float,
    depth: float,
    height: float,
    wall: float,
    lip_depth: float,
    lip_height: float,
    open_side: int,
) -> list[tuple[str, tuple[float, float, float], tuple[float, float, float]]]:
    closed_side_y = -open_side * (depth / 2.0 - wall / 2.0)
    lip_y = open_side * (depth / 2.0 - lip_depth / 2.0)
    return [
        _box_spec(f"{prefix}_web", (length, wall, height), (length / 2.0, closed_side_y, height / 2.0)),
        _box_spec(f"{prefix}_bottom_flange", (length, depth, wall), (length / 2.0, 0.0, wall / 2.0)),
        _box_spec(f"{prefix}_top_flange", (length, depth, wall), (length / 2.0, 0.0, height - wall / 2.0)),
        _box_spec(
            f"{prefix}_lower_lip",
            (length, lip_depth, lip_height),
            (length / 2.0, lip_y, wall + lip_height / 2.0),
        ),
        _box_spec(
            f"{prefix}_upper_lip",
            (length, lip_depth, lip_height),
            (length / 2.0, lip_y, height - wall - lip_height / 2.0),
        ),
    ]


def _outer_specs() -> list[tuple[str, tuple[float, float, float], tuple[float, float, float]]]:
    specs = _channel_specs(
        prefix="outer",
        length=OUTER_LENGTH,
        depth=OUTER_DEPTH,
        height=OUTER_HEIGHT,
        wall=OUTER_WALL,
        lip_depth=OUTER_LIP_DEPTH,
        lip_height=OUTER_LIP_HEIGHT,
        open_side=1,
    )
    specs.append(
        _box_spec("outer_mount_pad", (0.030, OUTER_DEPTH, 0.014), (0.020, 0.0, 0.016))
    )
    return specs


def _middle_specs() -> list[tuple[str, tuple[float, float, float], tuple[float, float, float]]]:
    outer_lip_inner_y = OUTER_DEPTH / 2.0 - OUTER_LIP_DEPTH
    rail_depth = 0.0010
    specs = _channel_specs(
        prefix="middle",
        length=MIDDLE_LENGTH,
        depth=MIDDLE_DEPTH,
        height=MIDDLE_HEIGHT,
        wall=MIDDLE_WALL,
        lip_depth=MIDDLE_LIP_DEPTH,
        lip_height=MIDDLE_LIP_HEIGHT,
        open_side=-1,
    )
    specs.extend(
        [
            _rail_spec(
                name="middle_lower_rail_front",
                x_start=0.080,
                length=0.070,
                y_min=outer_lip_inner_y - rail_depth,
                y_max=outer_lip_inner_y,
                z_center=0.0052,
                height=0.0022,
            ),
            _rail_spec(
                name="middle_lower_rail_rear",
                x_start=0.210,
                length=0.070,
                y_min=outer_lip_inner_y - rail_depth,
                y_max=outer_lip_inner_y,
                z_center=0.0052,
                height=0.0022,
            ),
            _rail_spec(
                name="middle_upper_rail_front",
                x_start=0.080,
                length=0.070,
                y_min=outer_lip_inner_y - rail_depth,
                y_max=outer_lip_inner_y,
                z_center=0.0308,
                height=0.0022,
            ),
            _rail_spec(
                name="middle_upper_rail_rear",
                x_start=0.210,
                length=0.070,
                y_min=outer_lip_inner_y - rail_depth,
                y_max=outer_lip_inner_y,
                z_center=0.0308,
                height=0.0022,
            ),
        ]
    )
    return specs


def _output_specs() -> list[tuple[str, tuple[float, float, float], tuple[float, float, float]]]:
    middle_lip_inner_y = -(MIDDLE_DEPTH / 2.0 - MIDDLE_LIP_DEPTH)
    rail_depth = 0.0010
    specs = _channel_specs(
        prefix="output",
        length=OUTPUT_LENGTH,
        depth=OUTPUT_DEPTH,
        height=OUTPUT_HEIGHT,
        wall=OUTPUT_WALL,
        lip_depth=OUTPUT_LIP_DEPTH,
        lip_height=OUTPUT_LIP_HEIGHT,
        open_side=1,
    )
    specs.extend(
        [
            _rail_spec(
                name="output_lower_rail_front",
                x_start=0.070,
                length=0.060,
                y_min=middle_lip_inner_y,
                y_max=middle_lip_inner_y + rail_depth,
                z_center=0.0038,
                height=0.0020,
            ),
            _rail_spec(
                name="output_lower_rail_rear",
                x_start=0.160,
                length=0.060,
                y_min=middle_lip_inner_y,
                y_max=middle_lip_inner_y + rail_depth,
                z_center=0.0038,
                height=0.0020,
            ),
            _rail_spec(
                name="output_upper_rail_front",
                x_start=0.070,
                length=0.060,
                y_min=middle_lip_inner_y,
                y_max=middle_lip_inner_y + rail_depth,
                z_center=0.0230,
                height=0.0020,
            ),
            _rail_spec(
                name="output_upper_rail_rear",
                x_start=0.160,
                length=0.060,
                y_min=middle_lip_inner_y,
                y_max=middle_lip_inner_y + rail_depth,
                z_center=0.0230,
                height=0.0020,
            ),
            _box_spec(
                "output_front_tab",
                (0.010, OUTPUT_DEPTH, OUTPUT_HEIGHT * 0.52),
                (OUTPUT_LENGTH - 0.005, 0.0, 0.013),
            ),
        ]
    )
    return specs


def _add_box_visuals(
    part,
    *,
    specs: list[tuple[str, tuple[float, float, float], tuple[float, float, float]]],
    material: str,
) -> None:
    for name, size_xyz, center_xyz in specs:
        part.visual(
            Box(size_xyz),
            origin=Origin(xyz=center_xyz),
            material=material,
            name=name,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nested_runner_two_stage_extension")

    model.material("outer_zinc", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("middle_steel", rgba=(0.56, 0.59, 0.63, 1.0))
    model.material("output_steel", rgba=(0.80, 0.82, 0.85, 1.0))

    outer = model.part("outer_section")
    _add_box_visuals(outer, specs=_outer_specs(), material="outer_zinc")
    outer.inertial = Inertial.from_geometry(
        Box((OUTER_LENGTH, OUTER_DEPTH, OUTER_HEIGHT)),
        mass=0.55,
        origin=Origin(xyz=(OUTER_LENGTH / 2.0, 0.0, OUTER_HEIGHT / 2.0)),
    )

    middle = model.part("middle_runner")
    _add_box_visuals(middle, specs=_middle_specs(), material="middle_steel")
    middle.inertial = Inertial.from_geometry(
        Box((MIDDLE_LENGTH, MIDDLE_DEPTH, MIDDLE_HEIGHT)),
        mass=0.36,
        origin=Origin(xyz=(MIDDLE_LENGTH / 2.0, 0.0, MIDDLE_HEIGHT / 2.0)),
    )

    output = model.part("output_runner")
    _add_box_visuals(output, specs=_output_specs(), material="output_steel")
    output.inertial = Inertial.from_geometry(
        Box((OUTPUT_LENGTH, OUTPUT_DEPTH, OUTPUT_HEIGHT)),
        mass=0.24,
        origin=Origin(xyz=(OUTPUT_LENGTH / 2.0, 0.0, OUTPUT_HEIGHT / 2.0)),
    )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(xyz=(OUTER_TO_MIDDLE_INSERT, 0.0, OUTER_TO_MIDDLE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=OUTER_TO_MIDDLE_TRAVEL,
            effort=140.0,
            velocity=0.45,
        ),
    )
    model.articulation(
        "middle_to_output",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=output,
        origin=Origin(xyz=(MIDDLE_TO_OUTPUT_INSERT, 0.0, MIDDLE_TO_OUTPUT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=MIDDLE_TO_OUTPUT_TRAVEL,
            effort=100.0,
            velocity=0.50,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    outer = object_model.get_part("outer_section")
    middle = object_model.get_part("middle_runner")
    output = object_model.get_part("output_runner")
    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_output = object_model.get_articulation("middle_to_output")

    ctx.expect_within(
        middle,
        outer,
        axes="yz",
        margin=0.0,
        name="middle runner stays captured in outer section cross-section",
    )
    ctx.expect_within(
        output,
        middle,
        axes="yz",
        margin=0.0,
        name="output runner stays captured in middle runner cross-section",
    )
    ctx.expect_overlap(
        middle,
        outer,
        axes="x",
        min_overlap=0.240,
        name="middle runner has clear retained overlap in outer section",
    )
    ctx.expect_overlap(
        output,
        middle,
        axes="x",
        min_overlap=0.220,
        name="output runner has clear retained overlap in middle runner",
    )

    middle_rest = ctx.part_world_position(middle)
    output_rest = ctx.part_world_position(output)
    with ctx.pose({outer_to_middle: OUTER_TO_MIDDLE_TRAVEL}):
        ctx.expect_within(
            middle,
            outer,
            axes="yz",
            margin=0.0,
            name="extended middle runner stays centered in outer section",
        )
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            min_overlap=0.240,
            name="extended middle runner still overlaps outer section",
        )
        middle_carried = ctx.part_world_position(middle)
        output_carried = ctx.part_world_position(output)

    with ctx.pose(
        {
            outer_to_middle: OUTER_TO_MIDDLE_TRAVEL,
            middle_to_output: MIDDLE_TO_OUTPUT_TRAVEL,
        }
    ):
        ctx.expect_within(
            output,
            middle,
            axes="yz",
            margin=0.0,
            name="fully extended output runner stays centered in middle runner",
        )
        ctx.expect_within(
            output,
            outer,
            axes="yz",
            margin=0.0,
            name="fully extended output runner stays within outer section profile",
        )
        ctx.expect_overlap(
            output,
            middle,
            axes="x",
            min_overlap=0.190,
            name="fully extended output runner still overlaps middle runner",
        )
        ctx.expect_overlap(
            output,
            outer,
            axes="x",
            min_overlap=0.060,
            name="fully extended output runner still overlaps outer section",
        )
        output_full = ctx.part_world_position(output)

    ctx.check(
        "middle runner extends along +X",
        middle_rest is not None
        and middle_carried is not None
        and middle_carried[0] > middle_rest[0] + 0.15,
        details=f"rest={middle_rest}, carried={middle_carried}",
    )
    ctx.check(
        "output runner is carried by middle stage",
        output_rest is not None
        and output_carried is not None
        and output_carried[0] > output_rest[0] + 0.15,
        details=f"rest={output_rest}, carried={output_carried}",
    )
    ctx.check(
        "output runner extends farther on second stage",
        output_carried is not None
        and output_full is not None
        and output_full[0] > output_carried[0] + 0.12,
        details=f"carried={output_carried}, full={output_full}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
