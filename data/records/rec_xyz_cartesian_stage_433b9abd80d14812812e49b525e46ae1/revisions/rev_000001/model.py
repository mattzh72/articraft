from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_three_axis_positioning_stack")

    graphite = model.material("graphite", rgba=(0.10, 0.11, 0.12, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.20, 0.21, 0.22, 1.0))
    ground_steel = model.material("ground_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    blue_anodized = model.material("blue_anodized", rgba=(0.05, 0.20, 0.46, 1.0))
    black_oxide = model.material("black_oxide", rgba=(0.02, 0.025, 0.03, 1.0))
    brass = model.material("brass", rgba=(0.78, 0.56, 0.22, 1.0))
    red_plastic = model.material("red_plastic", rgba=(0.72, 0.05, 0.03, 1.0))

    lower_frame = model.part("lower_frame")
    lower_frame.visual(
        Box((0.86, 0.40, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=graphite,
        name="base_plate",
    )
    for y in (-0.160, 0.160):
        lower_frame.visual(
            Box((0.80, 0.045, 0.060)),
            origin=Origin(xyz=(0.0, y, 0.070)),
            material=dark_steel,
            name=f"side_beam_{0 if y < 0 else 1}",
        )
        lower_frame.visual(
            Box((0.78, 0.018, 0.018)),
            origin=Origin(xyz=(0.0, y, 0.104)),
            material=ground_steel,
            name=f"x_way_{0 if y < 0 else 1}",
        )
    for x in (-0.400, 0.400):
        lower_frame.visual(
            Box((0.020, 0.36, 0.060)),
            origin=Origin(xyz=(x, 0.0, 0.070)),
            material=dark_steel,
            name=f"end_tie_{0 if x < 0 else 1}",
        )
    for x in (-0.34, 0.34):
        for y in (-0.145, 0.145):
            lower_frame.visual(
                Box((0.075, 0.050, 0.016)),
                origin=Origin(xyz=(x, y, -0.008)),
                material=black_oxide,
                name=f"foot_{0 if x < 0 else 1}_{0 if y < 0 else 1}",
            )

    x_carriage = model.part("x_carriage")
    x_carriage.visual(
        Box((0.420, 0.340, 0.045)),
        origin=Origin(),
        material=blue_anodized,
        name="x_saddle_plate",
    )
    for x in (-0.120, 0.120):
        x_carriage.visual(
            Box((0.026, 0.300, 0.022)),
            origin=Origin(xyz=(x, 0.0, 0.0335)),
            material=ground_steel,
            name=f"y_way_{0 if x < 0 else 1}",
        )
    x_carriage.visual(
        Box((0.030, 0.300, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.0285)),
        material=black_oxide,
        name="center_dust_cover",
    )
    for x in (-0.185, 0.185):
        x_carriage.visual(
            Box((0.018, 0.315, 0.025)),
            origin=Origin(xyz=(x, 0.0, 0.015)),
            material=dark_steel,
            name=f"saddle_side_lip_{0 if x < 0 else 1}",
        )

    plate_profile = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.290, 0.240, 0.018, corner_segments=8),
        [rounded_rect_profile(0.110, 0.110, 0.010, corner_segments=6)],
        height=0.035,
        center=True,
    )
    y_slide = model.part("y_slide")
    y_slide.visual(
        mesh_from_geometry(plate_profile, "y_slide_aperture_plate"),
        origin=Origin(),
        material=blue_anodized,
        name="aperture_plate",
    )
    y_slide.visual(
        Box((0.020, 0.120, 0.205)),
        origin=Origin(xyz=(-0.061, 0.0, 0.120)),
        material=black_oxide,
        name="guide_x_0",
    )
    y_slide.visual(
        Box((0.020, 0.120, 0.205)),
        origin=Origin(xyz=(0.061, 0.0, 0.120)),
        material=black_oxide,
        name="guide_x_1",
    )
    for y in (-0.061, 0.061):
        y_slide.visual(
            Box((0.122, 0.020, 0.205)),
            origin=Origin(xyz=(0.0, y, 0.120)),
            material=black_oxide,
            name=f"guide_y_{0 if y < 0 else 1}",
        )
    y_slide.visual(
        Box((0.030, 0.160, 0.022)),
        origin=Origin(xyz=(-0.062, 0.0, 0.2335)),
        material=black_oxide,
        name="top_collar_x_0",
    )
    y_slide.visual(
        Box((0.030, 0.160, 0.022)),
        origin=Origin(xyz=(0.062, 0.0, 0.2335)),
        material=black_oxide,
        name="top_collar_x_1",
    )
    y_slide.visual(
        Box((0.160, 0.030, 0.022)),
        origin=Origin(xyz=(0.0, -0.062, 0.2335)),
        material=black_oxide,
        name="top_collar_y_0",
    )
    y_slide.visual(
        Box((0.160, 0.030, 0.022)),
        origin=Origin(xyz=(0.0, 0.062, 0.2335)),
        material=black_oxide,
        name="top_collar_y_1",
    )
    y_slide.visual(
        Box((0.180, 0.025, 0.018)),
        origin=Origin(xyz=(0.0, -0.1325, 0.000)),
        material=ground_steel,
        name="front_scale_strip",
    )
    for x in (-0.095, 0.095):
        y_slide.visual(
            Box((0.020, 0.035, 0.030)),
            origin=Origin(xyz=(x, -0.1325, 0.006)),
            material=red_plastic,
            name=f"travel_stop_{0 if x < 0 else 1}",
        )

    ram = model.part("ram")
    ram.visual(
        Box((0.074, 0.074, 0.300)),
        origin=Origin(),
        material=ground_steel,
        name="ram_column",
    )
    for x in (-0.043, 0.043):
        ram.visual(
            Box((0.012, 0.052, 0.090)),
            origin=Origin(xyz=(x, 0.0, -0.020)),
            material=brass,
            name=f"gib_pad_x_{0 if x < 0 else 1}",
        )
    for y in (-0.043, 0.043):
        ram.visual(
            Box((0.052, 0.012, 0.090)),
            origin=Origin(xyz=(0.0, y, 0.055)),
            material=brass,
            name=f"gib_pad_y_{0 if y < 0 else 1}",
        )
    ram.visual(
        Box((0.100, 0.100, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.1625)),
        material=black_oxide,
        name="top_cap",
    )
    ram.visual(
        Box((0.090, 0.090, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, -0.1620)),
        material=brass,
        name="tool_pad",
    )

    model.articulation(
        "x_axis",
        ArticulationType.PRISMATIC,
        parent=lower_frame,
        child=x_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.1355)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.30, lower=-0.160, upper=0.160),
    )
    model.articulation(
        "y_axis",
        ArticulationType.PRISMATIC,
        parent=x_carriage,
        child=y_slide,
        origin=Origin(xyz=(0.0, 0.0, 0.0620)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=65.0, velocity=0.25, lower=-0.075, upper=0.075),
    )
    model.articulation(
        "z_axis",
        ArticulationType.PRISMATIC,
        parent=y_slide,
        child=ram,
        origin=Origin(xyz=(0.0, 0.0, 0.1650)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.18, lower=0.0, upper=0.100),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_frame = object_model.get_part("lower_frame")
    x_carriage = object_model.get_part("x_carriage")
    y_slide = object_model.get_part("y_slide")
    ram = object_model.get_part("ram")
    x_axis = object_model.get_articulation("x_axis")
    y_axis = object_model.get_articulation("y_axis")
    z_axis = object_model.get_articulation("z_axis")

    ctx.check(
        "three serial prismatic axes",
        all(j.articulation_type == ArticulationType.PRISMATIC for j in (x_axis, y_axis, z_axis)),
        details="The stack should be lower frame -> X carriage -> Y slide -> Z ram with prismatic joints.",
    )
    ctx.check(
        "axis directions are orthogonal xyz",
        x_axis.axis == (1.0, 0.0, 0.0)
        and y_axis.axis == (0.0, 1.0, 0.0)
        and z_axis.axis == (0.0, 0.0, 1.0),
        details=f"axes: x={x_axis.axis}, y={y_axis.axis}, z={z_axis.axis}",
    )

    ctx.expect_gap(
        x_carriage,
        lower_frame,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="x carriage is seated on lower X ways",
    )
    ctx.expect_overlap(
        x_carriage,
        lower_frame,
        axes="xy",
        min_overlap=0.25,
        name="x carriage footprint remains over the lower frame",
    )
    ctx.expect_gap(
        y_slide,
        x_carriage,
        axis="z",
        max_gap=0.001,
        max_penetration=0.000001,
        name="y slide is seated on carriage Y ways",
    )
    ctx.expect_gap(
        y_slide,
        ram,
        axis="x",
        min_gap=0.006,
        max_gap=0.020,
        positive_elem="top_collar_x_1",
        negative_elem="ram_column",
        name="ram column clears the positive X collar cheek",
    )
    ctx.expect_gap(
        ram,
        y_slide,
        axis="x",
        min_gap=0.006,
        max_gap=0.020,
        positive_elem="ram_column",
        negative_elem="top_collar_x_0",
        name="ram column clears the negative X collar cheek",
    )
    ctx.expect_gap(
        y_slide,
        ram,
        axis="y",
        min_gap=0.006,
        max_gap=0.020,
        positive_elem="top_collar_y_1",
        negative_elem="ram_column",
        name="ram column clears the positive Y collar cheek",
    )
    ctx.expect_gap(
        ram,
        y_slide,
        axis="y",
        min_gap=0.006,
        max_gap=0.020,
        positive_elem="ram_column",
        negative_elem="top_collar_y_0",
        name="ram column clears the negative Y collar cheek",
    )
    ctx.expect_overlap(
        ram,
        y_slide,
        axes="z",
        min_overlap=0.18,
        elem_a="ram_column",
        elem_b="guide_x_0",
        name="ram has retained insertion in the guide at rest",
    )

    x_rest = ctx.part_world_position(x_carriage)
    y_rest = ctx.part_world_position(y_slide)
    z_rest = ctx.part_world_position(ram)
    with ctx.pose({x_axis: 0.160}):
        x_extended = ctx.part_world_position(x_carriage)
        ctx.expect_overlap(
            x_carriage,
            lower_frame,
            axes="x",
            min_overlap=0.20,
            name="x carriage remains supported at positive X travel",
        )
    with ctx.pose({y_axis: 0.075}):
        y_extended = ctx.part_world_position(y_slide)
        ctx.expect_overlap(
            y_slide,
            x_carriage,
            axes="y",
            min_overlap=0.11,
            name="y slide remains supported at positive Y travel",
        )
    with ctx.pose({z_axis: 0.100}):
        z_extended = ctx.part_world_position(ram)
        ctx.expect_overlap(
            ram,
            y_slide,
            axes="z",
            min_overlap=0.09,
            elem_a="ram_column",
            elem_b="guide_x_0",
            name="ram stays inserted at full Z extension",
        )

    ctx.check(
        "positive X command moves the first carriage along X",
        x_rest is not None and x_extended is not None and x_extended[0] > x_rest[0] + 0.12,
        details=f"rest={x_rest}, extended={x_extended}",
    )
    ctx.check(
        "positive Y command moves the cross-slide along Y",
        y_rest is not None and y_extended is not None and y_extended[1] > y_rest[1] + 0.05,
        details=f"rest={y_rest}, extended={y_extended}",
    )
    ctx.check(
        "positive Z command raises the ram",
        z_rest is not None and z_extended is not None and z_extended[2] > z_rest[2] + 0.08,
        details=f"rest={z_rest}, extended={z_extended}",
    )

    return ctx.report()


object_model = build_object_model()
