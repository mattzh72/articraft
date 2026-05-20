from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="double_panel_shutter")

    painted_frame = model.material("painted_frame", rgba=(0.94, 0.93, 0.89, 1.0))
    shutter_paint = model.material("shutter_paint", rgba=(0.88, 0.87, 0.82, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.54, 0.55, 0.57, 1.0))

    outer_width = 0.580
    outer_height = 1.520
    frame_depth = 0.050
    jamb_width = 0.055
    rail_height = 0.055
    clear_width = outer_width - 2.0 * jamb_width
    clear_height = outer_height - 2.0 * rail_height

    panel_depth = 0.032
    panel_x_min = 0.012
    panel_x_max = 0.440
    panel_width = panel_x_max - panel_x_min
    panel_height = 1.402
    panel_half_height = panel_height * 0.5
    hinge_axis_x = -0.223

    stile_width = 0.048
    top_rail_height = 0.080
    bottom_rail_height = 0.080
    divider_height = 0.065
    upper_opening_bottom = 0.101
    upper_opening_top = panel_half_height - top_rail_height
    upper_opening_height = upper_opening_top - upper_opening_bottom
    left_opening_x = panel_x_min + stile_width
    right_opening_x = panel_x_max - stile_width
    opening_width = right_opening_x - left_opening_x
    opening_center_x = 0.5 * (left_opening_x + right_opening_x)

    louver_count = 4
    louver_pitch = upper_opening_height / louver_count
    louver_blade_length = opening_width - 0.014
    louver_pin_length = 0.007
    louver_chord = 0.026
    louver_thickness = 0.008

    def add_box_feature(
        part,
        *,
        size: tuple[float, float, float],
        xyz: tuple[float, float, float],
        material,
        name: str,
        rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> None:
        part.visual(
            Box(size),
            origin=Origin(xyz=xyz, rpy=rpy),
            material=material,
            name=name,
        )

    def build_louver_mesh(length: float, chord: float, thickness: float):
        profile = [
            (0.0000, chord * 0.50),
            (thickness * 0.48, chord * 0.18),
            (thickness * 0.32, -chord * 0.18),
            (0.0000, -chord * 0.50),
            (-thickness * 0.32, -chord * 0.18),
            (-thickness * 0.48, chord * 0.18),
        ]
        geom = ExtrudeGeometry.centered(profile, length, cap=True, closed=True)
        geom.rotate_y(math.pi / 2.0)
        return mesh_from_geometry(geom, "shutter_louver_blade")

    louver_mesh = build_louver_mesh(
        louver_blade_length,
        louver_chord,
        louver_thickness,
    )

    frame = model.part("frame")
    add_box_feature(
        frame,
        size=(jamb_width, frame_depth, outer_height),
        xyz=(-outer_width * 0.5 + jamb_width * 0.5, 0.0, 0.0),
        material=painted_frame,
        name="left_jamb",
    )
    add_box_feature(
        frame,
        size=(jamb_width, frame_depth, outer_height),
        xyz=(outer_width * 0.5 - jamb_width * 0.5, 0.0, 0.0),
        material=painted_frame,
        name="right_jamb",
    )
    add_box_feature(
        frame,
        size=(outer_width, frame_depth, rail_height),
        xyz=(0.0, 0.0, outer_height * 0.5 - rail_height * 0.5),
        material=painted_frame,
        name="head",
    )
    add_box_feature(
        frame,
        size=(outer_width, frame_depth, rail_height),
        xyz=(0.0, 0.0, -outer_height * 0.5 + rail_height * 0.5),
        material=painted_frame,
        name="sill",
    )
    add_box_feature(
        frame,
        size=(0.020, 0.006, 0.180),
        xyz=(hinge_axis_x - 0.010, -0.010, 0.425),
        material=hinge_metal,
        name="frame_upper_leaf",
    )
    add_box_feature(
        frame,
        size=(0.020, 0.006, 0.180),
        xyz=(hinge_axis_x - 0.010, -0.010, -0.425),
        material=hinge_metal,
        name="frame_lower_leaf",
    )
    frame.visual(
        Cylinder(radius=0.008, length=0.180),
        origin=Origin(xyz=(hinge_axis_x, 0.0, 0.425)),
        material=hinge_metal,
        name="frame_upper_knuckle",
    )
    frame.visual(
        Cylinder(radius=0.008, length=0.180),
        origin=Origin(xyz=(hinge_axis_x, 0.0, -0.425)),
        material=hinge_metal,
        name="frame_lower_knuckle",
    )
    frame.inertial = Inertial.from_geometry(
        Box((outer_width, frame_depth, outer_height)),
        mass=5.4,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    shutter_panel = model.part("shutter_panel")
    add_box_feature(
        shutter_panel,
        size=(stile_width, panel_depth, panel_height),
        xyz=(panel_x_min + stile_width * 0.5, 0.0, 0.0),
        material=shutter_paint,
        name="left_stile",
    )
    add_box_feature(
        shutter_panel,
        size=(stile_width, panel_depth, panel_height),
        xyz=(panel_x_max - stile_width * 0.5, 0.0, 0.0),
        material=shutter_paint,
        name="right_stile",
    )
    add_box_feature(
        shutter_panel,
        size=(panel_width, panel_depth, top_rail_height),
        xyz=(0.5 * (panel_x_min + panel_x_max), 0.0, panel_half_height - top_rail_height * 0.5),
        material=shutter_paint,
        name="top_rail",
    )
    add_box_feature(
        shutter_panel,
        size=(panel_width, panel_depth, bottom_rail_height),
        xyz=(0.5 * (panel_x_min + panel_x_max), 0.0, -panel_half_height + bottom_rail_height * 0.5),
        material=shutter_paint,
        name="bottom_rail",
    )
    add_box_feature(
        shutter_panel,
        size=(panel_width, panel_depth, divider_height),
        xyz=(0.5 * (panel_x_min + panel_x_max), 0.0, upper_opening_bottom - divider_height * 0.5),
        material=shutter_paint,
        name="divider_rail",
    )

    lower_panel_top = upper_opening_bottom - divider_height
    lower_panel_bottom = -panel_half_height + bottom_rail_height
    lower_panel_height = lower_panel_top - lower_panel_bottom
    add_box_feature(
        shutter_panel,
        size=(opening_width - 0.030, 0.010, lower_panel_height - 0.030),
        xyz=(
            opening_center_x,
            0.004,
            0.5 * (lower_panel_top + lower_panel_bottom),
        ),
        material=shutter_paint,
        name="lower_solid_panel",
    )
    add_box_feature(
        shutter_panel,
        size=(opening_width + 0.006, 0.012, lower_panel_height + 0.006),
        xyz=(
            opening_center_x,
            -0.005,
            0.5 * (lower_panel_top + lower_panel_bottom),
        ),
        material=shutter_paint,
        name="lower_panel_backer",
    )

    add_box_feature(
        shutter_panel,
        size=(0.024, 0.006, 0.180),
        xyz=(0.012, 0.011, 0.425),
        material=hinge_metal,
        name="panel_upper_leaf",
    )
    add_box_feature(
        shutter_panel,
        size=(0.024, 0.006, 0.180),
        xyz=(0.012, 0.011, -0.425),
        material=hinge_metal,
        name="panel_lower_leaf",
    )
    shutter_panel.inertial = Inertial.from_geometry(
        Box((panel_width, panel_depth, panel_height)),
        mass=3.0,
        origin=Origin(xyz=(0.5 * (panel_x_min + panel_x_max), 0.0, 0.0)),
    )

    model.articulation(
        "frame_to_shutter",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=shutter_panel,
        origin=Origin(xyz=(hinge_axis_x, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.4,
            lower=0.0,
            upper=math.pi / 2.0,
        ),
    )

    louver_zs = [
        upper_opening_bottom + louver_pitch * (index + 0.5)
        for index in range(louver_count)
    ]
    for index, louver_z in enumerate(louver_zs, start=1):
        add_box_feature(
            shutter_panel,
            size=(0.006, 0.012, 0.022),
            xyz=(left_opening_x - 0.003, 0.0, louver_z),
            material=shutter_paint,
            name=f"louver_{index}_left_bearing",
        )
        add_box_feature(
            shutter_panel,
            size=(0.006, 0.012, 0.022),
            xyz=(right_opening_x + 0.003, 0.0, louver_z),
            material=shutter_paint,
            name=f"louver_{index}_right_bearing",
        )
        louver = model.part(f"upper_louver_{index}")
        louver.visual(
            louver_mesh,
            material=shutter_paint,
            name="blade",
        )
        pin_offset = opening_width * 0.5 - louver_pin_length * 0.5
        louver.visual(
            Cylinder(radius=0.0035, length=louver_pin_length),
            origin=Origin(
                xyz=(-pin_offset, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=hinge_metal,
            name="left_pivot",
        )
        louver.visual(
            Cylinder(radius=0.0035, length=louver_pin_length),
            origin=Origin(
                xyz=(pin_offset, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=hinge_metal,
            name="right_pivot",
        )
        louver.inertial = Inertial.from_geometry(
            Box((louver_blade_length + louver_pin_length, louver_chord, louver_thickness)),
            mass=0.12,
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
        )
        model.articulation(
            f"shutter_to_upper_louver_{index}",
            ArticulationType.REVOLUTE,
            parent=shutter_panel,
            child=louver,
            origin=Origin(xyz=(opening_center_x, 0.0, louver_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.0,
                velocity=2.0,
                lower=-0.85,
                upper=0.55,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    frame = object_model.get_part("frame")
    shutter_panel = object_model.get_part("shutter_panel")
    shutter_hinge = object_model.get_articulation("frame_to_shutter")
    clear_width = 0.580 - 2.0 * 0.055
    clear_height = 1.520 - 2.0 * 0.055
    panel_width = 0.440 - 0.012
    panel_height = 1.402
    louvers = [object_model.get_part(f"upper_louver_{index}") for index in range(1, 5)]
    louver_joints = [
        object_model.get_articulation(f"shutter_to_upper_louver_{index}")
        for index in range(1, 5)
    ]

    shutter_limits = shutter_hinge.motion_limits
    ctx.check(
        "shutter hinge uses vertical side-hinge axis",
        shutter_hinge.axis == (0.0, 0.0, 1.0)
        and shutter_limits is not None
        and shutter_limits.lower == 0.0
        and shutter_limits.upper is not None
        and shutter_limits.upper >= 1.4,
        details=f"axis={shutter_hinge.axis}, limits={shutter_limits}",
    )

    ctx.check(
        "upper louvers use long-axis pivots",
        all(joint.axis == (1.0, 0.0, 0.0) for joint in louver_joints),
        details=str([joint.axis for joint in louver_joints]),
    )

    for index, louver in enumerate(louvers, start=1):
        ctx.expect_gap(
            louver,
            shutter_panel,
            axis="x",
            min_gap=0.001,
            positive_elem="blade",
            negative_elem="left_stile",
            name=f"louver {index} clears left stile",
        )
        ctx.expect_gap(
            shutter_panel,
            louver,
            axis="x",
            min_gap=0.001,
            positive_elem="right_stile",
            negative_elem="blade",
            name=f"louver {index} clears right stile",
        )

    ctx.expect_gap(
        louvers[0],
        shutter_panel,
        axis="z",
        min_gap=0.010,
        positive_elem="blade",
        negative_elem="divider_rail",
        name="lowest louver clears divider rail",
    )
    ctx.expect_gap(
        shutter_panel,
        louvers[-1],
        axis="z",
        min_gap=0.010,
        positive_elem="top_rail",
        negative_elem="blade",
        name="highest louver clears top rail",
    )

    def elem_center_y(part, elem: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        return 0.5 * (aabb[0][1] + aabb[1][1])

    def elem_span(part, elem: str, axis_index: int) -> float | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        return aabb[1][axis_index] - aabb[0][axis_index]

    closed_free_edge_y = elem_center_y(shutter_panel, "right_stile")
    with ctx.pose({shutter_hinge: 1.1}):
        open_free_edge_y = elem_center_y(shutter_panel, "right_stile")
    ctx.check(
        "shutter opens outward from the frame",
        closed_free_edge_y is not None
        and open_free_edge_y is not None
        and open_free_edge_y > closed_free_edge_y + 0.16,
        details=f"closed_y={closed_free_edge_y}, open_y={open_free_edge_y}",
    )

    reference_louver = louvers[1]
    reference_joint = louver_joints[1]
    rest_z_span = elem_span(reference_louver, "blade", 2)
    with ctx.pose({reference_joint: 0.45}):
        tilted_z_span = elem_span(reference_louver, "blade", 2)
    ctx.check(
        "a louver blade tilts on its long axis",
        rest_z_span is not None
        and tilted_z_span is not None
        and tilted_z_span > rest_z_span + 0.008,
        details=f"rest_z_span={rest_z_span}, tilted_z_span={tilted_z_span}",
    )

    ctx.check(
        "single shutter panel stays within frame opening envelope",
        clear_width > panel_width and clear_height > panel_height,
        details=(
            f"clear_width={clear_width}, panel_width={panel_width}, "
            f"clear_height={clear_height}, panel_height={panel_height}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
