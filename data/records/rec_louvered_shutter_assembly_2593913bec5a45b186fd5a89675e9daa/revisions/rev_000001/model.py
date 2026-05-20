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
    model = ArticulatedObject(name="timber_louvered_shutter")

    timber = model.material("timber", rgba=(0.42, 0.28, 0.16, 1.0))
    darker_timber = model.material("darker_timber", rgba=(0.31, 0.20, 0.11, 1.0))
    iron = model.material("iron", rgba=(0.21, 0.21, 0.22, 1.0))

    outer_width = 0.76
    opening_width = 0.56
    frame_depth = 0.08
    outer_height = 1.62
    opening_height = 1.42
    jamb_width = (outer_width - opening_width) * 0.5
    rail_height = (outer_height - opening_height) * 0.5

    hinge_x = -opening_width * 0.5 - 0.006
    panel_height = opening_height
    panel_width = 0.56
    panel_thickness = 0.045
    stile_width = 0.056
    bottom_rail_height = 0.10
    top_rail_height = 0.08
    panel_inner_y = panel_thickness * 0.5

    louver_count = 8
    louver_pitch = 0.145
    louver_length = 0.392
    louver_depth = 0.110
    louver_thickness = 0.016
    louver_pin_radius = 0.0055
    louver_pin_length = 0.022
    louver_limits = MotionLimits(
        effort=1.5,
        velocity=2.0,
        lower=-0.95,
        upper=0.95,
    )

    blade_profile = [
        (-louver_depth * 0.50, 0.0),
        (-louver_depth * 0.32, louver_thickness * 0.50),
        (louver_depth * 0.32, louver_thickness * 0.50),
        (louver_depth * 0.50, 0.0),
        (louver_depth * 0.32, -louver_thickness * 0.50),
        (-louver_depth * 0.32, -louver_thickness * 0.50),
    ]
    blade_mesh = mesh_from_geometry(
        ExtrudeGeometry.centered(blade_profile, louver_length, cap=True).rotate_y(math.pi / 2.0),
        "shutter_louver_blade",
    )

    opening_frame = model.part("opening_frame")
    opening_frame.visual(
        Box((jamb_width, frame_depth, outer_height)),
        origin=Origin(xyz=(-(opening_width * 0.5 + jamb_width * 0.5), 0.0, outer_height * 0.5)),
        material=darker_timber,
        name="left_jamb",
    )
    opening_frame.visual(
        Box((jamb_width, frame_depth, outer_height)),
        origin=Origin(xyz=((opening_width * 0.5 + jamb_width * 0.5), 0.0, outer_height * 0.5)),
        material=darker_timber,
        name="right_jamb",
    )
    opening_frame.visual(
        Box((outer_width, frame_depth, rail_height)),
        origin=Origin(xyz=(0.0, 0.0, rail_height * 0.5)),
        material=darker_timber,
        name="sill",
    )
    opening_frame.visual(
        Box((outer_width, frame_depth, rail_height)),
        origin=Origin(xyz=(0.0, 0.0, outer_height - rail_height * 0.5)),
        material=darker_timber,
        name="head",
    )
    for hinge_name, hinge_z in (("upper_hinge_pin", 1.29), ("lower_hinge_pin", 0.145)):
        opening_frame.visual(
            Cylinder(radius=0.009, length=0.17),
            origin=Origin(xyz=(hinge_x, frame_depth * 0.5 + 0.004, hinge_z)),
            material=iron,
            name=hinge_name,
        )
    for strap_name, strap_z in (("upper_frame_strap", 1.29), ("lower_frame_strap", 0.145)):
        opening_frame.visual(
            Box((0.17, 0.008, 0.055)),
            origin=Origin(xyz=(hinge_x - 0.080, frame_depth * 0.5 + 0.004, strap_z)),
            material=iron,
            name=strap_name,
        )
    opening_frame.inertial = Inertial.from_geometry(
        Box((outer_width, frame_depth, outer_height)),
        mass=38.0,
        origin=Origin(xyz=(0.0, 0.0, outer_height * 0.5)),
    )

    shutter_panel = model.part("shutter_panel")
    shutter_panel.visual(
        Box((stile_width, panel_thickness, panel_height)),
        origin=Origin(xyz=(0.042, panel_inner_y, panel_height * 0.5)),
        material=timber,
        name="left_stile",
    )
    shutter_panel.visual(
        Box((stile_width, panel_thickness, panel_height)),
        origin=Origin(xyz=(0.532, panel_inner_y, panel_height * 0.5)),
        material=timber,
        name="right_stile",
    )
    shutter_panel.visual(
        Box((0.546, panel_thickness, bottom_rail_height)),
        origin=Origin(xyz=(0.287, panel_inner_y, bottom_rail_height * 0.5)),
        material=timber,
        name="bottom_rail",
    )
    shutter_panel.visual(
        Box((0.546, panel_thickness, top_rail_height)),
        origin=Origin(xyz=(0.287, panel_inner_y, panel_height - top_rail_height * 0.5)),
        material=timber,
        name="top_rail",
    )
    shutter_panel.visual(
        Box((0.060, 0.008, 0.050)),
        origin=Origin(xyz=(0.040, panel_thickness * 0.5 + 0.004, 1.29)),
        material=iron,
        name="upper_panel_strap",
    )
    shutter_panel.visual(
        Box((0.060, 0.008, 0.050)),
        origin=Origin(xyz=(0.040, panel_thickness * 0.5 + 0.004, 0.145)),
        material=iron,
        name="lower_panel_strap",
    )
    shutter_panel.visual(
        Cylinder(radius=0.008, length=0.048),
        origin=Origin(xyz=(0.010, panel_thickness * 0.5 + 0.004, 1.29)),
        material=iron,
        name="upper_hinge_knuckle",
    )
    shutter_panel.visual(
        Cylinder(radius=0.008, length=0.048),
        origin=Origin(xyz=(0.010, panel_thickness * 0.5 + 0.004, 0.145)),
        material=iron,
        name="lower_hinge_knuckle",
    )
    shutter_panel.inertial = Inertial.from_geometry(
        Box((panel_width, panel_thickness, panel_height)),
        mass=20.0,
        origin=Origin(xyz=(panel_width * 0.5, panel_inner_y, panel_height * 0.5)),
    )

    control_rod = model.part("control_rod")
    control_rod.visual(
        Cylinder(radius=0.007, length=1.224),
        origin=Origin(xyz=(0.287, 0.055, 0.730)),
        material=iron,
        name="rod_shaft",
    )
    control_rod.visual(
        Box((0.022, 0.020, 0.018)),
        origin=Origin(xyz=(0.287, 0.055, 0.109)),
        material=iron,
        name="bottom_guide",
    )
    control_rod.visual(
        Box((0.022, 0.020, 0.018)),
        origin=Origin(xyz=(0.287, 0.055, 1.351)),
        material=iron,
        name="top_guide",
    )
    control_rod.inertial = Inertial.from_geometry(
        Box((0.03, 0.03, 1.26)),
        mass=0.9,
        origin=Origin(xyz=(0.287, 0.055, 0.73)),
    )

    model.articulation(
        "opening_to_panel",
        ArticulationType.REVOLUTE,
        parent=opening_frame,
        child=shutter_panel,
        origin=Origin(xyz=(hinge_x, frame_depth * 0.5, rail_height)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )
    model.articulation(
        "panel_to_control_rod",
        ArticulationType.FIXED,
        parent=shutter_panel,
        child=control_rod,
        origin=Origin(),
    )

    first_louver_z = 0.205
    for index in range(louver_count):
        louver = model.part(f"louver_{index}")
        louver.visual(
            blade_mesh,
            material=timber,
            name="blade",
        )
        louver.visual(
            Cylinder(radius=louver_pin_radius, length=louver_pin_length),
            origin=Origin(xyz=(-0.206, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=timber,
            name="left_pin",
        )
        louver.visual(
            Cylinder(radius=louver_pin_radius, length=louver_pin_length),
            origin=Origin(xyz=(0.206, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=timber,
            name="right_pin",
        )
        louver.visual(
            Box((0.014, 0.022, 0.030)),
            origin=Origin(xyz=(0.0, 0.011, -0.014)),
            material=iron,
            name="link_tab",
        )
        louver.inertial = Inertial.from_geometry(
            Box((louver_length + louver_pin_length, louver_thickness, louver_depth)),
            mass=1.2,
            origin=Origin(),
        )
        model.articulation(
            f"panel_to_louver_{index}",
            ArticulationType.REVOLUTE,
            parent=shutter_panel,
            child=louver,
            origin=Origin(xyz=(0.287, panel_inner_y, first_louver_z + index * louver_pitch)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=louver_limits,
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
    opening_frame = object_model.get_part("opening_frame")
    shutter_panel = object_model.get_part("shutter_panel")
    control_rod = object_model.get_part("control_rod")
    panel_hinge = object_model.get_articulation("opening_to_panel")
    mid_louver = object_model.get_part("louver_4")
    mid_louver_joint = object_model.get_articulation("panel_to_louver_4")

    ctx.expect_gap(
        shutter_panel,
        opening_frame,
        axis="y",
        min_gap=0.0,
        max_gap=0.002,
        positive_elem="left_stile",
        negative_elem="left_jamb",
        name="closed stile sits against the jamb face",
    )
    ctx.expect_overlap(
        shutter_panel,
        opening_frame,
        axes="xz",
        min_overlap=0.50,
        name="closed shutter covers the opening footprint",
    )
    ctx.expect_within(
        control_rod,
        shutter_panel,
        axes="xz",
        margin=0.03,
        name="control rod stays within the shutter frame bounds",
    )

    louver_joint_ok = True
    louver_joint_details: list[str] = []
    for index in range(8):
        joint = object_model.get_articulation(f"panel_to_louver_{index}")
        limits = joint.motion_limits
        joint_ok = (
            joint.axis == (1.0, 0.0, 0.0)
            and limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower < 0.0
            and limits.upper > 0.0
        )
        louver_joint_ok &= joint_ok
        if not joint_ok:
            louver_joint_details.append(
                f"{joint.name}: axis={joint.axis}, limits={limits.lower if limits else None},{limits.upper if limits else None}"
            )
    ctx.check(
        "all louvers hinge on their long horizontal axes",
        louver_joint_ok,
        details="; ".join(louver_joint_details),
    )

    closed_right_stile = ctx.part_element_world_aabb(shutter_panel, elem="right_stile")
    with ctx.pose({panel_hinge: math.radians(80.0)}):
        opened_right_stile = ctx.part_element_world_aabb(shutter_panel, elem="right_stile")
    panel_open_ok = (
        closed_right_stile is not None
        and opened_right_stile is not None
        and ((opened_right_stile[0][1] + opened_right_stile[1][1]) * 0.5)
        > ((closed_right_stile[0][1] + closed_right_stile[1][1]) * 0.5) + 0.20
    )
    ctx.check(
        "panel swings outward on the jamb-side vertical hinge",
        panel_open_ok,
        details=f"closed={closed_right_stile}, opened={opened_right_stile}",
    )

    closed_tab = ctx.part_element_world_aabb(mid_louver, elem="link_tab")
    with ctx.pose({mid_louver_joint: 0.65}):
        opened_tab = ctx.part_element_world_aabb(mid_louver, elem="link_tab")
    louver_motion_ok = (
        closed_tab is not None
        and opened_tab is not None
        and ((opened_tab[0][2] + opened_tab[1][2]) * 0.5)
        > ((closed_tab[0][2] + closed_tab[1][2]) * 0.5) + 0.004
    )
    ctx.check(
        "representative louver rotates about its long axis",
        louver_motion_ok,
        details=f"closed={closed_tab}, opened={opened_tab}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
