from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelFace,
    BezelGeometry,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rounded_panel_mesh(
    *,
    width: float,
    height: float,
    thickness: float,
    radius: float,
    z_center: float,
    y_center: float,
    name: str,
):
    """Build a rounded rectangular panel in the local X/Z plane."""
    geom = ExtrudeGeometry(
        rounded_rect_profile(width, height, radius, corner_segments=10),
        thickness,
        cap=True,
        center=True,
    )
    geom.rotate_x(math.pi / 2.0)
    geom.translate(0.0, y_center, z_center)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="automotive_glove_compartment")

    fascia_plastic = model.material(
        "warm_charcoal_dashboard",
        rgba=(0.085, 0.080, 0.073, 1.0),
    )
    soft_panel = model.material(
        "slightly_lighter_grained_door",
        rgba=(0.125, 0.120, 0.110, 1.0),
    )
    dark_shadow = model.material(
        "deep_cavity_shadow",
        rgba=(0.015, 0.014, 0.013, 1.0),
    )
    satin_edge = model.material(
        "subtle_satin_edge",
        rgba=(0.18, 0.17, 0.155, 1.0),
    )
    rubber = model.material("matte_black_rubber", rgba=(0.02, 0.019, 0.018, 1.0))

    frame = model.part("frame")

    outer_bezel = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.700, 0.460, 0.055, corner_segments=12),
        [rounded_rect_profile(0.580, 0.380, 0.030, corner_segments=12)],
        0.055,
        cap=True,
        center=False,
    )
    frame.visual(
        mesh_from_geometry(outer_bezel, "outer_bezel"),
        origin=Origin(xyz=(0.0, 0.0, 0.205), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=fascia_plastic,
        name="outer_bezel",
    )

    # Recessed glove-box bin behind the door, visible when opened.
    frame.visual(
        Box((0.592, 0.012, 0.360)),
        origin=Origin(xyz=(0.0, 0.236, 0.205)),
        material=dark_shadow,
        name="bin_back",
    )
    frame.visual(
        Box((0.018, 0.218, 0.340)),
        origin=Origin(xyz=(-0.287, 0.127, 0.205)),
        material=dark_shadow,
        name="bin_side_0",
    )
    frame.visual(
        Box((0.018, 0.218, 0.340)),
        origin=Origin(xyz=(0.287, 0.127, 0.205)),
        material=dark_shadow,
        name="bin_side_1",
    )
    frame.visual(
        Box((0.574, 0.218, 0.018)),
        origin=Origin(xyz=(0.0, 0.127, 0.386)),
        material=dark_shadow,
        name="bin_roof",
    )
    frame.visual(
        Box((0.574, 0.218, 0.018)),
        origin=Origin(xyz=(0.0, 0.127, 0.024)),
        material=dark_shadow,
        name="bin_sill",
    )

    # Surrounding molded dashboard pads give the assembly context and scale.
    frame.visual(
        Box((0.780, 0.115, 0.070)),
        origin=Origin(xyz=(0.0, 0.045, 0.455)),
        material=fascia_plastic,
        name="upper_dash_pad",
    )
    frame.visual(
        Box((0.745, 0.075, 0.060)),
        origin=Origin(xyz=(0.0, 0.034, -0.052)),
        material=fascia_plastic,
        name="lower_knee_pad",
    )
    frame.visual(
        Box((0.060, 0.078, 0.470)),
        origin=Origin(xyz=(-0.395, 0.030, 0.205)),
        material=fascia_plastic,
        name="side_pad_0",
    )
    frame.visual(
        Box((0.060, 0.078, 0.470)),
        origin=Origin(xyz=(0.395, 0.030, 0.205)),
        material=fascia_plastic,
        name="side_pad_1",
    )

    # Static knuckles on the dashboard half of the lower hinge.
    for suffix, x in (("0", -0.236), ("1", 0.236)):
        frame.visual(
            Box((0.086, 0.014, 0.024)),
            origin=Origin(xyz=(x, 0.000, -0.006)),
            material=satin_edge,
            name=f"hinge_mount_{suffix}",
        )
        frame.visual(
            Cylinder(radius=0.012, length=0.086),
            origin=Origin(xyz=(x, -0.006, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_edge,
            name=f"hinge_knuckle_{suffix}",
        )

    door = model.part("door")
    door.visual(
        _rounded_panel_mesh(
            width=0.540,
            height=0.350,
            thickness=0.026,
            radius=0.034,
            z_center=0.190,
            y_center=-0.018,
            name="door_panel",
        ),
        material=soft_panel,
        name="door_panel",
    )
    door.visual(
        _rounded_panel_mesh(
            width=0.500,
            height=0.300,
            thickness=0.002,
            radius=0.028,
            z_center=0.198,
            y_center=-0.0322,
            name="inner_reveal_line",
        ),
        material=satin_edge,
        name="inner_reveal_line",
    )
    latch_bezel = BezelGeometry(
        opening_size=(0.128, 0.036),
        outer_size=(0.208, 0.076),
        depth=0.008,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.015,
        outer_corner_radius=0.022,
        face=BezelFace(style="soft_lip", front_lip=0.0025, fillet=0.002),
        center=False,
    )
    door.visual(
        mesh_from_geometry(latch_bezel, "latch_bezel"),
        origin=Origin(xyz=(0.0, -0.031, 0.292), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_edge,
        name="latch_bezel",
    )
    door.visual(
        Box((0.370, 0.002, 0.005)),
        origin=Origin(xyz=(0.0, -0.0328, 0.228)),
        material=satin_edge,
        name="upper_character_groove",
    )
    door.visual(
        Box((0.420, 0.002, 0.004)),
        origin=Origin(xyz=(0.0, -0.0328, 0.078)),
        material=satin_edge,
        name="lower_character_groove",
    )
    door.visual(
        Box((0.360, 0.010, 0.026)),
        origin=Origin(xyz=(0.0, -0.014, 0.012)),
        material=satin_edge,
        name="hinge_leaf",
    )
    door.visual(
        Cylinder(radius=0.012, length=0.386),
        origin=Origin(xyz=(0.0, -0.006, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_edge,
        name="hinge_knuckle",
    )
    door.visual(
        Box((0.055, 0.006, 0.012)),
        origin=Origin(xyz=(-0.180, -0.031, 0.065)),
        material=rubber,
        name="stop_pad_0",
    )
    door.visual(
        Box((0.055, 0.006, 0.012)),
        origin=Origin(xyz=(0.180, -0.031, 0.065)),
        material=rubber,
        name="stop_pad_1",
    )

    latch = model.part("latch_button")
    latch.visual(
        _rounded_panel_mesh(
            width=0.106,
            height=0.026,
            thickness=0.007,
            radius=0.012,
            z_center=0.0,
            y_center=0.0,
            name="button_cap",
        ),
        material=rubber,
        name="button_cap",
    )
    latch.visual(
        Box((0.070, 0.004, 0.008)),
        origin=Origin(xyz=(0.0, 0.002, -0.017)),
        material=rubber,
        name="button_stem",
    )

    model.articulation(
        "frame_to_door",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=door,
        origin=Origin(xyz=(0.0, -0.006, 0.000)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=1.12),
    )
    model.articulation(
        "door_to_latch_button",
        ArticulationType.PRISMATIC,
        parent=door,
        child=latch,
        origin=Origin(xyz=(0.0, -0.039, 0.292)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.08, lower=0.0, upper=0.004),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    door = object_model.get_part("door")
    latch = object_model.get_part("latch_button")
    door_joint = object_model.get_articulation("frame_to_door")
    latch_joint = object_model.get_articulation("door_to_latch_button")

    ctx.expect_within(
        door,
        frame,
        axes="xz",
        inner_elem="door_panel",
        outer_elem="outer_bezel",
        margin=0.010,
        name="closed door fits inside dashboard opening",
    )
    ctx.expect_gap(
        frame,
        door,
        axis="y",
        positive_elem="outer_bezel",
        negative_elem="door_panel",
        min_gap=0.003,
        max_gap=0.012,
        name="closed door sits just proud of the fascia",
    )
    ctx.expect_within(
        latch,
        door,
        axes="xz",
        inner_elem="button_cap",
        outer_elem="latch_bezel",
        margin=0.002,
        name="latch button sits within the recessed bezel",
    )

    closed_aabb = ctx.part_world_aabb(door)
    closed_latch_pos = ctx.part_world_position(latch)
    with ctx.pose({door_joint: 1.12}):
        open_aabb = ctx.part_world_aabb(door)
    with ctx.pose({latch_joint: 0.004}):
        pressed_latch_pos = ctx.part_world_position(latch)

    ctx.check(
        "door opens downward and out from lower hinge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][1] < closed_aabb[0][1] - 0.160
        and open_aabb[1][2] < closed_aabb[1][2] - 0.080,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )
    ctx.check(
        "latch button depresses into the door",
        closed_latch_pos is not None
        and pressed_latch_pos is not None
        and pressed_latch_pos[1] > closed_latch_pos[1] + 0.003,
        details=f"closed={closed_latch_pos}, pressed={pressed_latch_pos}",
    )

    return ctx.report()


object_model = build_object_model()
