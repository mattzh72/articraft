from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def rectangle_profile(width: float, depth: float) -> list[tuple[float, float]]:
    half_w = width / 2.0
    half_d = depth / 2.0
    return [
        (-half_w, -half_d),
        (half_w, -half_d),
        (half_w, half_d),
        (-half_w, half_d),
    ]


def translate_profile(
    profile: list[tuple[float, float]], dx: float, dy: float
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    return tuple((aabb[0][i] + aabb[1][i]) / 2.0 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dj_mixer")

    body_black = model.material("body_black", rgba=(0.09, 0.09, 0.10, 1.0))
    panel_charcoal = model.material("panel_charcoal", rgba=(0.16, 0.16, 0.18, 1.0))
    fader_cap_gray = model.material("fader_cap_gray", rgba=(0.73, 0.73, 0.76, 1.0))
    cross_cap_orange = model.material("cross_cap_orange", rgba=(0.88, 0.51, 0.18, 1.0))
    knob_black = model.material("knob_black", rgba=(0.11, 0.11, 0.12, 1.0))
    indicator_blue = model.material("indicator_blue", rgba=(0.24, 0.58, 0.96, 1.0))

    housing_width = 0.320
    housing_depth = 0.240
    housing_height = 0.040
    floor_thickness = 0.004
    wall_thickness = 0.005
    panel_thickness = 0.003
    inner_wall_height = housing_height - floor_thickness - panel_thickness
    panel_top_z = housing_height
    panel_center_z = housing_height - panel_thickness / 2.0
    wall_center_z = floor_thickness + inner_wall_height / 2.0

    channel_slot_length = 0.100
    channel_slot_width = 0.008
    channel_fader_travel = 0.040
    channel_x_positions = (-0.105, -0.035, 0.035, 0.105)
    channel_slot_y = -0.005

    cross_slot_length = 0.108
    cross_slot_width = 0.009
    cross_fader_travel = 0.042
    cross_slot_y = -0.086

    master_slot_length = 0.082
    master_slot_width = 0.008
    master_fader_travel = 0.031
    master_slot_x = 0.140
    master_slot_y = -0.002

    knob_x_positions = (-0.080, 0.000, 0.080, -0.080, 0.000, 0.080)
    knob_y_positions = (0.070, 0.070, 0.070, 0.106, 0.106, 0.106)

    hole_profiles = [
        translate_profile(
            rounded_rect_profile(
                channel_slot_width,
                channel_slot_length,
                radius=channel_slot_width / 2.0,
                corner_segments=8,
            ),
            x,
            channel_slot_y,
        )
        for x in channel_x_positions
    ]
    hole_profiles.append(
        translate_profile(
            rounded_rect_profile(
                cross_slot_length,
                cross_slot_width,
                radius=cross_slot_width / 2.0,
                corner_segments=8,
            ),
            0.0,
            cross_slot_y,
        )
    )
    hole_profiles.append(
        translate_profile(
            rounded_rect_profile(
                master_slot_width,
                master_slot_length,
                radius=master_slot_width / 2.0,
                corner_segments=8,
            ),
            master_slot_x,
            master_slot_y,
        )
    )

    housing = model.part("housing")
    housing.visual(
        Box((housing_width, housing_depth, floor_thickness)),
        origin=Origin(xyz=(0.0, 0.0, floor_thickness / 2.0)),
        material=body_black,
        name="floor",
    )
    housing.visual(
        Box((housing_width, wall_thickness, inner_wall_height)),
        origin=Origin(
            xyz=(0.0, -housing_depth / 2.0 + wall_thickness / 2.0, wall_center_z)
        ),
        material=body_black,
        name="front_wall",
    )
    housing.visual(
        Box((housing_width, wall_thickness, inner_wall_height)),
        origin=Origin(
            xyz=(0.0, housing_depth / 2.0 - wall_thickness / 2.0, wall_center_z)
        ),
        material=body_black,
        name="rear_wall",
    )
    housing.visual(
        Box((wall_thickness, housing_depth - 2.0 * wall_thickness, inner_wall_height)),
        origin=Origin(
            xyz=(-housing_width / 2.0 + wall_thickness / 2.0, 0.0, wall_center_z)
        ),
        material=body_black,
        name="left_wall",
    )
    housing.visual(
        Box((wall_thickness, housing_depth - 2.0 * wall_thickness, inner_wall_height)),
        origin=Origin(
            xyz=(housing_width / 2.0 - wall_thickness / 2.0, 0.0, wall_center_z)
        ),
        material=body_black,
        name="right_wall",
    )

    top_panel_geom = ExtrudeWithHolesGeometry(
        rectangle_profile(housing_width, housing_depth),
        hole_profiles,
        panel_thickness,
        cap=True,
        center=True,
        closed=True,
    )
    housing.visual(
        mesh_from_geometry(top_panel_geom, "mixer_top_panel"),
        origin=Origin(xyz=(0.0, 0.0, panel_center_z)),
        material=panel_charcoal,
        name="top_panel",
    )
    housing.inertial = Inertial.from_geometry(
        Box((housing_width, housing_depth, housing_height)),
        mass=4.6,
        origin=Origin(xyz=(0.0, 0.0, housing_height / 2.0)),
    )

    def add_fader(
        name: str,
        *,
        x: float,
        y: float,
        axis: tuple[float, float, float],
        travel: float,
        cap_size: tuple[float, float, float],
        stem_size: tuple[float, float, float],
        guide_size: tuple[float, float, float],
        cap_material,
    ):
        fader = model.part(name)
        fader.visual(
            Box(cap_size),
            origin=Origin(xyz=(0.0, 0.0, cap_size[2] / 2.0)),
            material=cap_material,
            name="cap",
        )
        fader.visual(
            Box(stem_size),
            origin=Origin(xyz=(0.0, 0.0, -stem_size[2] / 2.0)),
            material=cap_material,
            name="stem",
        )
        fader.visual(
            Box(guide_size),
            origin=Origin(xyz=(0.0, 0.0, -(panel_thickness + guide_size[2] / 2.0))),
            material=body_black,
            name="guide",
        )
        fader.inertial = Inertial.from_geometry(
            Box(
                (
                    max(cap_size[0], guide_size[0]),
                    max(cap_size[1], guide_size[1]),
                    cap_size[2] + panel_thickness + guide_size[2],
                )
            ),
            mass=0.08,
            origin=Origin(
                xyz=(0.0, 0.0, (cap_size[2] - panel_thickness - guide_size[2]) / 4.0)
            ),
        )
        slider_joint = model.articulation(
            f"housing_to_{name}",
            ArticulationType.PRISMATIC,
            parent=housing,
            child=fader,
            origin=Origin(xyz=(x, y, panel_top_z)),
            axis=axis,
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=0.20,
                lower=-travel,
                upper=travel,
            ),
        )
        return fader, slider_joint

    def add_knob(name: str, *, x: float, y: float):
        knob = model.part(name)
        knob.visual(
            Cylinder(radius=0.015, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, 0.002)),
            material=knob_black,
            name="skirt",
        )
        knob.visual(
            Cylinder(radius=0.013, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, 0.007)),
            material=knob_black,
            name="body",
        )
        knob.visual(
            Box((0.003, 0.008, 0.0015)),
            origin=Origin(xyz=(0.0, 0.007, 0.0105)),
            material=indicator_blue,
            name="indicator",
        )
        knob.inertial = Inertial.from_geometry(
            Cylinder(radius=0.015, length=0.012),
            mass=0.06,
            origin=Origin(xyz=(0.0, 0.0, 0.006)),
        )
        knob_joint = model.articulation(
            f"housing_to_{name}",
            ArticulationType.REVOLUTE,
            parent=housing,
            child=knob,
            origin=Origin(xyz=(x, y, panel_top_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=1.0,
                velocity=2.5,
                lower=-2.356,
                upper=2.356,
            ),
        )
        return knob, knob_joint

    add_fader(
        "crossfader",
        x=0.0,
        y=cross_slot_y,
        axis=(1.0, 0.0, 0.0),
        travel=cross_fader_travel,
        cap_size=(0.020, 0.014, 0.016),
        stem_size=(0.010, 0.004, 0.007),
        guide_size=(0.024, 0.018, 0.004),
        cap_material=cross_cap_orange,
    )

    for index, x in enumerate(channel_x_positions, start=1):
        add_fader(
            f"channel_fader_{index}",
            x=x,
            y=channel_slot_y,
            axis=(0.0, 1.0, 0.0),
            travel=channel_fader_travel,
            cap_size=(0.014, 0.021, 0.012),
            stem_size=(0.0035, 0.010, 0.007),
            guide_size=(0.018, 0.022, 0.004),
            cap_material=fader_cap_gray,
        )

    add_fader(
        "master_fader",
        x=master_slot_x,
        y=master_slot_y,
        axis=(0.0, 1.0, 0.0),
        travel=master_fader_travel,
        cap_size=(0.013, 0.019, 0.011),
        stem_size=(0.0035, 0.010, 0.007),
        guide_size=(0.017, 0.020, 0.004),
        cap_material=fader_cap_gray,
    )

    for index, (x, y) in enumerate(zip(knob_x_positions, knob_y_positions), start=1):
        add_knob(f"eq_knob_{index}", x=x, y=y)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    crossfader = object_model.get_part("crossfader")
    channel_faders = [
        object_model.get_part(f"channel_fader_{index}") for index in range(1, 5)
    ]
    master_fader = object_model.get_part("master_fader")
    knobs = [object_model.get_part(f"eq_knob_{index}") for index in range(1, 7)]

    cross_joint = object_model.get_articulation("housing_to_crossfader")
    channel_joints = [
        object_model.get_articulation(f"housing_to_channel_fader_{index}")
        for index in range(1, 5)
    ]
    master_joint = object_model.get_articulation("housing_to_master_fader")
    knob_joints = [
        object_model.get_articulation(f"housing_to_eq_knob_{index}")
        for index in range(1, 7)
    ]

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "prompt part count matches DJ mixer layout",
        len(object_model.parts) == 13 and len(object_model.articulations) == 12,
        details=f"parts={len(object_model.parts)}, articulations={len(object_model.articulations)}",
    )

    for mounted_part in [crossfader, *channel_faders, master_fader, *knobs]:
        ctx.expect_contact(
            mounted_part,
            housing,
            name=f"{mounted_part.name} is physically mounted to the housing",
        )

    ctx.check(
        "crossfader uses a center horizontal prismatic joint",
        cross_joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(cross_joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={cross_joint.articulation_type}, axis={cross_joint.axis}",
    )
    for index, joint in enumerate(channel_joints, start=1):
        ctx.check(
            f"channel fader {index} uses a vertical prismatic joint",
            joint.articulation_type == ArticulationType.PRISMATIC
            and tuple(joint.axis) == (0.0, 1.0, 0.0),
            details=f"type={joint.articulation_type}, axis={joint.axis}",
        )
    ctx.check(
        "master output fader uses a vertical prismatic joint",
        master_joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(master_joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={master_joint.articulation_type}, axis={master_joint.axis}",
    )

    for index, joint in enumerate(knob_joints, start=1):
        ctx.check(
            f"EQ knob {index} uses a front-panel revolute joint",
            joint.articulation_type == ArticulationType.REVOLUTE
            and tuple(joint.axis) == (0.0, 0.0, 1.0),
            details=f"type={joint.articulation_type}, axis={joint.axis}",
        )

    cross_rest = ctx.part_world_position(crossfader)
    with ctx.pose({cross_joint: cross_joint.motion_limits.upper}):
        cross_open = ctx.part_world_position(crossfader)
        ctx.expect_contact(
            crossfader,
            housing,
            name="crossfader remains guided at maximum right travel",
        )
    ctx.check(
        "crossfader slides left to right across the mixer face",
        cross_rest is not None
        and cross_open is not None
        and cross_open[0] > cross_rest[0] + 0.03,
        details=f"rest={cross_rest}, moved={cross_open}",
    )

    channel_rest = ctx.part_world_position(channel_faders[0])
    with ctx.pose({channel_joints[0]: channel_joints[0].motion_limits.upper}):
        channel_open = ctx.part_world_position(channel_faders[0])
        ctx.expect_contact(
            channel_faders[0],
            housing,
            name="channel fader remains guided at maximum travel",
        )
    ctx.check(
        "channel volume fader slides from front toward rear",
        channel_rest is not None
        and channel_open is not None
        and channel_open[1] > channel_rest[1] + 0.03,
        details=f"rest={channel_rest}, moved={channel_open}",
    )

    master_rest = ctx.part_world_position(master_fader)
    with ctx.pose({master_joint: master_joint.motion_limits.upper}):
        master_open = ctx.part_world_position(master_fader)
        ctx.expect_contact(
            master_fader,
            housing,
            name="master fader remains guided at maximum travel",
        )
    ctx.check(
        "master output fader slides on its own vertical slot",
        master_rest is not None
        and master_open is not None
        and master_open[1] > master_rest[1] + 0.02,
        details=f"rest={master_rest}, moved={master_open}",
    )

    indicator_rest = aabb_center(
        ctx.part_element_world_aabb(knobs[0], elem="indicator")
    )
    with ctx.pose({knob_joints[0]: 1.2}):
        indicator_turned = aabb_center(
            ctx.part_element_world_aabb(knobs[0], elem="indicator")
        )
        ctx.expect_contact(
            knobs[0],
            housing,
            name="EQ knob stays seated while rotating",
        )
    ctx.check(
        "EQ knob indicator visibly rotates around the knob axis",
        indicator_rest is not None
        and indicator_turned is not None
        and abs(indicator_turned[0] - indicator_rest[0]) > 0.004
        and abs(indicator_turned[2] - indicator_rest[2]) < 0.001,
        details=f"rest={indicator_rest}, turned={indicator_turned}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
