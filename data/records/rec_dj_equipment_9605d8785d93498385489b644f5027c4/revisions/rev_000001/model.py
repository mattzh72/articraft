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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


HOUSING_WIDTH = 0.325
HOUSING_DEPTH = 0.215
HOUSING_TOP_Z = 0.036
FOOT_HEIGHT = 0.004
TOP_PANEL_THICKNESS = 0.0016

PAD_SIZE = 0.056
PAD_PITCH = 0.065
PAD_BODY_BOTTOM = 0.0012
PAD_BODY_TOP = 0.0088
PAD_HINGE_RADIUS = 0.003
PAD_COLUMNS = (-0.1125, -0.0475, 0.0175, 0.0825)
PAD_ROWS = (-0.028, 0.037)


def _profile_section(
    width: float,
    depth: float,
    radius: float,
    z: float,
    *,
    center_x: float = 0.0,
    center_y: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (center_x + x, center_y + y, z)
        for x, y in rounded_rect_profile(width, depth, radius, corner_segments=8)
    ]


def _build_housing_mesh():
    return section_loft(
        [
            _profile_section(HOUSING_WIDTH, HOUSING_DEPTH, 0.023, FOOT_HEIGHT),
            _profile_section(HOUSING_WIDTH - 0.003, HOUSING_DEPTH - 0.003, 0.021, FOOT_HEIGHT + 0.004),
            _profile_section(HOUSING_WIDTH - 0.008, HOUSING_DEPTH - 0.008, 0.018, HOUSING_TOP_Z),
        ]
    )


def _offset_profile(
    profile: list[tuple[float, float]],
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _build_top_panel_mesh():
    outer = rounded_rect_profile(0.300, 0.186, 0.012, corner_segments=8)
    pad_bay = _offset_profile(
        rounded_rect_profile(0.250, 0.136, 0.012, corner_segments=8),
        dx=-0.015,
        dy=0.0035,
    )
    slider_slot = _offset_profile(
        rounded_rect_profile(0.008, 0.108, 0.003, corner_segments=8),
        dx=0.139,
        dy=0.0,
    )
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            outer,
            [pad_bay, slider_slot],
            TOP_PANEL_THICKNESS,
            center=True,
            cap=True,
            closed=True,
        ),
        "sampler_top_panel",
    )


def _build_pad_mesh(name: str):
    front_offset = 0.002
    center_y = front_offset + PAD_SIZE * 0.5
    pad_mesh = section_loft(
        [
            _profile_section(PAD_SIZE, PAD_SIZE, 0.008, PAD_BODY_BOTTOM, center_y=center_y),
            _profile_section(PAD_SIZE - 0.004, PAD_SIZE - 0.004, 0.007, PAD_BODY_TOP, center_y=center_y),
        ]
    )
    return mesh_from_geometry(pad_mesh, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dj_sampler_unit")

    housing_mat = model.material("housing_charcoal", rgba=(0.14, 0.15, 0.17, 1.0))
    deck_mat = model.material("deck_graphite", rgba=(0.08, 0.09, 0.10, 1.0))
    pad_mat = model.material("pad_rubber", rgba=(0.24, 0.25, 0.27, 1.0))
    knob_mat = model.material("knob_black", rgba=(0.11, 0.12, 0.13, 1.0))
    indicator_mat = model.material("indicator_white", rgba=(0.92, 0.93, 0.94, 1.0))
    accent_mat = model.material("accent_orange", rgba=(0.86, 0.43, 0.16, 1.0))
    foot_mat = model.material("foot_rubber", rgba=(0.05, 0.05, 0.05, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_geometry(_build_housing_mesh(), "sampler_housing_shell"),
        material=housing_mat,
        name="housing_shell",
    )

    for foot_index, (x_pos, y_pos) in enumerate(
        (
            (-0.120, -0.072),
            (0.120, -0.072),
            (-0.120, 0.072),
            (0.120, 0.072),
        )
    ):
        housing.visual(
            Cylinder(radius=0.011, length=FOOT_HEIGHT),
            origin=Origin(xyz=(x_pos, y_pos, FOOT_HEIGHT * 0.5)),
            material=foot_mat,
            name=f"foot_{foot_index}",
        )

    housing.inertial = Inertial.from_geometry(
        Box((HOUSING_WIDTH, HOUSING_DEPTH, HOUSING_TOP_Z)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, HOUSING_TOP_Z * 0.5)),
    )

    top_panel = model.part("top_panel")
    top_panel.visual(
        _build_top_panel_mesh(),
        material=deck_mat,
        name="top_panel_plate",
    )
    top_panel.inertial = Inertial.from_geometry(
        Box((0.300, 0.186, TOP_PANEL_THICKNESS)),
        mass=0.035,
    )
    model.articulation(
        "housing_to_top_panel",
        ArticulationType.FIXED,
        parent=housing,
        child=top_panel,
        origin=Origin(xyz=(0.0, 0.0, HOUSING_TOP_Z + TOP_PANEL_THICKNESS * 0.5)),
    )

    pad_mesh = _build_pad_mesh("sampler_pad_surface")
    for row_index, row_y in enumerate(PAD_ROWS):
        for col_index, col_x in enumerate(PAD_COLUMNS):
            pad_name = f"pad_{row_index}_{col_index}"
            pad = model.part(pad_name)
            pad.visual(
                Cylinder(radius=PAD_HINGE_RADIUS, length=PAD_SIZE - 0.014),
                origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=knob_mat,
                name="hinge_barrel",
            )
            pad.visual(
                pad_mesh,
                material=pad_mat,
                name="pad_surface",
            )
            pad.inertial = Inertial.from_geometry(
                Box((PAD_SIZE, PAD_SIZE, PAD_BODY_TOP)),
                mass=0.055,
                origin=Origin(xyz=(0.0, 0.030, 0.0045)),
            )
            model.articulation(
                f"housing_to_{pad_name}",
                ArticulationType.REVOLUTE,
                parent=housing,
                child=pad,
                origin=Origin(xyz=(col_x, row_y - 0.030, HOUSING_TOP_Z + PAD_HINGE_RADIUS)),
                axis=(-1.0, 0.0, 0.0),
                motion_limits=MotionLimits(
                    effort=6.0,
                    velocity=2.0,
                    lower=0.0,
                    upper=math.radians(3.0),
                ),
            )

    master_tempo_knob = model.part("master_tempo_knob")
    master_tempo_knob.visual(
        Cylinder(radius=0.015, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=knob_mat,
        name="knob_skirt",
    )
    master_tempo_knob.visual(
        Cylinder(radius=0.011, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=knob_mat,
        name="knob_cap",
    )
    master_tempo_knob.visual(
        Box((0.003, 0.010, 0.002)),
        origin=Origin(xyz=(0.0, 0.0085, 0.018)),
        material=indicator_mat,
        name="indicator",
    )
    master_tempo_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.015, length=0.018),
        mass=0.035,
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
    )
    model.articulation(
        "top_panel_to_master_tempo_knob",
        ArticulationType.REVOLUTE,
        parent=top_panel,
        child=master_tempo_knob,
        origin=Origin(xyz=(0.128, 0.079, TOP_PANEL_THICKNESS * 0.5)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=4.0,
            lower=-2.5,
            upper=2.5,
        ),
    )

    volume_slider = model.part("volume_slider")
    volume_slider.visual(
        Box((0.012, 0.020, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=knob_mat,
        name="slider_carriage",
    )
    volume_slider.visual(
        Box((0.006, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=knob_mat,
        name="slider_stem",
    )
    volume_slider.visual(
        Box((0.018, 0.028, 0.007)),
        origin=Origin(xyz=(0.0, 0.0, 0.0145)),
        material=accent_mat,
        name="slider_grip",
    )
    volume_slider.inertial = Inertial.from_geometry(
        Box((0.018, 0.028, 0.021)),
        mass=0.025,
        origin=Origin(xyz=(0.0, 0.0, 0.0105)),
    )
    model.articulation(
        "top_panel_to_volume_slider",
        ArticulationType.PRISMATIC,
        parent=top_panel,
        child=volume_slider,
        origin=Origin(xyz=(0.139, 0.0, TOP_PANEL_THICKNESS * 0.5)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.12,
            lower=-0.034,
            upper=0.034,
        ),
    )

    return model


def _aabb_center(aabb):
    return tuple((aabb[0][index] + aabb[1][index]) * 0.5 for index in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    top_panel = object_model.get_part("top_panel")
    knob = object_model.get_part("master_tempo_knob")
    slider = object_model.get_part("volume_slider")
    knob_joint = object_model.get_articulation("top_panel_to_master_tempo_knob")
    slider_joint = object_model.get_articulation("top_panel_to_volume_slider")

    pad_parts = []
    pad_joints = []
    for row_index in range(2):
        for col_index in range(4):
            pad_name = f"pad_{row_index}_{col_index}"
            pad_parts.append(object_model.get_part(pad_name))
            pad_joints.append(object_model.get_articulation(f"housing_to_{pad_name}"))

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check("eight_trigger_pads_present", len(pad_parts) == 8, details=f"found {len(pad_parts)} pads")
    ctx.expect_contact(top_panel, housing, name="top_panel_mounts_to_housing")

    for pad, joint in zip(pad_parts, pad_joints):
        axis = joint.axis
        ctx.check(
            f"{joint.name}_hinges_about_x",
            math.isclose(abs(axis[0]), 1.0, abs_tol=1e-9)
            and math.isclose(axis[1], 0.0, abs_tol=1e-9)
            and math.isclose(axis[2], 0.0, abs_tol=1e-9),
            details=f"unexpected pad joint axis {axis}",
        )
        ctx.expect_contact(pad, housing, name=f"{pad.name}_contacts_housing")
        ctx.expect_overlap(pad, housing, axes="xy", min_overlap=0.04, name=f"{pad.name}_sits_on_housing")

    ctx.check(
        "master_tempo_knob_axis_is_vertical",
        knob_joint.axis == (0.0, 0.0, 1.0),
        details=f"unexpected knob axis {knob_joint.axis}",
    )
    ctx.expect_contact(knob, top_panel, name="master_tempo_knob_contacts_top_panel")
    ctx.expect_overlap(knob, top_panel, axes="xy", min_overlap=0.02, name="master_tempo_knob_on_top_panel")

    ctx.check(
        "volume_slider_axis_runs_along_side_slot",
        slider_joint.axis == (0.0, 1.0, 0.0),
        details=f"unexpected slider axis {slider_joint.axis}",
    )
    ctx.expect_contact(slider, top_panel, name="volume_slider_contacts_slot")
    ctx.expect_overlap(slider, top_panel, axes="y", min_overlap=0.02, name="volume_slider_captured_in_slot")

    pad_positions = [ctx.part_world_position(pad) for pad in pad_parts]
    assert all(position is not None for position in pad_positions)
    x_positions = sorted({round(position[0], 4) for position in pad_positions if position is not None})
    y_positions = sorted({round(position[1], 4) for position in pad_positions if position is not None})
    ctx.check(
        "pad_grid_has_four_even_columns",
        len(x_positions) == 4 and all(abs((x_positions[i + 1] - x_positions[i]) - PAD_PITCH) < 0.001 for i in range(3)),
        details=f"pad column positions were {x_positions}",
    )
    ctx.check(
        "pad_grid_has_two_even_rows",
        len(y_positions) == 2 and abs((y_positions[1] - y_positions[0]) - PAD_PITCH) < 0.001,
        details=f"pad row hinge positions were {y_positions}",
    )

    sample_pad = pad_parts[0]
    sample_joint = pad_joints[0]
    pad_surface_rest = ctx.part_element_world_aabb(sample_pad, elem="pad_surface")
    assert pad_surface_rest is not None
    with ctx.pose({sample_joint: sample_joint.motion_limits.upper}):
        pad_surface_pressed = ctx.part_element_world_aabb(sample_pad, elem="pad_surface")
        assert pad_surface_pressed is not None
        ctx.check(
            "trigger_pad_depresses_when_hinged",
            pad_surface_pressed[0][2] < pad_surface_rest[0][2] - 0.001,
            details=f"rest {pad_surface_rest}, pressed {pad_surface_pressed}",
        )
        ctx.expect_gap(
            sample_pad,
            housing,
            axis="z",
            max_penetration=0.0,
            name="pressed_pad_does_not_sink_into_housing",
        )

    slider_rest = ctx.part_world_position(slider)
    assert slider_rest is not None
    with ctx.pose({slider_joint: slider_joint.motion_limits.upper}):
        slider_high = ctx.part_world_position(slider)
        assert slider_high is not None
        ctx.check(
            "volume_slider_moves_along_track",
            slider_high[1] > slider_rest[1] + 0.03
            and abs(slider_high[0] - slider_rest[0]) < 1e-6
            and abs(slider_high[2] - slider_rest[2]) < 1e-6,
            details=f"rest {slider_rest}, moved {slider_high}",
        )
        ctx.expect_contact(slider, top_panel, name="volume_slider_remains_mounted_at_high_setting")

    indicator_rest = ctx.part_element_world_aabb(knob, elem="indicator")
    assert indicator_rest is not None
    with ctx.pose({knob_joint: math.pi / 2.0}):
        indicator_turn = ctx.part_element_world_aabb(knob, elem="indicator")
        assert indicator_turn is not None
        rest_center = _aabb_center(indicator_rest)
        turn_center = _aabb_center(indicator_turn)
        ctx.check(
            "master_tempo_knob_indicator_orbits_center",
            abs(turn_center[0] - rest_center[0]) > 0.006 and abs(turn_center[1] - rest_center[1]) > 0.006,
            details=f"rest {rest_center}, turned {turn_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
