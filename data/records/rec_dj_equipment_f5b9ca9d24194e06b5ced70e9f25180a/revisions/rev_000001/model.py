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


def _offset_profile(
    profile: list[tuple[float, float]],
    dx: float,
    dy: float,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="four_channel_dj_mixer")

    housing_width = 0.320
    housing_depth = 0.255
    housing_height = 0.046
    wall_thickness = 0.006
    top_thickness = 0.0035
    bottom_thickness = 0.003
    slot_width = 0.006
    slot_length = 0.116
    slot_travel = 0.045
    cross_slot_width = 0.007
    cross_slot_length = 0.108
    cross_travel = 0.045
    top_surface_z = housing_height
    slider_clearance = 0.0

    model.material("chassis_black", rgba=(0.12, 0.12, 0.13, 1.0))
    model.material("panel_black", rgba=(0.06, 0.06, 0.07, 1.0))
    model.material("trim_dark", rgba=(0.18, 0.18, 0.20, 1.0))
    model.material("knob_dark", rgba=(0.14, 0.14, 0.15, 1.0))
    model.material("fader_cream", rgba=(0.92, 0.91, 0.86, 1.0))
    model.material("fader_orange", rgba=(0.84, 0.42, 0.14, 1.0))
    model.material("fader_blue", rgba=(0.18, 0.46, 0.82, 1.0))
    model.material("fader_green", rgba=(0.27, 0.62, 0.29, 1.0))

    housing = model.part("housing")

    channel_slot_xs = (-0.102, -0.034, 0.034, 0.102)
    channel_slot_y = -0.002
    cross_slot_y = -0.056

    top_panel_geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(housing_width, housing_depth, radius=0.018),
        [
            _offset_profile(
                rounded_rect_profile(slot_width, slot_length, radius=slot_width * 0.5),
                slot_x,
                channel_slot_y,
            )
            for slot_x in channel_slot_xs
        ]
        + [
            _offset_profile(
                rounded_rect_profile(
                    cross_slot_length,
                    cross_slot_width,
                    radius=cross_slot_width * 0.5,
                ),
                0.0,
                cross_slot_y,
            )
        ],
        top_thickness,
        cap=True,
        center=True,
        closed=True,
    )
    housing.visual(
        mesh_from_geometry(top_panel_geom, "dj_mixer_top_panel"),
        origin=Origin(xyz=(0.0, 0.0, housing_height - top_thickness * 0.5)),
        material="panel_black",
        name="top_panel",
    )
    housing.visual(
        Box((housing_width - 2.0 * wall_thickness, housing_depth - 2.0 * wall_thickness, bottom_thickness)),
        origin=Origin(xyz=(0.0, 0.0, bottom_thickness * 0.5)),
        material="trim_dark",
        name="bottom_panel",
    )
    housing.visual(
        Box((wall_thickness, housing_depth - 0.020, housing_height - top_thickness - bottom_thickness)),
        origin=Origin(
            xyz=(
                -housing_width * 0.5 + wall_thickness * 0.5,
                0.0,
                bottom_thickness + (housing_height - top_thickness - bottom_thickness) * 0.5,
            )
        ),
        material="chassis_black",
        name="left_wall",
    )
    housing.visual(
        Box((wall_thickness, housing_depth - 0.020, housing_height - top_thickness - bottom_thickness)),
        origin=Origin(
            xyz=(
                housing_width * 0.5 - wall_thickness * 0.5,
                0.0,
                bottom_thickness + (housing_height - top_thickness - bottom_thickness) * 0.5,
            )
        ),
        material="chassis_black",
        name="right_wall",
    )
    housing.visual(
        Box((housing_width - 0.020, wall_thickness, housing_height - top_thickness - bottom_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                -housing_depth * 0.5 + wall_thickness * 0.5,
                bottom_thickness + (housing_height - top_thickness - bottom_thickness) * 0.5,
            )
        ),
        material="chassis_black",
        name="front_wall",
    )
    housing.visual(
        Box((housing_width - 0.020, wall_thickness, housing_height - top_thickness - bottom_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                housing_depth * 0.5 - wall_thickness * 0.5,
                bottom_thickness + (housing_height - top_thickness - bottom_thickness) * 0.5,
            )
        ),
        material="chassis_black",
        name="back_wall",
    )
    housing.visual(
        Box((housing_width - 0.010, 0.016, 0.006)),
        origin=Origin(xyz=(0.0, -housing_depth * 0.5 + 0.014, housing_height - 0.010)),
        material="trim_dark",
        name="front_ledge",
    )
    housing.inertial = Inertial.from_geometry(
        Box((housing_width, housing_depth, housing_height)),
        mass=2.9,
        origin=Origin(xyz=(0.0, 0.0, housing_height * 0.5)),
    )

    channel_materials = (
        "fader_cream",
        "fader_orange",
        "fader_blue",
        "fader_green",
    )

    for index, (slot_x, slider_material) in enumerate(zip(channel_slot_xs, channel_materials), start=1):
        fader = model.part(f"channel_{index}_fader")
        fader.visual(
            Box((0.014, 0.010, 0.017)),
            origin=Origin(xyz=(0.0, 0.0, 0.0085)),
            material=slider_material,
            name="cap_body",
        )
        fader.visual(
            Box((0.010, 0.012, 0.009)),
            origin=Origin(xyz=(0.0, 0.0, 0.0145)),
            material=slider_material,
            name="cap_grip",
        )
        fader.visual(
            Box((0.003, 0.003, 0.018)),
            origin=Origin(xyz=(0.0, 0.0, -0.0085)),
            material="trim_dark",
            name="stem",
        )
        fader.inertial = Inertial.from_geometry(
            Box((0.016, 0.012, 0.036)),
            mass=0.028,
            origin=Origin(xyz=(0.0, 0.0, 0.006)),
        )
        model.articulation(
            f"housing_to_channel_{index}_fader",
            ArticulationType.PRISMATIC,
            parent=housing,
            child=fader,
            origin=Origin(xyz=(slot_x, channel_slot_y, top_surface_z + slider_clearance)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=0.18,
                lower=-slot_travel,
                upper=slot_travel,
            ),
        )

        controls = model.part(f"channel_{index}_controls")
        controls.visual(
            Box((0.026, 0.086, 0.002)),
            origin=Origin(xyz=(0.0, 0.0, 0.001)),
            material="panel_black",
            name="control_strip",
        )
        for knob_index, knob_y in enumerate((-0.026, 0.0, 0.026), start=1):
            controls.visual(
                Cylinder(radius=0.006, length=0.003),
                origin=Origin(xyz=(0.0, knob_y, 0.0035)),
                material="knob_dark",
                name=f"knob_skirt_{knob_index}",
            )
            controls.visual(
                Cylinder(radius=0.005, length=0.010),
                origin=Origin(xyz=(0.0, knob_y, 0.008)),
                material="knob_dark",
                name=f"knob_body_{knob_index}",
            )
        controls.inertial = Inertial.from_geometry(
            Box((0.028, 0.090, 0.018)),
            mass=0.05,
            origin=Origin(xyz=(0.0, 0.0, 0.006)),
        )
        model.articulation(
            f"housing_to_channel_{index}_controls",
            ArticulationType.FIXED,
            parent=housing,
            child=controls,
            origin=Origin(xyz=(slot_x, 0.084, top_surface_z)),
        )

    crossfader = model.part("crossfader")
    crossfader.visual(
        Box((0.026, 0.011, 0.013)),
        origin=Origin(xyz=(0.0, 0.0, 0.0065)),
        material="fader_cream",
        name="cap_body",
    )
    crossfader.visual(
        Box((0.020, 0.013, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material="fader_cream",
        name="cap_grip",
    )
    crossfader.visual(
        Box((0.0035, 0.0035, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.0085)),
        material="trim_dark",
        name="stem",
    )
    crossfader.inertial = Inertial.from_geometry(
        Box((0.028, 0.013, 0.032)),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
    )
    model.articulation(
        "housing_to_crossfader",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=crossfader,
        origin=Origin(xyz=(0.0, cross_slot_y, top_surface_z + slider_clearance)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=0.22,
            lower=-cross_travel,
            upper=cross_travel,
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

    housing = object_model.get_part("housing")
    top_panel = housing.get_visual("top_panel")
    crossfader = object_model.get_part("crossfader")
    cross_joint = object_model.get_articulation("housing_to_crossfader")
    cross_cap = crossfader.get_visual("cap_body")

    ctx.expect_contact(
        crossfader,
        housing,
        elem_a=cross_cap,
        elem_b=top_panel,
        name="crossfader cap rides on the deck surface",
    )

    cross_lower = cross_joint.motion_limits.lower
    cross_upper = cross_joint.motion_limits.upper
    with ctx.pose({cross_joint: cross_lower}):
        cross_left = ctx.part_world_position(crossfader)
        ctx.expect_within(
            crossfader,
            housing,
            axes="xy",
            inner_elem=cross_cap,
            outer_elem=top_panel,
            margin=0.0,
            name="crossfader cap stays on the mixer deck at left throw",
        )
    with ctx.pose({cross_joint: cross_upper}):
        cross_right = ctx.part_world_position(crossfader)
        ctx.expect_within(
            crossfader,
            housing,
            axes="xy",
            inner_elem=cross_cap,
            outer_elem=top_panel,
            margin=0.0,
            name="crossfader cap stays on the mixer deck at right throw",
        )
    ctx.check(
        "crossfader travels horizontally across the center slot",
        cross_left is not None
        and cross_right is not None
        and cross_right[0] > cross_left[0] + 0.08
        and abs(cross_right[1] - cross_left[1]) < 1e-6,
        details=f"left={cross_left}, right={cross_right}",
    )

    for index in range(1, 5):
        fader = object_model.get_part(f"channel_{index}_fader")
        joint = object_model.get_articulation(f"housing_to_channel_{index}_fader")
        cap = fader.get_visual("cap_body")
        controls = object_model.get_part(f"channel_{index}_controls")
        control_strip = controls.get_visual("control_strip")

        ctx.expect_contact(
            fader,
            housing,
            elem_a=cap,
            elem_b=top_panel,
            name=f"channel {index} fader cap rides on the top panel",
        )
        ctx.expect_contact(
            controls,
            housing,
            elem_a=control_strip,
            elem_b=top_panel,
            name=f"channel {index} control strip is mounted to the top panel",
        )

        with ctx.pose({joint: joint.motion_limits.lower}):
            low_pos = ctx.part_world_position(fader)
            ctx.expect_within(
                fader,
                housing,
                axes="xy",
                inner_elem=cap,
                outer_elem=top_panel,
                margin=0.0,
                name=f"channel {index} cap stays on deck at low throw",
            )
        with ctx.pose({joint: joint.motion_limits.upper}):
            high_pos = ctx.part_world_position(fader)
            ctx.expect_within(
                fader,
                housing,
                axes="xy",
                inner_elem=cap,
                outer_elem=top_panel,
                margin=0.0,
                name=f"channel {index} cap stays on deck at high throw",
            )

        ctx.check(
            f"channel {index} fader travels vertically along its slot",
            low_pos is not None
            and high_pos is not None
            and high_pos[1] > low_pos[1] + 0.08
            and abs(high_pos[0] - low_pos[0]) < 1e-6,
            details=f"low={low_pos}, high={high_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
