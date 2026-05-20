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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


BASE_HALF_WIDTH = 0.24
BASE_REAR_Y = -0.26
BASE_FRONT_Y = 0.32
BASE_UNDERSIDE_Z = 0.108
BASE_TUBE_HEIGHT = 0.05
BASE_TUBE_TOP_Z = BASE_UNDERSIDE_Z + BASE_TUBE_HEIGHT
OUTER_SLEEVE_HEIGHT = 0.82
INNER_COLUMN_RADIUS = 0.0245
INNER_COLUMN_LENGTH = 1.20


def _tube_shell_mesh(name: str, outer_radius: float, inner_radius: float, height: float):
    outer_profile = [(outer_radius, 0.0), (outer_radius, height)]
    inner_profile = [(inner_radius, 0.0), (inner_radius, height)]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=48,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def _ring_torus_mesh(name: str, major_radius: float, tube_radius: float, *, y_offset: float):
    geom = TorusGeometry(
        major_radius,
        tube_radius,
        radial_segments=20,
        tubular_segments=64,
    )
    geom.rotate_x(math.pi / 2.0)
    geom.translate(0.0, y_offset, 0.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="salon_ring_light_stand")

    satin_black = model.material("satin_black", rgba=(0.14, 0.15, 0.16, 1.0))
    matte_black = model.material("matte_black", rgba=(0.08, 0.08, 0.09, 1.0))
    warm_white = model.material("warm_white", rgba=(0.94, 0.94, 0.90, 1.0))
    steel = model.material("steel", rgba=(0.69, 0.71, 0.74, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.10, 0.10, 0.10, 1.0))

    base_frame = model.part("base_frame")
    rail_center_z = BASE_UNDERSIDE_Z + BASE_TUBE_HEIGHT / 2.0
    base_frame.visual(
        Box((0.06, 0.58, BASE_TUBE_HEIGHT)),
        origin=Origin(xyz=(-BASE_HALF_WIDTH, 0.03, rail_center_z)),
        material=satin_black,
        name="left_rail",
    )
    base_frame.visual(
        Box((0.06, 0.58, BASE_TUBE_HEIGHT)),
        origin=Origin(xyz=(BASE_HALF_WIDTH, 0.03, rail_center_z)),
        material=satin_black,
        name="right_rail",
    )
    base_frame.visual(
        Box((0.54, 0.06, BASE_TUBE_HEIGHT)),
        origin=Origin(xyz=(0.0, BASE_REAR_Y, rail_center_z)),
        material=satin_black,
        name="rear_crossbar",
    )
    sleeve_center_z = BASE_TUBE_TOP_Z + OUTER_SLEEVE_HEIGHT / 2.0
    base_frame.visual(
        Box((0.008, 0.064, OUTER_SLEEVE_HEIGHT)),
        origin=Origin(xyz=(-0.032, BASE_REAR_Y, sleeve_center_z)),
        material=satin_black,
        name="left_sleeve_wall",
    )
    base_frame.visual(
        Box((0.008, 0.064, OUTER_SLEEVE_HEIGHT)),
        origin=Origin(xyz=(0.032, BASE_REAR_Y, sleeve_center_z)),
        material=satin_black,
        name="right_sleeve_wall",
    )
    base_frame.visual(
        Box((0.064, 0.008, OUTER_SLEEVE_HEIGHT)),
        origin=Origin(xyz=(0.0, BASE_REAR_Y - 0.028, sleeve_center_z)),
        material=satin_black,
        name="rear_sleeve_wall",
    )
    base_frame.visual(
        Box((0.020, 0.020, 0.040)),
        origin=Origin(
            xyz=(0.046, BASE_REAR_Y + 0.024, BASE_TUBE_TOP_Z + OUTER_SLEEVE_HEIGHT - 0.02)
        ),
        material=steel,
        name="height_lock_knob",
    )

    inner_column = model.part("inner_column")
    inner_column.visual(
        Cylinder(radius=INNER_COLUMN_RADIUS, length=INNER_COLUMN_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, INNER_COLUMN_LENGTH / 2.0)),
        material=steel,
        name="telescoping_column",
    )

    model.articulation(
        "base_to_inner_column",
        ArticulationType.PRISMATIC,
        parent=base_frame,
        child=inner_column,
        origin=Origin(xyz=(0.0, BASE_REAR_Y, BASE_TUBE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.25,
            lower=0.0,
            upper=0.35,
        ),
    )

    tilt_bracket = model.part("tilt_bracket")
    tilt_bracket.visual(
        Cylinder(radius=0.021, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=satin_black,
        name="mount_spigot",
    )
    tilt_bracket.visual(
        Box((0.028, 0.06, 0.26)),
        origin=Origin(xyz=(0.0, 0.0, 0.13)),
        material=satin_black,
        name="forward_spine",
    )
    tilt_bracket.visual(
        Box((0.574, 0.03, 0.02)),
        origin=Origin(xyz=(0.0, -0.02, 0.24)),
        material=satin_black,
        name="yoke_bridge",
    )
    tilt_bracket.visual(
        Box((0.018, 0.12, 0.18)),
        origin=Origin(xyz=(-0.278, 0.03, 0.18)),
        material=satin_black,
        name="left_yoke_arm",
    )
    tilt_bracket.visual(
        Box((0.018, 0.12, 0.18)),
        origin=Origin(xyz=(0.278, 0.03, 0.18)),
        material=satin_black,
        name="right_yoke_arm",
    )

    model.articulation(
        "inner_column_to_tilt_bracket",
        ArticulationType.FIXED,
        parent=inner_column,
        child=tilt_bracket,
        origin=Origin(xyz=(0.0, 0.0, INNER_COLUMN_LENGTH)),
    )

    ring_head = model.part("ring_head")
    ring_head.visual(
        _ring_torus_mesh(
            "ring_light_housing",
            major_radius=0.22,
            tube_radius=0.039,
            y_offset=-0.01,
        ),
        material=matte_black,
        name="rear_housing",
    )
    ring_head.visual(
        _ring_torus_mesh(
            "ring_light_diffuser",
            major_radius=0.22,
            tube_radius=0.028,
            y_offset=0.008,
        ),
        material=warm_white,
        name="front_diffuser",
    )
    ring_head.visual(
        Cylinder(radius=0.014, length=0.014),
        origin=Origin(xyz=(-0.262, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="left_trunnion",
    )
    ring_head.visual(
        Cylinder(radius=0.014, length=0.014),
        origin=Origin(xyz=(0.262, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="right_trunnion",
    )

    model.articulation(
        "tilt_bracket_to_ring_head",
        ArticulationType.REVOLUTE,
        parent=tilt_bracket,
        child=ring_head,
        origin=Origin(xyz=(0.0, 0.09, 0.18)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.5,
            lower=-0.65,
            upper=0.85,
        ),
    )

    caster_positions = {
        "rear_left": (-BASE_HALF_WIDTH, BASE_REAR_Y),
        "rear_right": (BASE_HALF_WIDTH, BASE_REAR_Y),
        "front_left": (-BASE_HALF_WIDTH, 0.30),
        "front_right": (BASE_HALF_WIDTH, 0.30),
    }

    for prefix, (x_pos, y_pos) in caster_positions.items():
        caster = model.part(f"{prefix}_caster")
        caster.visual(
            Cylinder(radius=0.01, length=0.04),
            origin=Origin(xyz=(0.0, 0.0, -0.02)),
            material=steel,
            name="swivel_stem",
        )
        caster.visual(
            Box((0.04, 0.024, 0.02)),
            origin=Origin(xyz=(0.0, 0.0, -0.05)),
            material=satin_black,
            name="fork_crown",
        )
        caster.visual(
            Box((0.008, 0.024, 0.04)),
            origin=Origin(xyz=(-0.013, 0.0, -0.08)),
            material=satin_black,
            name="left_fork_leg",
        )
        caster.visual(
            Box((0.008, 0.024, 0.04)),
            origin=Origin(xyz=(0.013, 0.0, -0.08)),
            material=satin_black,
            name="right_fork_leg",
        )

        model.articulation(
            f"base_to_{prefix}_caster",
            ArticulationType.CONTINUOUS,
            parent=base_frame,
            child=caster,
            origin=Origin(xyz=(x_pos, y_pos, BASE_UNDERSIDE_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=12.0),
        )

        wheel = model.part(f"{prefix}_wheel")
        wheel.visual(
            Cylinder(radius=0.028, length=0.018),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=wheel_rubber,
            name="wheel_tire",
        )
        wheel.visual(
            Cylinder(radius=0.015, length=0.022),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name="wheel_hub",
        )

        model.articulation(
            f"{prefix}_caster_to_wheel",
            ArticulationType.CONTINUOUS,
            parent=caster,
            child=wheel,
            origin=Origin(xyz=(0.0, 0.0, -0.092)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=30.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    base_frame = object_model.get_part("base_frame")
    inner_column = object_model.get_part("inner_column")
    tilt_bracket = object_model.get_part("tilt_bracket")
    ring_head = object_model.get_part("ring_head")

    column_slider = object_model.get_articulation("base_to_inner_column")
    ring_tilt = object_model.get_articulation("tilt_bracket_to_ring_head")

    ctx.check(
        "column_slider_axis_is_vertical",
        column_slider.axis == (0.0, 0.0, 1.0),
        f"Expected vertical slider axis, got {column_slider.axis!r}",
    )
    ctx.check(
        "ring_tilt_axis_is_horizontal",
        ring_tilt.axis == (1.0, 0.0, 0.0),
        f"Expected left-right tilt axis, got {ring_tilt.axis!r}",
    )

    ctx.expect_contact(
        inner_column,
        base_frame,
        name="column_is_seated_in_lower_column_stage",
    )
    ctx.expect_gap(
        inner_column,
        base_frame,
        axis="z",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem="telescoping_column",
        negative_elem="rear_crossbar",
        name="column_starts_just_above_rear_crossbar",
    )
    ctx.expect_contact(
        tilt_bracket,
        inner_column,
        name="tilt_bracket_is_mounted_to_column",
    )
    ctx.expect_contact(
        ring_head,
        tilt_bracket,
        name="ring_head_is_clipped_into_tilt_bracket",
    )
    ctx.expect_origin_gap(
        ring_head,
        base_frame,
        axis="z",
        min_gap=1.25,
        name="ring_head_sits_well_above_u_base",
    )

    caster_prefixes = ("rear_left", "rear_right", "front_left", "front_right")
    for prefix in caster_prefixes:
        caster = object_model.get_part(f"{prefix}_caster")
        wheel = object_model.get_part(f"{prefix}_wheel")
        swivel = object_model.get_articulation(f"base_to_{prefix}_caster")
        spin = object_model.get_articulation(f"{prefix}_caster_to_wheel")

        ctx.check(
            f"{prefix}_caster_swivel_axis_is_vertical",
            swivel.axis == (0.0, 0.0, 1.0),
            f"Expected vertical caster swivel axis, got {swivel.axis!r}",
        )
        ctx.check(
            f"{prefix}_wheel_spin_axis_is_horizontal",
            spin.axis == (1.0, 0.0, 0.0),
            f"Expected horizontal wheel axle axis, got {spin.axis!r}",
        )
        ctx.expect_contact(
            caster,
            base_frame,
            name=f"{prefix}_caster_mounts_under_base",
        )
        ctx.expect_contact(
            wheel,
            caster,
            name=f"{prefix}_wheel_is_captured_in_fork",
        )
        ctx.expect_gap(
            base_frame,
            wheel,
            axis="z",
            min_gap=0.03,
            max_gap=0.08,
            name=f"{prefix}_wheel_hangs_below_frame",
        )

    rest_column_pos = ctx.part_world_position(inner_column)
    with ctx.pose({column_slider: 0.30}):
        raised_column_pos = ctx.part_world_position(inner_column)
        ctx.check(
            "column_slider_raises_head",
            rest_column_pos is not None
            and raised_column_pos is not None
            and raised_column_pos[2] > rest_column_pos[2] + 0.29,
            (
                f"Expected raised column origin to move up by about 0.30 m, got "
                f"{rest_column_pos!r} -> {raised_column_pos!r}"
            ),
        )

    with ctx.pose({column_slider: 0.18, ring_tilt: 0.55}):
        ctx.expect_contact(
            ring_head,
            tilt_bracket,
            name="ring_head_stays_supported_while_tilted",
        )
        ctx.expect_gap(
            ring_head,
            base_frame,
            axis="z",
            min_gap=0.10,
            name="tilted_ring_head_clears_stand",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
