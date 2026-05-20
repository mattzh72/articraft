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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


TOP_PLATE_Z = 0.062
CHANNEL_XS = (-0.108, -0.036, 0.036, 0.108)
EQ_LAYOUT = (
    ("high_eq", 0.082),
    ("mid_eq", 0.047),
    ("low_eq", 0.012),
)
VOLUME_Y = -0.065
CROSSFADER_Y = -0.103


def _joint_name(part_name: str) -> str:
    return f"chassis_to_{part_name}"


def _build_knob_mesh(
    logical_name: str,
    *,
    radius: float,
    height: float,
    top_radius_scale: float,
):
    profile = [
        (0.0, 0.0),
        (radius * 0.95, 0.0),
        (radius, height * 0.14),
        (radius * 0.94, height * 0.46),
        (radius * (0.90 + 0.08 * top_radius_scale), height * 0.78),
        (radius * top_radius_scale, height * 0.95),
        (radius * top_radius_scale * 0.78, height),
        (0.0, height),
    ]
    return mesh_from_geometry(LatheGeometry(profile, segments=48), logical_name)


def _all_channel_knob_names() -> list[str]:
    names: list[str] = []
    for channel_index in range(1, 5):
        for band_name, _ in EQ_LAYOUT:
            names.append(f"channel_{channel_index}_{band_name}_knob")
        names.append(f"channel_{channel_index}_volume_knob")
    return names


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rotary_dj_mixer")

    body_black = model.material("body_black", rgba=(0.10, 0.11, 0.12, 1.0))
    faceplate = model.material("faceplate", rgba=(0.73, 0.74, 0.76, 1.0))
    cheek_wood = model.material("cheek_wood", rgba=(0.34, 0.23, 0.14, 1.0))
    knob_black = model.material("knob_black", rgba=(0.13, 0.14, 0.15, 1.0))
    indicator_white = model.material("indicator_white", rgba=(0.91, 0.92, 0.93, 1.0))
    meter_glass = model.material("meter_glass", rgba=(0.20, 0.36, 0.32, 0.55))
    foot_rubber = model.material("foot_rubber", rgba=(0.07, 0.07, 0.08, 1.0))
    channel_orange = model.material("channel_orange", rgba=(0.86, 0.46, 0.17, 1.0))
    channel_blue = model.material("channel_blue", rgba=(0.18, 0.48, 0.82, 1.0))
    channel_green = model.material("channel_green", rgba=(0.25, 0.67, 0.39, 1.0))
    channel_red = model.material("channel_red", rgba=(0.77, 0.23, 0.22, 1.0))

    chassis = model.part("chassis")
    shell_mesh = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.350, 0.270, 0.028), 0.052),
        "mixer_shell",
    )
    chassis.visual(
        shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=body_black,
        name="shell",
    )
    chassis.visual(
        Box((0.318, 0.238, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=faceplate,
        name="top_plate",
    )
    chassis.visual(
        Box((0.118, 0.026, 0.018)),
        origin=Origin(xyz=(0.0, 0.110, 0.067)),
        material=faceplate,
        name="rear_bridge",
    )
    chassis.visual(
        Box((0.080, 0.014, 0.002)),
        origin=Origin(xyz=(0.0, 0.110, 0.077)),
        material=meter_glass,
        name="meter_window",
    )
    chassis.visual(
        Box((0.016, 0.230, 0.036)),
        origin=Origin(xyz=(-0.167, 0.0, 0.040)),
        material=cheek_wood,
        name="left_cheek",
    )
    chassis.visual(
        Box((0.016, 0.230, 0.036)),
        origin=Origin(xyz=(0.167, 0.0, 0.040)),
        material=cheek_wood,
        name="right_cheek",
    )
    chassis.visual(
        Box((0.240, 0.010, 0.001)),
        origin=Origin(xyz=(0.0, -0.111, 0.0625)),
        material=faceplate,
        name="front_trim",
    )

    for foot_index, (foot_x, foot_y) in enumerate(
        ((-0.118, -0.088), (0.118, -0.088), (-0.118, 0.088), (0.118, 0.088))
    ):
        chassis.visual(
            Box((0.030, 0.020, 0.006)),
            origin=Origin(xyz=(foot_x, foot_y, 0.003)),
            material=foot_rubber,
            name=f"foot_{foot_index + 1}",
        )

    channel_materials = (channel_orange, channel_blue, channel_green, channel_red)
    for channel_index, (channel_x, label_material) in enumerate(
        zip(CHANNEL_XS, channel_materials),
        start=1,
    ):
        chassis.visual(
            Box((0.024, 0.008, 0.001)),
            origin=Origin(xyz=(channel_x, 0.113, 0.0625)),
            material=label_material,
            name=f"channel_{channel_index}_label",
        )

    chassis.inertial = Inertial.from_geometry(
        Box((0.350, 0.270, 0.078)),
        mass=5.8,
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
    )

    eq_knob_mesh = _build_knob_mesh(
        "eq_knob_body",
        radius=0.012,
        height=0.018,
        top_radius_scale=0.78,
    )
    volume_knob_mesh = _build_knob_mesh(
        "volume_knob_body",
        radius=0.015,
        height=0.021,
        top_radius_scale=0.82,
    )
    crossfader_knob_mesh = _build_knob_mesh(
        "crossfader_knob_body",
        radius=0.026,
        height=0.029,
        top_radius_scale=0.72,
    )

    def add_rotary_knob(
        *,
        part_name: str,
        mesh,
        x: float,
        y: float,
        radius: float,
        height: float,
        mass: float,
        velocity: float,
        effort: float,
        indicator_length_scale: float = 0.62,
    ) -> None:
        knob_part = model.part(part_name)
        knob_part.visual(
            mesh,
            material=knob_black,
            name="knob_body",
        )
        knob_part.visual(
            Box((radius * indicator_length_scale, max(radius * 0.16, 0.0018), 0.0012)),
            origin=Origin(
                xyz=(radius * indicator_length_scale * 0.5, 0.0, height - 0.0006)
            ),
            material=indicator_white,
            name="indicator",
        )
        knob_part.inertial = Inertial.from_geometry(
            Cylinder(radius=radius, length=height),
            mass=mass,
            origin=Origin(xyz=(0.0, 0.0, height * 0.5)),
        )
        model.articulation(
            _joint_name(part_name),
            ArticulationType.CONTINUOUS,
            parent=chassis,
            child=knob_part,
            origin=Origin(xyz=(x, y, TOP_PLATE_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=effort,
                velocity=velocity,
            ),
        )

    for channel_index, channel_x in enumerate(CHANNEL_XS, start=1):
        for band_name, band_y in EQ_LAYOUT:
            add_rotary_knob(
                part_name=f"channel_{channel_index}_{band_name}_knob",
                mesh=eq_knob_mesh,
                x=channel_x,
                y=band_y,
                radius=0.012,
                height=0.018,
                mass=0.018,
                velocity=16.0,
                effort=0.14,
            )

        add_rotary_knob(
            part_name=f"channel_{channel_index}_volume_knob",
            mesh=volume_knob_mesh,
            x=channel_x,
            y=VOLUME_Y,
            radius=0.015,
            height=0.021,
            mass=0.024,
            velocity=14.0,
            effort=0.18,
            indicator_length_scale=0.68,
        )

    add_rotary_knob(
        part_name="rotary_crossfader_knob",
        mesh=crossfader_knob_mesh,
        x=0.0,
        y=CROSSFADER_Y,
        radius=0.026,
        height=0.029,
        mass=0.058,
        velocity=12.0,
        effort=0.30,
        indicator_length_scale=0.72,
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

    chassis = object_model.get_part("chassis")
    top_plate = chassis.get_visual("top_plate")

    all_knob_names = _all_channel_knob_names() + ["rotary_crossfader_knob"]

    for knob_name in all_knob_names:
        knob_part = object_model.get_part(knob_name)
        knob_joint = object_model.get_articulation(_joint_name(knob_name))
        limits = knob_joint.motion_limits

        ctx.check(
            f"{knob_joint.name} is continuous",
            knob_joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"type={knob_joint.articulation_type}",
        )
        ctx.check(
            f"{knob_joint.name} spins about +Z",
            knob_joint.axis == (0.0, 0.0, 1.0),
            details=f"axis={knob_joint.axis}",
        )
        ctx.check(
            f"{knob_joint.name} has unbounded continuous limits",
            limits is not None and limits.lower is None and limits.upper is None,
            details=f"limits={limits}",
        )
        ctx.expect_gap(
            knob_part,
            chassis,
            axis="z",
            max_gap=0.0005,
            max_penetration=0.0,
            negative_elem=top_plate,
            name=f"{knob_name} sits on the top plate",
        )

    crossfader = object_model.get_part("rotary_crossfader_knob")
    crossfader_joint = object_model.get_articulation(_joint_name("rotary_crossfader_knob"))
    first_eq = object_model.get_part("channel_1_high_eq_knob")
    first_eq_joint = object_model.get_articulation(_joint_name("channel_1_high_eq_knob"))

    def indicator_center(part_name: str):
        part = object_model.get_part(part_name)
        aabb = ctx.part_element_world_aabb(part, elem="indicator")
        if aabb is None:
            return None
        return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))

    def part_origin(part_name: str):
        return ctx.part_world_position(object_model.get_part(part_name))

    with ctx.pose({first_eq_joint: 0.0, crossfader_joint: 0.0}):
        eq_rest_indicator = indicator_center("channel_1_high_eq_knob")
        cross_rest_indicator = indicator_center("rotary_crossfader_knob")
        eq_rest_origin = part_origin("channel_1_high_eq_knob")
        cross_rest_origin = part_origin("rotary_crossfader_knob")

    with ctx.pose({first_eq_joint: math.pi / 2.0, crossfader_joint: math.pi / 2.0}):
        eq_quarter_indicator = indicator_center("channel_1_high_eq_knob")
        cross_quarter_indicator = indicator_center("rotary_crossfader_knob")
        eq_quarter_origin = part_origin("channel_1_high_eq_knob")
        cross_quarter_origin = part_origin("rotary_crossfader_knob")

    ctx.check(
        "EQ knob indicator starts on the positive X side",
        eq_rest_indicator is not None
        and eq_rest_origin is not None
        and eq_rest_indicator[0] > eq_rest_origin[0] + 0.002,
        details=f"indicator={eq_rest_indicator}, origin={eq_rest_origin}",
    )
    ctx.check(
        "EQ knob quarter turn moves indicator to the positive Y side",
        eq_quarter_indicator is not None
        and eq_quarter_origin is not None
        and eq_quarter_indicator[1] > eq_quarter_origin[1] + 0.002,
        details=f"indicator={eq_quarter_indicator}, origin={eq_quarter_origin}",
    )
    ctx.check(
        "Crossfader knob indicator starts on the positive X side",
        cross_rest_indicator is not None
        and cross_rest_origin is not None
        and cross_rest_indicator[0] > cross_rest_origin[0] + 0.006,
        details=f"indicator={cross_rest_indicator}, origin={cross_rest_origin}",
    )
    ctx.check(
        "Crossfader quarter turn moves indicator to the positive Y side",
        cross_quarter_indicator is not None
        and cross_quarter_origin is not None
        and cross_quarter_indicator[1] > cross_quarter_origin[1] + 0.006,
        details=f"indicator={cross_quarter_indicator}, origin={cross_quarter_origin}",
    )

    ctx.expect_origin_distance(
        crossfader,
        chassis,
        axes="x",
        max_dist=0.003,
        name="crossfader stays centered on the mixer",
    )

    volume_positions = [
        ctx.part_world_position(object_model.get_part(f"channel_{channel_index}_volume_knob"))
        for channel_index in range(1, 5)
    ]
    ctx.check(
        "channel volume knobs are laid out left to right",
        all(position is not None for position in volume_positions)
        and all(
            volume_positions[index][0] < volume_positions[index + 1][0] - 0.04
            for index in range(3)
        ),
        details=f"volume_positions={volume_positions}",
    )
    ctx.expect_origin_gap(
        object_model.get_part("channel_2_volume_knob"),
        crossfader,
        axis="y",
        min_gap=0.03,
        name="crossfader sits forward of the channel volume row",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
